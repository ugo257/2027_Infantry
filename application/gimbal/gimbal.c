/*------------------------------------------------------------------------------*/
#include "stdio.h"//标准库
#include <math.h>
/*------------------------------------------------------------------------------*/
#include "gimbal.h"//拿到本模块对外接口 GimbalInit(),GimbalTask()      
#include "robot_board.h"//根据 CHASSIS_BOARD / ONE_BOARD 决定编译哪部分代码
#include "robot_params.h"//读云台几何参数、限位、控制宏定义等
#include "robot_types.h"//
#include "robot_def.h"//云台 SMC 开关与参数接口
/*------------------------------------------------------------------------------*/
#include "dji_motor.h"//分别给四个轮毂电机和两个关节电机提供驱动接口
#include "DMmotor.h"//DM电机的接口
/*------------------------------------------------------------------------------*/
#include "ins_task.h"//使用 IMU/INS 解算结果
/*------------------------------------------------------------------------------*/
#include "message_center.h"//云台和上层命令模块通过发布/订阅通信
/*------------------------------------------------------------------------------*/
#include "general_def.h"//一些module的通用数值型定义
/*------------------------------------------------------------------------------*/
#include "bmi088.h"//IMU传感器接口
#include "referee_UI.h"//裁判系统数据解析，提供给UI显示用
#include "controller.h"//PID控制器实现
/*------------------------------------------------------------------------------*/

static INS_Instance *gimbal_IMU_data;               //保存云台 IMU/INS 实例指针
DJIMotorInstance *yaw_motor;                        //云台 yaw 轴的 DJI 电机对象
DMMotorInstance *pitch_motor;                       //云台 pitch 轴的 DM 电机对象
DJIMotorInstance *pitch_version;                    //云台 pitch 轴的 DJI 电机对象(未使用)
static Publisher_t *gimbal_pub;                     //用于发布云台的数据
static Subscriber_t *gimbal_sub;                    //用于订阅云台的控制命令
static Gimbal_Upload_Data_s gimbal_feedback_data;   //云台要回传给别的模块的数据
static Gimbal_Ctrl_Cmd_s gimbal_cmd_recv;           //云台接收到的控制命令
static float pitch_speed_feedforward = 0;           //俯仰轴速度前馈
static float yaw_speedFeed;
//static float yaw_angle_imu,yaw_gyro_imu;
extern Chassis_Ctrl_Cmd_s_uart chassis_rs485_recv;  //表示从另一块板/串口收到的数据
static float yaw_speed_feedforward = 0;             //yaw轴速度前馈
static float filtered_yaw_vel = 0;                  //保存滤波后的视觉 yaw 速度
static const float yaw_vel_filter_alpha = 0.12f;    //一阶低通滤波系数，数值越小滤波效果越明显，但响应越慢
static const float yaw_vel_reverse_alpha = 0.45f;   //视觉速度换向时加快滤波收敛，减少前馈拖尾

static const float yaw_vel_deadzone = 0.1f;         //yaw 速度死区
extern float vision_yaw_vel;                        //视觉给出的 yaw 速度
static float yaw_feedforward_vel_gain = -0.8f;      //定义 yaw 速度前馈增益
static const float yaw_feedforward_err_cutoff = 0.20f;
static const float yaw_feedforward_err_full = 3.0f;
static const float yaw_speed_feedforward_limit = 80.0f;
const float pitch_offset = 7.7f;                    //作为 pitch 偏置量
float yaw_gyro_twoboard = 0, yaw_current_feedforward;  //yaw轴电流前馈
float pitch_current_feedforward, K_pitch_current_feedforward, B_pitch_current_feedforward;//俯仰轴电流前馈的 PID 参数
float pitch_tor_feedforward = 0;                    //保存 pitch 力矩前馈
float pitch_gyro_measure = 0;                       //保存 pitch 轴陀螺仪测量值
/* Pitch gravity compensation (DM torque domain) */
#define PITCH_GRAVITY_FF_ONEKEY_GAIN (-1.2f)   // one-key overall gain
#define PITCH_GRAVITY_FF_BASE_AMP    (2.20f)  // gravity compensation amplitude
#define PITCH_GRAVITY_FF_ANGLE_BIAS  (0.0f)   // rad
#define PITCH_GRAVITY_FF_DAMPING_K   (-0.05f)  // damping on pitch gyro(rad/s)
#define PITCH_GRAVITY_FF_MIN         (-3.0f)  // torque feedforward lower limit
#define PITCH_GRAVITY_FF_MAX         (3.0f)   // torque feedforward upper limit

static float clampf_local(float x, float min, float max)
{
    if (x < min)
        return min;
    if (x > max)
        return max;
    return x;
}

static float YawVisionFeedforwardScale(float yaw_error_deg)
{
    const float err_abs = fabsf(yaw_error_deg);

    if (err_abs <= yaw_feedforward_err_cutoff)
        return 0.0f;
    if (err_abs >= yaw_feedforward_err_full)
        return 1.0f;

    return (err_abs - yaw_feedforward_err_cutoff) /
           (yaw_feedforward_err_full - yaw_feedforward_err_cutoff);
}

static float PitchGravityTorqueFeedforward(float pitch_angle_rad, float pitch_gyro_rads, float motor_pos_rad)
{
#if GIMBAL_PITCH_GRAVITY_USE_MOTOR_POS_FIT
    const float dx = motor_pos_rad - GIMBAL_PITCH_GRAVITY_POS_CENTER;
    const float gravity_fit = GIMBAL_PITCH_GRAVITY_FIT_C0 +
                              GIMBAL_PITCH_GRAVITY_FIT_C1 * dx +
                              GIMBAL_PITCH_GRAVITY_FIT_C2 * dx * dx;
    const float ff_raw = gravity_fit - GIMBAL_PITCH_GRAVITY_DAMPING_GAIN * pitch_gyro_rads;
    (void)pitch_angle_rad;
    return clampf_local(ff_raw, GIMBAL_PITCH_GRAVITY_FIT_MIN, GIMBAL_PITCH_GRAVITY_FIT_MAX);
#else
    const float gravity_term = PITCH_GRAVITY_FF_BASE_AMP * sinf(pitch_angle_rad - PITCH_GRAVITY_FF_ANGLE_BIAS);
    const float damping_term = PITCH_GRAVITY_FF_DAMPING_K * pitch_gyro_rads;
    const float ff_raw       = PITCH_GRAVITY_FF_ONEKEY_GAIN * (gravity_term - damping_term);
    (void)motor_pos_rad;
    return clampf_local(ff_raw, PITCH_GRAVITY_FF_MIN, PITCH_GRAVITY_FF_MAX);
#endif
}

static float PitchLinkageSmcScale(float crank_angle_rad)
{
    float effective_sin = fabsf(sinf(crank_angle_rad - GIMBAL_PITCH_LINKAGE_CRANK_ZERO_RAD));

    if (effective_sin <= GIMBAL_PITCH_LINKAGE_DEADZONE_SIN)
        return 0.0f;

    return clampf_local((effective_sin - GIMBAL_PITCH_LINKAGE_DEADZONE_SIN) /
                        (1.0f - GIMBAL_PITCH_LINKAGE_DEADZONE_SIN),
                        0.0f,
                        1.0f);
}

typedef struct {
    float lambda;
    float ki;
    float linear_k;
    float switch_k;
    float boundary;
    float out_limit;
    float output_sign;
    float out_filter;
    float integral_limit;
    float ref_vel_filter;
    float err_deadband;
    float vel_deadband;
    float surface_deadband;
    float startup_step;
    uint8_t wrap_angle;
} GimbalSMCConfig_t;

typedef struct {
    float err_integral;
    float ref_last;
    float ref_vel;
    float output;
    float startup_scale;
    uint8_t inited;
} GimbalSMCState_t;

static GimbalSMCState_t yaw_smc_state;
static GimbalSMCState_t pitch_smc_state;

static float GimbalWrapAngle180(float angle)
{
    while (angle > 180.0f)
        angle -= 360.0f;
    while (angle < -180.0f)
        angle += 360.0f;
    return angle;
}

static void GimbalSMCReset(GimbalSMCState_t *state)
{
    state->err_integral = 0.0f;
    state->ref_last = 0.0f;
    state->ref_vel = 0.0f;
    state->output = 0.0f;
    state->startup_scale = 0.0f;
    state->inited = 0u;
}

static float GimbalSMCSat(float x)
{
    return clampf_local(x, -1.0f, 1.0f);
}

static float GimbalSMCCalculate(GimbalSMCState_t *state,
                                const GimbalSMCConfig_t *cfg,
                                float ref,
                                float measure,
                                float measure_vel)
{
    float err;
    float ref_delta;
    float ref_vel_raw;
    float err_dot;
    float surface;
    float output_target;

    if (!state->inited) {
        state->ref_last = ref;
        state->ref_vel = 0.0f;
        state->output = 0.0f;
        state->err_integral = 0.0f;
        state->startup_scale = 0.0f;
        state->inited = 1u;
    }

    err = ref - measure;
    ref_delta = ref - state->ref_last;
    if (cfg->wrap_angle) {
        err = GimbalWrapAngle180(err);
        ref_delta = GimbalWrapAngle180(ref_delta);
    }
    if (fabsf(err) < cfg->err_deadband)
        err = 0.0f;

    ref_vel_raw = ref_delta / GIMBAL_SMC_CTRL_DT;
    state->ref_vel += cfg->ref_vel_filter * (ref_vel_raw - state->ref_vel);
    if (fabsf(measure_vel) < cfg->vel_deadband)
        measure_vel = 0.0f;
    err_dot = state->ref_vel - measure_vel;

    state->err_integral += err * GIMBAL_SMC_CTRL_DT;
    state->err_integral = clampf_local(state->err_integral, -cfg->integral_limit, cfg->integral_limit);

    surface = err_dot + cfg->lambda * err + cfg->ki * state->err_integral;
    if (fabsf(surface) < cfg->surface_deadband) {
        output_target = 0.0f;
    } else {
        output_target = cfg->output_sign *
                        (cfg->linear_k * surface + cfg->switch_k * GimbalSMCSat(surface / cfg->boundary));
    }
    output_target = clampf_local(output_target, -cfg->out_limit, cfg->out_limit);
    if (state->startup_scale < 1.0f) {
        state->startup_scale += cfg->startup_step;
        if (state->startup_scale > 1.0f)
            state->startup_scale = 1.0f;
    }
    output_target *= state->startup_scale;

    state->output += cfg->out_filter * (output_target - state->output);
    state->ref_last = ref;
    return state->output;
}

static const GimbalSMCConfig_t yaw_smc_config = {
    .lambda = GIMBAL_YAW_SMC_LAMBDA,
    .ki = GIMBAL_YAW_SMC_KI,
    .linear_k = GIMBAL_YAW_SMC_LINEAR_K,
    .switch_k = GIMBAL_YAW_SMC_SWITCH_K,
    .boundary = GIMBAL_YAW_SMC_BOUNDARY,
    .out_limit = GIMBAL_YAW_SMC_OUT_LIMIT,
    .output_sign = GIMBAL_YAW_SMC_OUTPUT_SIGN,
    .out_filter = GIMBAL_YAW_SMC_FILTER,
    .integral_limit = GIMBAL_YAW_SMC_INT_LIMIT,
    .ref_vel_filter = GIMBAL_YAW_SMC_REF_VEL_FILTER,
    .err_deadband = GIMBAL_YAW_SMC_ERR_DEADBAND,
    .vel_deadband = GIMBAL_YAW_SMC_VEL_DEADBAND,
    .surface_deadband = GIMBAL_YAW_SMC_SURFACE_DEADBAND,
    .startup_step = GIMBAL_YAW_SMC_STARTUP_STEP,
    .wrap_angle = 1u,
};

static const GimbalSMCConfig_t pitch_smc_config = {
    .lambda = GIMBAL_PITCH_SMC_LAMBDA,
    .ki = GIMBAL_PITCH_SMC_KI,
    .linear_k = GIMBAL_PITCH_SMC_LINEAR_K,
    .switch_k = GIMBAL_PITCH_SMC_SWITCH_K,
    .boundary = GIMBAL_PITCH_SMC_BOUNDARY,
    .out_limit = GIMBAL_PITCH_SMC_OUT_LIMIT,
    .output_sign = GIMBAL_PITCH_SMC_OUTPUT_SIGN,
    .out_filter = GIMBAL_PITCH_SMC_FILTER,
    .integral_limit = GIMBAL_PITCH_SMC_INT_LIMIT,
    .ref_vel_filter = GIMBAL_PITCH_SMC_REF_VEL_FILTER,
    .err_deadband = GIMBAL_PITCH_SMC_ERR_DEADBAND,
    .vel_deadband = GIMBAL_PITCH_SMC_VEL_DEADBAND,
    .surface_deadband = GIMBAL_PITCH_SMC_SURFACE_DEADBAND,
    .startup_step = GIMBAL_PITCH_SMC_STARTUP_STEP,
    .wrap_angle = 0u,
};
// static PID_Setting_s pitch_settings,yaw_settings;//保存 pitch 和 yaw 的 PID 设置参数
// extern float imu_angle[3];                       //IMU 角度数组
// extern float imu_gyro[3];                        //IMU 角速度数组
/*--------------------------------------------------------------------------------------------------------------------------*/
//currentFromAngle():用于把角度映射成电流值
float currentFromAngle(float x){
    float y=-0.0011*pow(x,5)+0.0623*pow(x,4)-1.2218*pow(x,3)+7.4649*pow(x,2)-42*x+6776;

    return -y;
}
/*--------------------------------------------------------------------------------------------------------------------------*/

uint16_t powerLim;//根据电池电量计算出的功率限制值，单位为百分比
// static float getYawSpeedFeed(uint16_t power){
//     switch (power)
//     {
//     case 70:
//         return 1.25;
//         break;
//         case 75:
//         return 1.25;
//         break;
//         case 80:
//         return 1.4;
//         break;
//         case 85:
//         return 1.4;
//         break;
//         case 90:
//         return 1.4;
//         break;
//         case 95:
//         return 1.4;
//         break;
//         case 100:
//         return 1.5;
//         break;
//         case 105:
//         return 1.5;
//         break;
//         case 110:
//         return 1.5;
//         break;
//         case 120:
//         return 1.6;
//         break;
//         case 200:
//         return 1.7;
//         break;
//     default:return 0;
//         break;
//     }
// }
/*------------------------------------------------------------------------云台初始化函数---------------------------------------------------------------------------------------*/
void GimbalInit()
{
    #if defined(ONE_BOARD) || defined(CHASSIS_BOARD)

/*-----------------------------------------------------------YAW轴6020电机的配置，包含 PID 参数设置和 CAN 通信设置---------------------------------------------------------------*/
    Motor_Init_Config_s yaw_config = {
        .can_init_config = {
            .can_handle = &hcan1,
            .tx_id      = 1,                                                                                    //配置CAN发送ID,挂在can1上
        },
        .controller_param_init_config = {
            .angle_PID = {
                .Kp            = 3,//1.7,//1.8,//0.6, // 0.24, // 0.31, // 0.45
                .Ki            = 1,
                .Kd            = 0.1f,//0.13,//0.07,
                .DeadBand      = 0.0f,
                .Improve       = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,  //开启梯形积分、积分限幅和微分先行
                .IntegralLimit = 5,                                                                             //积分限幅
                .MaxOut = 100,                                                                                  //输出限幅
            },
            .speed_PID = {
                .Kp            = 2500, // 18000, // 10500,//1000,//10000,// 11000
                .Ki            = 0,    // 0
                .Kd            = 10,    // 10, // 30
                .Improve       = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,// | PID_OutputFilter,
                .IntegralLimit = 500,
                .MaxOut        = 10000,//16384,//25000, // 20000
                // .Output_LPF_RC=1,//0.4,
                // .CoefA=0.2,
                // .CoefB=2,//0.3,
            },
            .other_angle_feedback_ptr = &chassis_rs485_recv.yaw_angle,//yaw 角度反馈不取电机编码器，而取 chassis_rs485_recv.yaw_angle 这个“外部反馈
            //&gimbal_IMU_data->output.INS_angle_deg[INS_YAW_ADDRESS_OFFSET], 
            // ins_task.md bodyframe
            .other_speed_feedback_ptr = &chassis_rs485_recv.yaw_gyro,
            //&yaw_gyro_twoboard,//&chassis_rs485_recv.yaw_gyro-&gimbal_IMU_data->INS_data.INS_gyro[INS_YAW_ADDRESS_OFFSET],//,&gimbal_IMU_data->INS_data.INS_gyro[INS_YAW_ADDRESS_OFFSET],
            .speed_feedforward_ptr = &yaw_speed_feedforward,
            .current_feedforward_ptr = &yaw_current_feedforward,
        },
        .controller_setting_init_config = {
            .angle_feedback_source = OTHER_FEED,
            .speed_feedback_source = OTHER_FEED,
            .outer_loop_type       = ANGLE_LOOP,
            .close_loop_type       = ANGLE_LOOP | SPEED_LOOP,
            .motor_reverse_flag    = MOTOR_DIRECTION_REVERSE,
            // .feedback_reverse_flag = FEEDBACK_DIRECTION_REVERSE,
            .feedforward_flag      = SPEED_FEEDFORWARD | CURRENT_FEEDFORWARD,  //把速度前馈和电流前馈指针分别绑到 yaw_speed_feedforward、yaw_current_feedforward
        },
        .motor_type = GM6020};
        yaw_motor   = DJIMotorInit(&yaw_config);
        #endif
/*----------------------------------------------------------------------BMI088陀螺仪配置--------------------------------------------------------------------------------------*/
//BMI088 配置，包括中断脚、加热 PID、PWM、SPI 片选、标定模式、工作模式等，最后调用 INS_Init(BMI088Register(&config)) 来初始化 IMU/INS 实例
#if defined(ONE_BOARD) || defined(GIMBAL_BOARD)
    BMI088_Init_Config_s config = {
        .acc_int_config  = {.GPIOx = GPIOC, .GPIO_Pin = GPIO_PIN_4},
        .gyro_int_config = {.GPIOx = GPIOC, .GPIO_Pin = GPIO_PIN_5},
        .heat_pid_config = {
            .Kp            = 0.32f,
            .Ki            = 0.0004f,
            .Kd            = 0,
            .Improve       = PID_IMPROVE_NONE,
            .IntegralLimit = 0.90f,
            .MaxOut        = 0.95f,
        },
        .heat_pwm_config = {
            .htim      = &htim10,
            .channel   = TIM_CHANNEL_1,
            .dutyratio = 0,
            .period    = 5000 - 1,
        },
        .spi_acc_config = {
            .GPIOx      = GPIOA,
            .cs_pin     = GPIO_PIN_4,
            .spi_handle = &hspi1,
        },
        .spi_gyro_config = {
            .GPIOx      = GPIOB,
            .cs_pin     = GPIO_PIN_0,
            .spi_handle = &hspi1,
        },
        //.cali_mode = BMI088_CALIBRATE_ONLINE_MODE,
        .cali_mode = BMI088_LOAD_PRE_CALI_MODE,                 //使用预校准参数，避免每次开机都要校准
        .work_mode = BMI088_BLOCK_PERIODIC_MODE,            //工作在阻塞周期模式，后续在 INS_Task 里会设置周期和阻塞时间    

    };
    gimbal_IMU_data = INS_Init(BMI088Register(&config)); //初始化云台 IMU/INS 实例
/*-----------------------------------------------------------------------------Pitch电机配置----------------------------------------------------------------------------------*/
// PITCH
        Motor_Init_Config_s pitch_motor_config = {//DM4310
        .can_init_config = {
            .can_handle = &hcan1,
            .tx_id = 0x02,
            .rx_id = 0x12,
        },
        .motor_type = DM_Motor,
        .controller_param_init_config ={
            .angle_PID = {
                .Kp = GIMBAL_PITCH_ANGLE_KP,
                .Ki = GIMBAL_PITCH_ANGLE_KI,
                .Kd = GIMBAL_PITCH_ANGLE_KD,
                .DeadBand = GIMBAL_PITCH_ANGLE_DEADBAND,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit |
                           PID_Derivative_On_Measurement | PID_DerivativeFilter |
                           PID_OutputFilter,
                .IntegralLimit = GIMBAL_PITCH_ANGLE_INTEGRAL_LIMIT,
                .MaxOut = GIMBAL_PITCH_ANGLE_MAXOUT,
                .Output_LPF_RC = GIMBAL_PITCH_ANGLE_OUTPUT_FILTER,
                .Derivative_LPF_RC = GIMBAL_PITCH_ANGLE_DERIVATIVE_FILTER,
            },
            .speed_PID = {
                .Kp = GIMBAL_PITCH_SPEED_KP,
                .Ki = GIMBAL_PITCH_SPEED_KI,
                .Kd = GIMBAL_PITCH_SPEED_KD,
                .DeadBand = GIMBAL_PITCH_SPEED_DEADBAND,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit |
                           PID_OutputFilter,
                .IntegralLimit = GIMBAL_PITCH_SPEED_INTEGRAL_LIMIT,
                .MaxOut = GIMBAL_PITCH_SPEED_MAXOUT,
                .Output_LPF_RC = GIMBAL_PITCH_SPEED_OUTPUT_FILTER,
            },
            //  .other_angle_feedback_ptr = &gimbal_IMU_data->output.INS_angle[INS_PITCH_ADDRESS_OFFSET], // pitch
            .other_angle_feedback_ptr = &gimbal_IMU_data->output.INS_angle[INS_PITCH_ADDRESS_OFFSET],     //pitch角度反馈:IMU 里的 pitch 弧度
            // ??????????????,????,ins_task.md??c??bodyframe?????
            .other_speed_feedback_ptr = &gimbal_IMU_data->INS_data.INS_gyro[INS_PITCH_ADDRESS_OFFSET],
            .speed_feedforward_ptr = &pitch_speed_feedforward,
            .current_feedforward_ptr = &pitch_tor_feedforward,
        },
        .controller_setting_init_config = {
            .angle_feedback_source = OTHER_FEED,
            .speed_feedback_source = OTHER_FEED,
            .outer_loop_type       = ANGLE_LOOP,
            .close_loop_type       = SPEED_LOOP | ANGLE_LOOP,
            .motor_reverse_flag    = MOTOR_DIRECTION_NORMAL,
            .feedback_reverse_flag = FEEDBACK_DIRECTION_NORMAL,
            .feedforward_flag      = SPEED_FEEDFORWARD | CURRENT_FEEDFORWARD,
            .control_range = {
                .P_max = 12.5,
                .V_max = 30,
                .T_max = 10,
            },
        },
    };
    pitch_motor = DMMotorInit(&pitch_motor_config);         //创建 DM 电机实例后先停机，避免上电立刻输出
    DMMotorStop(pitch_motor);
    #endif
    gimbal_pub = PubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));//注册一个名为 "gimbal_feed" 的话题，用于发布云台数据，数据长度为 Gimbal_Upload_Data_s 结构体的大小
    gimbal_sub = SubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));//注册一个名为 "gimbal_cmd" 的话题，用于订阅云台控制命令，数据长度为 Gimbal_Ctrl_Cmd_s 结构体的大小
}
/*------------------------------------------------------------------------辅助函数--------------------------------------------------------------------------------------------*/
void pitch_limit(float angle)
{
    if(angle>PITCH_UP_LIMIT)
        angle=PITCH_UP_LIMIT;
    else if(angle<PITCH_DOWN_LIMIT)
        angle=PITCH_DOWN_LIMIT;
    pitch_motor->motor_controller.pid_ref = angle;
}
/*-----------------------------------------------------------------------目前未使用的前馈拟合曲线-------------------------------------------------------------------------*/
/*
 * A nonlinear function that maps a value
 * from the range [X_MIN, -X_MAX] to the range [Y_MIN, Y_MAX].
 */
double nonlinear(const double x) {
#define X_MIN (-0.5)
#define X_MAX 0.5
#define Y_MIN (-2700)
#define Y_MAX 3000

    const double scaled = (x - X_MIN) / (X_MAX - X_MIN) * 4.0 - 2.0;
    const double out = (tanh(scaled) + 1.0) / 2.0;
    return out * (Y_MAX - Y_MIN) + Y_MIN;
}
//把 [-0.5,0.5] 非线性映射到 [-2700,3000]，本质是 tanh 形状的前馈函数，可以用来把视觉给出的 yaw 速度映射成一个前馈值，配合 PID 控制器更好地跟踪目标

float PitchNonlinear(float x) {
    //return (215.65 * x -3050.0);
    return (186.0 * x - 1292.0);
}
//线性映射函数，把视觉给出的 pitch 速度映射成一个前馈值，参数需要根据实际情况调整

#define NUM_SEGMENTS 11

const double coeffs[NUM_SEGMENTS][4] = {
    {0.8782, 47.2692, -548.1474, 7500.0000},
    {0.8782, 49.9038, -450.9744, 7000.0000},
    {-2.4321, 53.3287, -316.7722, 6500.0000},
    {-2.3856, 8.8205, 62.3377, 6000.0000},
    {34.5316, -56.3055, -369.7766, 5500.0000},
    {-6.0712, 78.3677, -341.0958, 5000.0000},
    {-0.4334, 1.8704, -4.0958, 4500.0000},
    {1.7548, -13.4722, -140.9974, 4000.0000},
    {-3.9207, 20.5351, -95.3710, 3000.0000},
    {5.0001, -41.0982, -203.1216, 2500.0000},
    {5.0001, -12.5977, -305.1439, 2000.0000}
};

const double x_start[NUM_SEGMENTS] = {
    -20.0000, -19.0000, -17.7000, -11.6000, -2.5000, -1.2000, 3.0000, 14.8000, 21.2600, 26.5000, 28.4000,
};
//把 [-20,28.4] 范围内的输入 x 非线性映射成一个输出 y，使用 11 段三次样条插值，每段的系数存储在 coeffs 数组里，x_start 数组存储了每段的起始 x 值
double evaluate_spline(const double x) {
    if (x < x_start[0]) {
        return coeffs[0][3];
    } else if (x > x_start[NUM_SEGMENTS - 1]) {
        return coeffs[NUM_SEGMENTS - 1][3];
    }
    int i;
    // 找到 x 落在哪一段区间内
    for (i = 0; i < NUM_SEGMENTS; i++) {
        if (x >= x_start[i] && (i == NUM_SEGMENTS - 1 || x < x_start[i + 1])) {
            break;
        }
    }
    // 计算 x 在该段区间内的相对位置，并使用对应的系数计算输出 y
    const double dx = x - x_start[i];
    const double y = coeffs[i][0] * dx * dx * dx + coeffs[i][1] * dx * dx + coeffs[i][2] * dx + coeffs[i][3];

    return y;
}
//拟合前馈曲线的函数，输入 x 是一个角度值，输出 y 是对应的前馈电流值，使用了 11 段三次样条插值来拟合一个非线性关系，可以用来把 pitch 轴的角度误差映射成一个前馈电流值，配合 PID 控制器更好地跟踪目标

// static float YawFeedForwardCalculate(float direction)
// {
//     if (direction > 1e-6)
//     {
//         yaw_current_feedforward = 4000;
//     }
//     else if (direction < -1e-6)
//     {
//         yaw_current_feedforward = -4000;
//     }
//     else
//     {
//         yaw_current_feedforward = 0;
//     }
// }

/*----------------------------------------------------------------------- yaw 电流前馈和视觉 PID 中间层------------------------------------------------------------------------*/
// static PIDInstance YAW_version_PID = {
//     .Kp            = 1,   // 25,//25, // 50,//70, // 4.5
//     .Ki            = 0,    // 0
//     .Kd            = 0, // 0.0,  // 0.07,  // 0
//     .DeadBand      = 0,  // 0.75,  //
//     .IntegralLimit = 3000,
//     // .Improve       = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
//     .MaxOut        = 30,

// };
// static PIDInstance PITCH_version_PID = {
//     .Kp            = 1,   // 25,//25, // 50,//70, // 4.5
//     .Ki            = 0,    // 0
//     .Kd            = 0, // 0.0,  // 0.07,  // 0
//     .DeadBand      = 0,  // 0.75,  //
//     .IntegralLimit = 30,
//     .Improve       = PID_OutputFilter|PID_Derivative_On_Measurement|PID_Integral_Limit|PID_Trapezoid_Intergral/*PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement*/,
//     .MaxOut        = 30,
//     .Output_LPF_RC=0.7,
// };
/*---------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
float nuc_version_control[2];
// float filiter(float input){
//     static float alpha=1,input_last;
//     float output=alpha*input+(1-alpha)*input_last;
//     input_last=output;
//     return output;
// }
float yaw_feedforward_angleErrmax=10,yaw_feedforward_max=3200; //

// float fpv_pitch_test=0 , telescope_test=0;
uint16_t pitch_test=0;
uint8_t fpv_state=0;//0:rise 1:down 2:ready
uint16_t fpv_count=0;
float fpv_pitch_up=0,fpv_pitch_down=0,telescope_up=0,telescope_down=0;
extern uint8_t telescope_pos ;//0:normal 1:zoom
extern uint8_t fpv_pos ;//0:normal 1:lob
extern float pitch_vel;
void GimbalTask()
{
    
    SubGetMessage(gimbal_sub, &gimbal_cmd_recv);//从消息中心取最新 gimbal_cmd，写到 gimbal_cmd_recv
/*---------------------------------------------旧的 yaw 电流前馈和视觉 PID 中间层，全部停用-----------------------------------------------------------------------------*/
    // yaw_current_feedforward=yaw_feedforward_max*(yaw_motor->motor_controller.angle_PID.Err/yaw_feedforward_angleErrmax);//angleErr??????4000
    // if(yaw_current_feedforward>yaw_feedforward_max)yaw_current_feedforward=yaw_feedforward_max;
    // else if(yaw_current_feedforward<-yaw_feedforward_max)yaw_current_feedforward=-yaw_feedforward_max;
    // gimbal_cmd_recv.pitch_version=filiter(gimbal_cmd_recv.pitch_version);
    // gimbal_cmd_recv.yaw_version=filiter(gimbal_cmd_recv.yaw_version);
    // if(gimbal_cmd_recv.nuc_mode){
    //     nuc_version_control[0]=PIDCalculate(&PITCH_version_PID,gimbal_cmd_recv.pitch_version,0);
    //     nuc_version_control[1]=PIDCalculate(&YAW_version_PID,gimbal_cmd_recv.yaw_version,0);
    // }
    // else memset(nuc_version_control,0,sizeof(nuc_version_control));
/*-------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
    // 
    #if defined(ONE_BOARD) || defined(CHASSIS_BOARD)        
    // yaw_gyro_twoboard=chassis_rs485_recv.yaw_gyro-gimbal_IMU_data->INS_data.INS_gyro[INS_YAW_ADDRESS_OFFSET];
    //yaw_current_feedforward=nonlinear(yaw_motor->motor_controller.angle_PID.Err);
    
    // if(!gimbal_cmd_recv.yaw_speedFeed)
    // yaw_speedFeed=1.25;
    // else yaw_speedFeed=0;
    // if(gimbal_cmd_recv.yaw_kp)yaw_motor->motor_controller.angle_PID.Kp=gimbal_cmd_recv.yaw_kp;
    // else yaw_motor->motor_controller.angle_PID.Kp=1.7;
    // if(gimbal_cmd_recv.yaw_kd)yaw_motor->motor_controller.angle_PID.Kd=gimbal_cmd_recv.yaw_kd;
    // else yaw_motor->motor_controller.angle_PID.Kd=0.13;
    // if(gimbal_cmd_recv.yaw_speedKp)yaw_motor->motor_controller.speed_PID.Kp=gimbal_cmd_recv.yaw_speedKp;
    // else yaw_motor->motor_controller.speed_PID.Kp=3000;
/*-------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
    switch (gimbal_cmd_recv.gimbal_mode) {
        // 停机模式，yaw 和 pitch 轴都停机，不使用任何前馈，电机不输出力矩
        case GIMBAL_ZERO_FORCE:
            DJIMotorStop(yaw_motor);          
            yaw_speed_feedforward = 0.0f;
            yaw_current_feedforward = 0.0f;
            filtered_yaw_vel = 0.0f;
            GimbalSMCReset(&yaw_smc_state);
            // DJIMotorStop(pitch_motor);
            break;
        // 视觉模式，yaw 轴使用视觉前馈，pitch 轴不使用前馈，完全由 PID 控制器根据 IMU 反馈来控制
        case GIMBAL_GYRO_MODE: { // 这个模式下 yaw 轴使用视觉前馈，pitch 轴不使用前馈，完全由 PID 控制器根据 IMU 反馈来控制
            DJIMotorEnable(yaw_motor);
            DJIMotorChangeFeed(yaw_motor,ANGLE_LOOP, OTHER_FEED);
            DJIMotorChangeFeed(yaw_motor,SPEED_LOOP, OTHER_FEED);
            DJIMotorOuterLoop(yaw_motor, ANGLE_LOOP);
            float yaw_smc_ref = gimbal_cmd_recv.yaw;
            if (gimbal_cmd_recv.nuc_mode == version_control)
            {
                float raw_yaw_vel = 0;
                float yaw_error = gimbal_cmd_recv.yaw_version - *yaw_motor->motor_controller.other_angle_feedback_ptr;
                float yaw_vel_alpha = yaw_vel_filter_alpha;
                // 根据编译选项选择 yaw 速度的来源，如果是 ONE_BOARD 或 GIMBAL_BOARD 就直接用视觉给出的 yaw 速度，如果是 CHASSIS_BOARD 就用串口收到的 yaw 速度
                #if defined(ONE_BOARD) || defined(GIMBAL_BOARD)
                raw_yaw_vel = vision_yaw_vel;//视觉给出的 yaw 速度，单位是度每秒
                #elif defined(CHASSIS_BOARD)//chassis_rs485_recv.yaw_gyro 就是串口收到的 yaw 速度，单位是度每秒
                    raw_yaw_vel = chassis_rs485_recv.yaw_vel;
                #endif

                // 为了避免 yaw 轴在 0/360 度附近来回切换导致的误差过大，进行一个特殊处理，如果误差超过 180 度，就加减 360 度让它变成一个较小的误差
                if (yaw_error > 180.0f)
                {
                    gimbal_cmd_recv.yaw_version = *yaw_motor->motor_controller.other_angle_feedback_ptr - 360.0 + yaw_error;
                    yaw_error -= 360.0f;
                }
                else if (yaw_error < -180.0)
                {
                    gimbal_cmd_recv.yaw_version = *yaw_motor->motor_controller.other_angle_feedback_ptr + (360 + yaw_error);
                    yaw_error += 360.0f;
                }

                // 对原始 yaw 速度进行一阶低通滤波，得到滤波后的 yaw 速度，单位还是度每秒
                if (raw_yaw_vel * filtered_yaw_vel < 0.0f) {
                    yaw_vel_alpha = yaw_vel_reverse_alpha;
                }
                filtered_yaw_vel += yaw_vel_alpha * (raw_yaw_vel - filtered_yaw_vel);

                // 如果滤波后的 yaw 速度绝对值小于死区阈值，就把它置零，避免微小的噪声引起不必要的前馈
                if (fabsf(filtered_yaw_vel) < yaw_vel_deadzone) {
                    filtered_yaw_vel = 0.0f;
                }

                yaw_speed_feedforward = filtered_yaw_vel * yaw_feedforward_vel_gain;
                yaw_speed_feedforward *= YawVisionFeedforwardScale(yaw_error);
                yaw_speed_feedforward = clampf_local(yaw_speed_feedforward,
                                                     -yaw_speed_feedforward_limit,
                                                     yaw_speed_feedforward_limit);
                
                yaw_smc_ref = gimbal_cmd_recv.yaw_version;
                DJIMotorSetRef(yaw_motor, yaw_smc_ref);
            }
            else
            {
                yaw_speed_feedforward = 0;//如果不是自瞄模式，就不使用视觉前馈，yaw_speed_feedforward 置零
                filtered_yaw_vel = 0.0f;
                //yaw_current_feedforward = 0;
                yaw_smc_ref = gimbal_cmd_recv.yaw;
                DJIMotorSetRef(yaw_motor, yaw_smc_ref);// yaw 角度参考直接来自命令，单位是度，云台会尽力把 yaw 轴转到这个角度
            }
#if GIMBAL_YAW_SMC_ENABLE
            yaw_current_feedforward = GimbalSMCCalculate(&yaw_smc_state,
                                                         &yaw_smc_config,
                                                         yaw_smc_ref,
                                                         *yaw_motor->motor_controller.other_angle_feedback_ptr,
                                                         (*yaw_motor->motor_controller.other_speed_feedback_ptr) * GIMBAL_YAW_SMC_GYRO_TO_DEG);
#else
            yaw_current_feedforward = 0.0f;
#endif
            break;
        }
        case GIMBAL_MOTOR_MODE:
            DJIMotorEnable(yaw_motor);//电机使能
            DJIMotorChangeFeed(yaw_motor,ANGLE_LOOP,MOTOR_FEED);//把 yaw 轴的外环和内环的反馈源都切换成电机编码器，这样就不使用视觉前馈了，完全由电机自己根据编码器反馈来控制
            DJIMotorOuterLoop(yaw_motor, ANGLE_LOOP);//开启 yaw 轴的角度环控制
            DJIMotorSetRef(yaw_motor, gimbal_cmd_recv.yaw); // yaw 角度参考直接来自命令，单位是度，云台会尽力把 yaw 轴转到这个角度
            yaw_speed_feedforward = 0.0f;
            yaw_current_feedforward = 0.0f;
            filtered_yaw_vel = 0.0f;
            GimbalSMCReset(&yaw_smc_state);
        break;
        default:
            break;
    }
/*-------------------------把 yaw 角度环输出、IMU 指针、单圈角、ecd、总角度、实际电流塞进 gimbal_feedback_data，供别的模块读取------------------------------------------------------*/
    gimbal_feedback_data.yaw_angle_pidout             =yaw_motor->motor_controller.angle_PID.Output;
    gimbal_feedback_data.gimbal_imu_data              = gimbal_IMU_data;
    gimbal_feedback_data.yaw_motor_single_round_angle = (uint16_t)yaw_motor->measure.angle_single_round; // yaw 电机的单圈角度，单位是编码器单位，0-8191 对应 0-360 度
    gimbal_feedback_data.yaw_ecd                      = yaw_motor->measure.ecd;
    gimbal_feedback_data.pitch_ecd                    = 0;//pitch_motor->measure.ecd;
    gimbal_feedback_data.yaw_total_angle              = yaw_motor->measure.total_angle;
    gimbal_feedback_data.pitch_total_angle            = 0;//pitch_motor->measure.total_angle;
    gimbal_feedback_data.yaw_motor_real_current       = (float)yaw_motor->measure.real_current;
/*-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
    //yaw_current_feedforward = YawFeedForwardCalculate(yaw_motor->motor_controller.angle_PID.Err);
    #endif
    #if defined(ONE_BOARD) || defined(GIMBAL_BOARD)
    #ifndef BIG_HEAD
    // pitch_tor_feedforward = 0.42146 * cos(1.43 + gimbal_IMU_data->output.INS_angle[0]);
    // pitch_tor_feedforward = 
    // pitch_current_feedforward = PitchNonlinear(*pitch_motor->motor_controller.other_angle_feedback_ptr);

    const float pitch_angle_measure = gimbal_IMU_data->output.INS_angle[INS_PITCH_ADDRESS_OFFSET];
    pitch_gyro_measure =  gimbal_IMU_data->INS_data.INS_gyro[INS_PITCH_ADDRESS_OFFSET];
    const float pitch_gravity_feedforward = PitchGravityTorqueFeedforward(pitch_angle_measure,
                                                                          pitch_gyro_measure,
                                                                          pitch_motor->measure.pos);
    pitch_tor_feedforward = pitch_gravity_feedforward;
    pitch_current_feedforward = pitch_gravity_feedforward;
    switch (gimbal_cmd_recv.gimbal_mode) {
        //停掉 DM pitch 电机，并把 angle/speed 两个 PID 的积分项清零，同时关掉速度前馈，防止重新使能时积分残留
        case GIMBAL_ZERO_FORCE:
            DMMotorStop(pitch_motor);
            pitch_motor->motor_controller.speed_PID.Iout = 0;
            pitch_motor->motor_controller.angle_PID.Iout = 0;
            pitch_speed_feedforward = 0;
            pitch_tor_feedforward = 0;
            pitch_current_feedforward = 0;
            GimbalSMCReset(&pitch_smc_state);
            break;
        case GIMBAL_GYRO_MODE://使能 DM pitch 电机，切换到角度环和速度环都使用 IMU 反馈的模式，开启角度环控制
            DMMotorEnable1(pitch_motor);
            // DJIMotorChangeFeed(pitch_motor,SPEED_LOOP, OTHER_FEED);
            // DJIMotorChangeFeed(pitch_motor,ANGLE_LOOP, OTHER_FEED);
            // DJIMotorOuterLoop(pitch_motor, ANGLE_LOOP);
            
            if (gimbal_cmd_recv.nuc_mode == version_control)//自瞄模式，pitch 角度参考来自命令，并且使用一个单独的前馈值 pitch_version 来进行前馈补偿，这个前馈值可以是一个拟合函数的输出，也可以是一个滤波后的视觉速度乘以一个增益
            {
                // DJIMotorSetRef(pitch_motor, gimbal_cmd_recv.pitch_version);
                pitch_limit(gimbal_cmd_recv.pitch_version);//对 pitch 角度参考进行限幅，确保它在安全范围内
                pitch_speed_feedforward = pitch_vel;//把视觉给出的 pitch 速度直接作为速度前馈，这样可以让 pitch 轴更好地跟踪目标的运动，参数需要根据实际情况调整
            }
            else
            {
                // DJIMotorSetRef(pitch_motor,pitch_offset + gimbal_cmd_recv.pitch);
                pitch_limit(gimbal_cmd_recv.pitch);//对 pitch 角度参考进行限幅，确保它在安全范围内
                pitch_speed_feedforward = 0;//如果不是自瞄模式，就不使用视觉前馈，pitch_speed_feedforward 置零
            }
#if GIMBAL_PITCH_SMC_ENABLE
            {
                float pitch_smc_feedforward = 0.0f;
                float pitch_smc_err = pitch_motor->motor_controller.pid_ref - pitch_angle_measure;
                if (fabsf(pitch_smc_err) < GIMBAL_PITCH_SMC_LINKAGE_ERR_GATE &&
                    fabsf(pitch_gyro_measure) < GIMBAL_PITCH_SMC_LINKAGE_GYRO_GATE) {
                    GimbalSMCReset(&pitch_smc_state);
                } else {
                    pitch_smc_feedforward = GimbalSMCCalculate(&pitch_smc_state,
                                                               &pitch_smc_config,
                                                               pitch_motor->motor_controller.pid_ref,
                                                               pitch_angle_measure,
                                                               pitch_gyro_measure);
                    pitch_smc_feedforward *= PitchLinkageSmcScale(pitch_motor->measure.pos);
                }
                pitch_tor_feedforward = clampf_local(pitch_gravity_feedforward + pitch_smc_feedforward,
                                                     GIMBAL_PITCH_FF_TOTAL_MIN,
                                                     GIMBAL_PITCH_FF_TOTAL_MAX);
                pitch_current_feedforward = pitch_tor_feedforward;
            }
#else
            pitch_tor_feedforward = pitch_gravity_feedforward;
            pitch_current_feedforward = pitch_gravity_feedforward;
#endif
            // DJIMotorSetRef(fpv_pitch_motor,fpv_pitch_test);
            // DJIMotorSetRef(telescope_motor,telescope_test);
           
        
            break;
        case GIMBAL_MOTOR_MODE://使能 DM pitch 电机，切换到角度环和速度环都使用电机编码器反馈的模式，开启角度环控制，pitch 角度参考直接来自命令
            DJIMotorEnable(pitch_motor);
            // DJIMotorOuterLoop(pitch_motor, ANGLE_LOOP);
            // DJIMotorChangeFeed(pitch_motor,ANGLE_LOOP,MOTOR_FEED);
            // DJIMotorChangeFeed(pitch_motor,SPEED_LOOP,MOTOR_FEED);
       
            DJIMotorSetRef(pitch_motor, pitch_offset + gimbal_cmd_recv.pitch); //对 pitch 角度参考加上一个固定的偏置，确保它在一个合理的范围内，避免过度转动导致损坏，同时也可以根据实际情况调整这个偏置量
            // DJIMotorSetRef(pitch_motor, pitch_test); 
            pitch_speed_feedforward = 0;
            pitch_tor_feedforward = pitch_gravity_feedforward;
            pitch_current_feedforward = pitch_gravity_feedforward;
            GimbalSMCReset(&pitch_smc_state);

        break;
        default:
            break;
    } 
    // pitch_motor->motor_controller.other_speed_feedback_ptr = &pitch_speed_feedforward;
    #endif // !BIG_HEAD
    #ifdef BIG_HEAD
    pitch_current_feedforward=-exp((352-gimbal_IMU_data->output.INS_angle_deg[INS_PITCH_ADDRESS_OFFSET])/40);
    // pitch_motor->motor_controller.speed_PID.MaxOut=16384-fabs(pitch_current_feedforward);
    if(pitch_current_feedforward>-3500)pitch_current_feedforward=-3500;
    else if(pitch_current_feedforward<-6500)pitch_current_feedforward=-6500;
        switch (gimbal_cmd_recv.gimbal_mode) {
        
        //停掉 pitch 电机，并把 angle/speed 两个 PID 的积分项清零，同时关掉速度前馈，防止重新使能时积分残留
        case GIMBAL_ZERO_FORCE:
            DJIMotorStop(pitch_motor);
            break;
        // 视觉模式，pitch 轴使用 IMU 反馈，完全由 PID 控制器根据 IMU 反馈来控制
        case GIMBAL_GYRO_MODE: //这个模式下 pitch 轴使用 IMU 反馈，完全由 PID 控制器根据 IMU 反馈来控制
            DJIMotorEnable(pitch_motor);
            DJIMotorChangeFeed(pitch_motor,ANGLE_LOOP, OTHER_FEED);
            DJIMotorOuterLoop(pitch_motor, ANGLE_LOOP);
            DJIMotorSetRef(pitch_motor,gimbal_cmd_recv.pitch);
            break;
        case GIMBAL_MOTOR_MODE://这个模式下 pitch 轴使用电机编码器反馈，完全由电机自己根据编码器反馈来控制
            DJIMotorEnable(pitch_motor);
            DJIMotorChangeFeed(pitch_motor,ANGLE_LOOP, OTHER_FEED);
            DJIMotorOuterLoop(pitch_motor, ANGLE_LOOP);
            DJIMotorSetRef(pitch_motor, gimbal_cmd_recv.pitch);
        break;
        default:
            break;
    }
    #endif
    gimbal_feedback_data.gimbal_imu_data              = gimbal_IMU_data;//把 IMU 数据塞进 gimbal_feedback_data，供别的模块读取
    // gimbal_feedback_data.yaw_motor_single_round_angle = (uint16_t)yaw_motor->measure.angle_single_round; 
    // gimbal_feedback_data.yaw_ecd                      = yaw_motor->measure.ecd;
    // gimbal_feedback_data.pitch_ecd                    = pitch_motor->measure.ecd;
    // gimbal_feedback_data.yaw_total_angle              = yaw_motor->measure.total_angle;
    // gimbal_feedback_data.pitch_total_angle            = pitch_motor->measure.total_angle;
    
    #endif

    // gimbal_feedback_data.gimbal_imu_data              = gimbal_IMU_data;
    // gimbal_feedback_data.yaw_motor_single_round_angle = (uint16_t)yaw_motor->measure.angle_single_round; 
    // gimbal_feedback_data.yaw_ecd                      = yaw_motor->measure.ecd;
    // gimbal_feedback_data.pitch_ecd                    = pitch_motor->measure.ecd;
    // gimbal_feedback_data.yaw_total_angle              = yaw_motor->measure.total_angle;
    // gimbal_feedback_data.pitch_total_angle            = pitch_motor->measure.total_angle;
    PubPushMessage(gimbal_pub, (void *)&gimbal_feedback_data);//把最新的 gimbal_feedback_data 发布到消息中心，供别的模块订阅读取
}





