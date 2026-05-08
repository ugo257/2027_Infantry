/**
 * @file chassis.c
 * @author NeoZeng neozng1@hnu.edu.cn
 * @brief 底盘应用,负责接收robot_cmd的控制命令并根据命令进行运动学解算,得到输出
 *        注意底盘采取右手系,对于平面视图,底盘纵向运动的正前方为x正方向;横向运动的右侧为y正方向
 *
 * @version 0.1
 * @date 2022-12-04
 *
 * @copyright Copyright (c) 2022
 *
 */
/*------------------------------------------------------------------------------*/
#include "chassis.h"//拿到本模块对外接口 ChassisInit()、ChassisTask()
#include "robot_board.h"//根据 CHASSIS_BOARD / ONE_BOARD 决定编译哪部分代码
#include "robot_params.h"//读底盘几何参数、限位、控制宏定义等
#include "robot_types.h"//读 Chassis_Ctrl_Cmd_s、chassis_mode_e、leg_mode_e 等类型
#include "remote_control.h"// 读遥控器数据
/*------------------------------------------------------------------------------*/
#include "dji_motor.h"//分别给四个轮毂电机和两个关节电机提供驱动接口
#include "DMmotor.h"
/*------------------------------------------------------------------------------*/
#include "super_cap.h"//超级电容和功率控制
#include "power_calc.h"
/*------------------------------------------------------------------------------*/
#include "message_center.h"//底盘和上层命令模块通过发布/订阅通信
/*------------------------------------------------------------------------------*/
#include "arm_math.h"//用 CMSIS 数学库算 sin/cos 等函数
#include "ins_task.h"//使用 IMU/INS 解算结果
#include "tool.h"//目前仅有键盘斜坡函数
/*------------------------------------------------------------------------------*/
#include "referee_init.h"//裁判系统初始化
#include "referee_UI.h"//裁判系统数据解析，提供给UI显示用
#include "rm_referee.h"//裁判系统数据解析，提供给底盘控制用
/*------------------------------------------------------------------------------*/
#include "general_def.h"//一些module的通用数值型定义
/*------------------------------------------------------------------------------*/
#include "bsp_dwt.h"//高精度时间函数，用于计算底盘运动解算的时间间隔，进而进行积分等运算
#include "bsp_can.h"// CAN通信接口，底盘通过CAN总线与电机、超级电容等模块通信
/*---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
/* 根据robot_def.h中的macro自动计算的参数 */
#define HALF_WHEEL_BASE  (WHEEL_BASE / 2.0f)     // 半轴距
#define HALF_TRACK_WIDTH (TRACK_WIDTH / 2.0f)    // 半轮距
#define PERIMETER_WHEEL  (RADIUS_WHEEL * 2 * PI) // 轮子周长(暂时未使用)
/*给四个轮子定义索引，分别是左前、右前、右后、左后。*/
#define LF               0
#define RF               1
#define RB               2
#define LB               3
/*这是一套“速度误差 -> 目标加速度 -> 车体力/力矩 -> 电机电流前馈”的参数*/
/*
 * 麦轮力控参数：
 * 1) 速度误差 -> 目标加速度（P 环）
 * 2) 目标加速度 -> 轮作用力分配
 * 3) 轮作用力 -> 电流前馈
 */
#define CHASSIS_FORCE_EQ_MASS           1.0f        //等效质量，单位kg，表示底盘在进行力控时的惯性大小，数值越大底盘越不容易被加速
#define CHASSIS_FORCE_EQ_INERTIA        0.8f        //等效转动惯量，单位kg*m^2，表示底盘在进行力控时的转动惯量大小，数值越大底盘越不容易被加速
#define CHASSIS_FORCE_VEL_P             2.0f        //速度误差->加速度的P环增益，单位s^-1，数值越大底盘对速度误差的响应越快，但过大会导致震荡
#define CHASSIS_FORCE_WZ_P              1.4f        //角速度误差->角加速度的P环增益，单位s^-1，数值越大底盘对角速度误差的响应越快，但过大会导致震荡
#define CHASSIS_FORCE_ACC_LIMIT         2500.0f     //加速度限幅，单位mm/s^2，表示底盘在进行力控时的最大加速度，数值越小底盘越平稳但响应越慢
#define CHASSIS_FORCE_WZ_ACC_LIMIT      4500.0f    //角加速度限幅，单位rad/s^2，表示底盘在进行力控时的最大角加速度，数值越小底盘越平稳但响应越慢
#define CHASSIS_FORCE_TO_CURRENT        1.3f       //加速度->电流的转换系数，单位A/(m/s^2)，数值越大底盘的力控输出越大，但过大会导致震荡
#define CHASSIS_FORCE_CURRENT_FF_LIMIT  3500.0f    //电流前馈限幅，单位A，表示底盘在进行力控时的最大电流前馈，数值越小底盘越平稳但响应越慢
#define CHASSIS_FORCE_CURRENT_FF_SLEW_STEP 450.0f  //力控电流前馈每周期最大变化量，限制突变
#define CHASSIS_FORCE_OBS_FILTER_ALPHA  0.25f      //轮速反推底盘速度的一阶低通系数
#define CHASSIS_FORCE_OBS_DEADBAND      1.0f       //观测速度小死区，抑制静止附近抖动
/*---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
#define LF_CENTER ((HALF_TRACK_WIDTH + center_gimbal_offset_x + HALF_WHEEL_BASE - center_gimbal_offset_y) * DEGREE_2_RAD)//左前轮距云台中心的夹角，单位弧度
#define RF_CENTER ((HALF_TRACK_WIDTH - center_gimbal_offset_x + HALF_WHEEL_BASE - center_gimbal_offset_y) * DEGREE_2_RAD)//右前轮距云台中心的夹角，单位弧度
#define LB_CENTER ((HALF_TRACK_WIDTH + center_gimbal_offset_x + HALF_WHEEL_BASE + center_gimbal_offset_y) * DEGREE_2_RAD)//左后轮距云台中心的夹角，单位弧度
#define RB_CENTER ((HALF_TRACK_WIDTH - center_gimbal_offset_x + HALF_WHEEL_BASE + center_gimbal_offset_y) * DEGREE_2_RAD)//右后轮距云台中心的夹角，单位弧度
/*---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
// 低速起步摩擦补偿的阈值、滞回宽度和平滑系数
#define FRICTION_KICK_THRESHOLD 100.0f    // 低速启动阈值（度/秒）
#define FRICTION_HYSTERESIS 30.0f          // 滞回带宽度（度/秒）
#define FRICTION_SMOOTHING 0.2f            // 补偿平滑系数（0~1）
/*---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
/* 底盘应用包含的模块和信息存储,底盘是单例模式,因此不需要为底盘建立单独的结构体 */
#ifdef CHASSIS_BOARD // 如果是底盘板,使用板载IMU获取底盘转动角速度
// #include "can_comm.h"
// static CANCommInstance *chasiss_can_comm; // 双板通信CAN comm
//attitude_t *Chassis_IMU_data;
#endif // CHASSIS_BOARD
/*---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
#if defined(CHASSIS_BOARD) || defined(ONE_BOARD)
static Publisher_t *chassis_pub;                    // 用于发布底盘的数据
static Subscriber_t *chassis_sub;                   // 用于订阅底盘的控制命令
#endif                                              // !ONE_BOARD
static Chassis_Ctrl_Cmd_s chassis_cmd_recv;         // 底盘接收到的控制命令
static Chassis_Ctrl_Cmd_s_half_float chassis_cmd_recv_half_float;//半精度版命令缓存，目前基本没在运行逻辑里用。
static Chassis_Upload_Data_s chassis_feedback_data; // 底盘要回传给别的模块的数据
static SuperCapInstance *supercap;                  // 超级电容实例指针
static DJIMotorInstance *motor_lf, *motor_rf, *motor_lb, *motor_rb; // 四个麦轮电机对象
static DJIMotorInstance *sync_belt_motor_l, *sync_belt_motor_r;      // 同步带左右电机对象
DMMotorInstance *joint_l, *joint_r;                 //左右关节电机对象
extern referee_info_t *referee_data_for_ui;         //裁判系统数据，底盘会读电源状态
uint16_t power_supdata_watch  ;                     //电源数据监视变量，底盘每次接收到裁判系统发来的数据就更新这个变量，底盘定时器每次到达就检查这个变量，如果超过一定时间没更新就认为裁判系统掉线了
extern float motorset[4];
/*---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
// 为了方便调试加入的量
static uint8_t center_gimbal_offset_x = CENTER_GIMBAL_OFFSET_X; // 云台旋转中心距底盘几何中心的距离,前后方向,云台位于正中心时默认设为0
static uint8_t center_gimbal_offset_y = CENTER_GIMBAL_OFFSET_Y; // 云台旋转中心距底盘几何中心的距离,左右方向,云台位于正中心时默认设为0
/*---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
static INS_Instance *Chassis_IMU_data;              // 底盘 INS 数据指针
// extern INS_Instance *gimbal_IMU_data;             // 云台 IMU 数据指针，底盘控制云台时需要用到云台的IMU数据
extern RC_ctrl_t rc_ctrl[2];                         // 遥控器数据，底盘控制模式切换和一些特殊动作需要用到遥控器数据
/*---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
/* 私有函数计算的中介变量,设为静态避免参数传递的开销 */
static float chassis_vx, chassis_vy, chassis_vw; // 将云台系的速度投影到底盘
static float vt_lf, vt_rf, vt_lb, vt_rb;         // 底盘速度解算后的临时输出,待进行限幅
static float kl=1;                              //左右腿长度差补偿系数，理论上应该等于右腿长度/左腿长度，但实际使用中需要调试这个值才能达到最好的效果
extern INS_Instance *INS;                       // IMU数据指针，底盘控制时需要用到IMU数据
int8_t sign_lf, sign_rf, sign_lb, sign_rb;      //每个轮子的摩擦补偿状态
static float friction_lf_state = 0.0f;          //每个轮子的摩擦补偿状态，正数表示正向摩擦补偿，负数表示反向摩擦补偿，绝对值表示补偿的大小
static float friction_rf_state = 0.0f;
static float friction_lb_state = 0.0f;
static float friction_rb_state = 0.0f;
static float chassis_force_ff_lf = 0.0f;        //四个轮子的电流前馈值，会传给轮电机控制器
static float chassis_force_ff_rf = 0.0f;
static float chassis_force_ff_lb = 0.0f;
static float chassis_force_ff_rb = 0.0f;
static float chassis_force_obs_vx_state = 0.0f;
static float chassis_force_obs_vy_state = 0.0f;
static float chassis_force_obs_wz_state = 0.0f;
static uint8_t chassis_force_obs_inited = 0u;
static leg_mode_e leg_mode = LEG_ACTIVE_SUSPENSION;//腿部当前模式，默认 LEG_ACTIVE_SUSPENSION
GPIO_InitTypeDef GPIO_InitStruct = {0};            // GPIO初始化结构体，底盘控制时需要用到GPIO输出一些信号
volatile static float joint_l_tor_feedforward = 0, joint_r_tor_feedforward = 0;//两个关节的力矩前馈。，单位 Nm，正数表示增加正向力矩，负数表示增加反向力矩
float length_diff,length_diff_tor;                 //左右腿长度差和长度差对应的力矩
static float rotate_vxy_scale_state = 1.0f;        //自旋模式下，平移量缩放和跟踪质量状态。
static float rotate_wz_track_ratio_state = 1.0f;    
/*---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
/**
 * @brief 计算每个轮毂电机的输出,正运动学解算
 *        用宏进行预替换减小开销,运动解算具体过程参考教程
 */
float vxy_k=1,super_vxy_k=2;                    //平移速度缩放系数，vxy_k 是普通模式下的，super_vxy_k 目前没实际用
float friction_compensation_lf = 600.0f;        //四个轮子单独调的静摩擦补偿量
float friction_compensation_rf = -750.0f;  
float friction_compensation_lb = 110.0f;  
float friction_compensation_rb = -100.0f;  
// float friction_compensation_lf = 0.0f;  
// float friction_compensation_rf = 0.0f;  
// float friction_compensation_lb = 0.0f;  
// float friction_compensation_rb = 0.0f;  
/*---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
// 跟随模式底盘的pid
// 目前没有设置单位，有些不规范，之后有需要再改
static PIDInstance Chassis_Follow_PID = {
    .Kp            = 105,//105,   // 25,//25, // 50,//70, // 4.5
    .Ki            = 0,    // 0
    .Kd            = 1.5, // 0.0,  // 0.07,  // 0
    .DeadBand      = 2.0,  // 0.75,  //跟随模式设置了死区，防止抖动
    .IntegralLimit = 3000,
    .Improve       = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
    .MaxOut        = 16384,

};
//左右腿长度差补偿 PID，用来让左右腿长度尽量一致
static PIDInstance Leg_Diff_PID = {
    .Kp            = 150,   // 25,//25, // 50,//70, // 4.5
    .Ki            = 0,    // 0
    .Kd            = 0, // 0.0,  // 0.07,  // 0
    .DeadBand      = 0.01,  //跟随模式设置了死区，防止抖动
    .IntegralLimit = 1,
    .Improve       = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
    .MaxOut        = 10,
};
/*---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
// 收腿平滑参数，避免切到收腿模式时姿态和同步带指令突变
#define LEG_CLIMB_DIP_TARGET        0.07f
#define LEG_RETRACT_DIP_TARGET     -0.1f
#define LEG_DIP_SLEW_STEP           0.0010f
#define LEG_FOLLOW_KP_SLEW_STEP     1.0f
#define LEG_SYNC_BELT_SLEW_STEP    350.0f
#define LEG_SYNC_BELT_PRELOAD_REF  8000.0f
#define LEG_SYNC_BELT_MAX_REF      15000.0f
#define LEG_SYNC_BELT_FLY_SLOPE_REF 0.0f
#define LEG_SYNC_BELT_FLY_SLOPE_REF_SIGN -1.0f
#define SYNC_BELT_DIRECTION_DEADBAND 10.0f
#define LEG_MANUAL_EXTEND_DIP_TARGET 0.20f
#define LEG_MANUAL_EXTEND_LEG_P       0.08f
#define LEG_MANUAL_EXTEND_LENGTH_SLEW_STEP 0.00020f
#define LEG_ACTIVE_LENGTH_TARGET_MIN  0.125f
#define LEG_ACTIVE_LENGTH_TARGET_MAX  0.285f
#define LEG_ACTIVE_FOLLOW_LENGTH_MAX  0.220f
#define LEG_ACTIVE_LENGTH_PROTECT_START 0.220f
#define LEG_ACTIVE_LENGTH_PROTECT_KP    80.0f
#define LEG_ACTIVE_LENGTH_PROTECT_MAX_TORQUE 2.5f
#define LEG_ACTIVE_LENGTH_PROTECT_TORQUE_SIGN 1.0f
#define LEG_ACTIVE_LENGTH_RETRACT_SLEW_STEP 0.00025f
#define LEG_ACTIVE_LENGTH_EXTEND_SLEW_STEP  0.00080f
#define LEG_ACTIVE_POS_KP               0.0f
#define LEG_ACTIVE_POS_KD               0.0f
#define LEG_FOLLOW_PITCH_ERR_FILTER_ALPHA   0.05f
#define LEG_FOLLOW_PITCH_ERR_DEADBAND       0.015f
#define LEG_FLY_SLOPE_CONTACT_DIP_TARGET 0.050f
#define LEG_FLY_SLOPE_PRE_EXTEND_DIP_TARGET 0.070f
#define LEG_FLY_SLOPE_CONTACT_LEG_P      0.75f
#define LEG_FLY_SLOPE_PRE_EXTEND_LEG_P   0.18f
#define LEG_FLY_SLOPE_CONTACT_LENGTH_ADD 0.015f
#define LEG_FLY_SLOPE_PRE_EXTEND_LENGTH_ADD 0.045f
#define LEG_FLY_SLOPE_CONTACT_LENGTH_TARGET LEG_FLY_SLOPE_LENGTH_MIN
#define LEG_FLY_SLOPE_PRE_EXTEND_LENGTH_TARGET 0.260f
#define LEG_FLY_SLOPE_LENGTH_MIN         0.125f
#define LEG_FLY_SLOPE_LENGTH_MAX         0.285f
#define LEG_FLY_SLOPE_LENGTH_SLEW_STEP   0.00050f
#define LEG_FLY_SLOPE_POS_KP              18.0f
#define LEG_FLY_SLOPE_POS_KD               0.4f
#define LEG_FLY_SLOPE_CONTACT_JOINT_ASSIST 0.020f
#define LEG_FLY_SLOPE_PRE_EXTEND_JOINT_ASSIST 0.025f
#define LEG_FLY_SLOPE_BACKWARD_VX_THRESHOLD 1000.0f
#define LEG_FLY_SLOPE_BACKWARD_VX_SIGN       1.0f
#define LEG_FLY_SLOPE_BACKWARD_DIP_TARGET    0.0f
#define LEG_FLY_SLOPE_BACKWARD_LENGTH_SLEW_STEP 0.00100f
#define LEG_FLY_SLOPE_PRE_EXTEND_DELAY_COUNT 500u
#define LEG_FLY_SLOPE_SLOPE_SEEN_PITCH   0.20f
#define LEG_FLY_SLOPE_TAKEOFF_PITCH      0.12f
#define LEG_RETRACT_TARGET_LENGTH_MIN   0.125f
#define LEG_RETRACT_TARGET_LENGTH_STEP  0.0016f
#define LEG_RETRACT_TARGET_TORQUE_MAX   20.0f
#define LEG_RETRACT_TORQUE_FF_A       4.9440f
#define LEG_RETRACT_TORQUE_FF_B     -17.9963f
#define LEG_RETRACT_TORQUE_FF_C      20.2303f
#define LEG_RETRACT_TORQUE_MIN_ANGLE  1.0f
#define LEG_RETRACT_TORQUE_MAX_ANGLE  1.25f
#define LEG_RETRACT_TORQUE_MAX        12.0f
#define LEG_RETRACT_MANUAL_BOOST_TORQUE 25.0f
#define LEG_RETRACT_MANUAL_TORQUE_LIMIT 28.0f
#define LEG_RETRACT_ENTRY_BOOST_COUNT    200u
#define LEG_RETRACT_ENTRY_BOOST_TORQUE   22.0f
#define LEG_RETRACT_ENTRY_TORQUE_LIMIT   26.0f
#define LEG_RETRACT_POS_KP               80.0f
#define LEG_RETRACT_POS_KD                1.5f
#define LEG_MANUAL_PRELOAD_COUNT        100u
#define LEG_MANUAL_PRELOAD_DIP_TARGET   0.10f
#define LEG_MANUAL_PRELOAD_TORQUE       6.0f
#define LEG_RETRACT_LIMIT_TORQUE_THRESHOLD 18.0f
#define LEG_RETRACT_LIMIT_VEL_THRESHOLD     0.20f
#define LEG_RETRACT_LIMIT_DETECT_COUNT      25u
#define LEG_RETRACT_HOLD_TORQUE_MIN         4.0f
#define LEG_AUTO_RETRACT_VX_THRESHOLD       8000.0f
#define LEG_AUTO_RETRACT_PITCH_ERR_TRIGGER  0.060f
#define LEG_AUTO_RETRACT_PITCH_ERR_MAX      0.120f
#define LEG_AUTO_RETRACT_PITCH_GYRO_TRIGGER 0.90f
#define LEG_AUTO_RETRACT_PITCH_GYRO_MAX     1.80f
#define LEG_AUTO_RETRACT_TORQUE_TRIGGER     8.0f
#define LEG_AUTO_RETRACT_TORQUE_MAX_TRIGGER 12.0f
#define LEG_AUTO_RETRACT_DETECT_COUNT       12u
#define LEG_AUTO_RETRACT_DURATION_COUNT     160u
#define LEG_AUTO_RETRACT_COOLDOWN_COUNT     120u
#define LEG_AUTO_RETRACT_BOOST_TORQUE       16.0f
#define LEG_AUTO_RETRACT_TORQUE_LIMIT       24.0f
#define LEG_AUTO_RETRACT_SYNC_BELT_REF      10000.0f
#define LEG_EDGE_HIT_VX_THRESHOLD           6000.0f
#define LEG_EDGE_HIT_TORQUE_TRIGGER         12.0f
#define LEG_EDGE_HIT_JOINT_VEL_THRESHOLD    0.25f
#define LEG_EDGE_HIT_PITCH_ERR_TRIGGER      0.075f
#define LEG_EDGE_HIT_PITCH_GYRO_TRIGGER     0.95f
#define LEG_EDGE_HIT_DETECT_COUNT           8u
#define LEG_EDGE_HIT_PRELOAD_COUNT          60u
#define LEG_EDGE_HIT_RETRACT_COUNT          180u
#define LEG_EDGE_HIT_BOOST_TORQUE           22.0f
#define LEG_EDGE_HIT_TORQUE_LIMIT           26.0f
#define LEG_EDGE_HIT_SYNC_BELT_REF          14000.0f

void ChassisInit()
{
    ;
#if defined(CHASSIS_BOARD)
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
        .cali_mode = BMI088_LOAD_PRE_CALI_MODE,
        //.cali_mode = BMI088_LOAD_PRE_CALI_MODE,
        .work_mode = BMI088_BLOCK_PERIODIC_MODE,
};
    Chassis_IMU_data = INS_Init(BMI088Register(&config)); 
/*---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
/*在 CHASSIS_BOARD 下配置 BMI088：PC4/PC5 作为加速度计/陀螺仪中断，TIM10 CH1 做温控加热 PWM，PA4/PB0 为 SPI 片选，载入预校准，工作在阻塞周期模式，最后INS_Init(BMI088Register(&config)) 初始化底盘 IMU*/
/*---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
#endif
    #if defined(ONE_BOARD) || defined(CHASSIS_BOARD)
    // 四个轮子的参数一样,改tx_id和反转标志位即可
    Motor_Init_Config_s chassis_motor_config_1 = {
        .can_init_config.can_handle   = &hcan1,
        .controller_param_init_config = {
            .speed_PID = {
                .Kp            = 3 ,// 4.0
                .Ki            = 0,   // 0
                .Kd            = 0,   // 0
                .IntegralLimit = 3000,
                .Improve       = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .MaxOut        = 16000,
            },
            .current_feedforward_ptr = &chassis_force_ff_lf,
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type       = SPEED_LOOP,
            .close_loop_type       = SPEED_LOOP,
            .feedforward_flag      = CURRENT_FEEDFORWARD,
        },
        .motor_type = M3508,
    };
    Motor_Init_Config_s chassis_motor_config_2 = {
        .can_init_config.can_handle   = &hcan1,
        .controller_param_init_config = {
            .speed_PID = {
                .Kp            = 3 ,// 4.5 , 3.75
                .Ki            = 0,   // 0
                .Kd            = 0,   // 0
                .IntegralLimit = 3000,
                .Improve       = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .MaxOut        = 16000,
            },
            .current_feedforward_ptr = &chassis_force_ff_rf,
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type       = SPEED_LOOP,
            .close_loop_type       = SPEED_LOOP,
            .feedforward_flag      = CURRENT_FEEDFORWARD,
        },
        .motor_type = M3508,
    };
    Motor_Init_Config_s chassis_motor_config_3 = {
        .can_init_config.can_handle   = &hcan1,
        .controller_param_init_config = {
            .speed_PID = {
                .Kp            = 3 ,// 4.5
                .Ki            = 0,   // 0
                .Kd            = 0,   // 0
                .IntegralLimit = 3000,
                .Improve       = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .MaxOut        = 16000,
            },
            .current_feedforward_ptr = &chassis_force_ff_rb,
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type       = SPEED_LOOP,
            .close_loop_type       = SPEED_LOOP,
            .feedforward_flag      = CURRENT_FEEDFORWARD,
        },
        .motor_type = M3508,
    };
    Motor_Init_Config_s chassis_motor_config_4 = {
        .can_init_config.can_handle   = &hcan1,
        .controller_param_init_config = {
            .speed_PID = {
                .Kp            = 3 ,// 4.5
                .Ki            = 0,   // 0
                .Kd            = 0,   // 0
                .IntegralLimit = 3000,
                .Improve       = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .MaxOut        = 16000,
            },
            .current_feedforward_ptr = &chassis_force_ff_lb,
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type       = SPEED_LOOP,
            .close_loop_type       = SPEED_LOOP,
            .feedforward_flag      = CURRENT_FEEDFORWARD,
        },
        .motor_type = M3508,
    };
/*---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
/*定义四份 Motor_Init_Config_s：四个轮电机都走 hcan1，控制方式都是速度闭环，反馈源都是电机自身反馈，电机型号是 M3508；四份配置的主要差别是各自绑定的 current_feedforward_ptr*/
/*---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
    //  @todo: 当前还没有设置电机的正反转,仍然需要手动添加reference的正负号,需要电机module的支持,待修改.
    chassis_motor_config_1.can_init_config.tx_id                             = 1;
    chassis_motor_config_1.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    motor_lf                                                               = DJIMotorInit(&chassis_motor_config_1);
    
    chassis_motor_config_2.can_init_config.tx_id                             = 2;
    chassis_motor_config_2.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    motor_rf                                                               = DJIMotorInit(&chassis_motor_config_2);
    
    chassis_motor_config_3.can_init_config.tx_id                             = 3;
    chassis_motor_config_3.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    motor_rb                                                               = DJIMotorInit(&chassis_motor_config_3);

    chassis_motor_config_4.can_init_config.tx_id                             = 4;
    chassis_motor_config_4.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    motor_lb                                                               = DJIMotorInit(&chassis_motor_config_4);
/*---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
    /*tx_id = 1/2/3/4 后分别调用 DJIMotorInit()，把四个配置实例化成 motor_lf、motor_rf、motor_rb、motor_lb */
    /*---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/    
    Motor_Init_Config_s sync_belt_motor_config = {
        .can_init_config.can_handle   = &hcan2,
        .controller_param_init_config = {
            .speed_PID = {
                .Kp            = 3,
                .Ki            = 0,
                .Kd            = 0,
                .IntegralLimit = 3000,
                .Improve       = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .MaxOut        = 16000,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type       = SPEED_LOOP,
            .close_loop_type       = SPEED_LOOP,
            .feedforward_flag      = CURRENT_FEEDFORWARD,
        },
        .motor_type = M3508,
    };

    sync_belt_motor_config.can_init_config.tx_id                             = 3;
    sync_belt_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    sync_belt_motor_l                                                        = DJIMotorInit(&sync_belt_motor_config);

    sync_belt_motor_config.can_init_config.tx_id                             = 4;
    sync_belt_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    sync_belt_motor_r                                                        = DJIMotorInit(&sync_belt_motor_config);

    Motor_Init_Config_s joint_motor_config = {
        .can_init_config.can_handle = &hcan2,
        .motor_type = DM_Motor,
        .controller_param_init_config ={
            .angle_PID = {
                .Kp = 15,
                .Ki = 0.0,
                .Kd = 0.01,
                .DeadBand = 0,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit ,
                .IntegralLimit = 0.5,
                .MaxOut = 10,
            },
            .speed_PID = {
                .Kp = 25,
                .Ki = 0,
                .Kd = 0.03,
                .DeadBand = 0,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit,
                .IntegralLimit = 0.5,
                .MaxOut = 25,
            },
            //  .other_angle_feedback_ptr = &gimbal_IMU_data->output.INS_angle[INS_PITCH_ADDRESS_OFFSET], // pitch?????
             .other_angle_feedback_ptr = &Chassis_IMU_data->output.INS_angle[0],  //等待修改   
            // ??????????????,????,ins_task.md??c??bodyframe?????
            .other_speed_feedback_ptr = &Chassis_IMU_data->INS_data.INS_gyro[1],
            // .current_feedforward_ptr = &joint_tor_feedforward,
        },
        .controller_setting_init_config = {
            .angle_feedback_source = OTHER_FEED,
            .speed_feedback_source = OTHER_FEED,
            .outer_loop_type       = ANGLE_LOOP,
            .close_loop_type       = SPEED_LOOP | ANGLE_LOOP,
            .motor_reverse_flag    = MOTOR_DIRECTION_NORMAL,
            .feedforward_flag      = FEEDFORWARD_NONE,
            .control_range = {
                .P_max = 12.5,
                .V_max = 10,
                .T_max = 28,
            },
        },
    };
/*---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
/*配置两个 DM 关节电机：走 hcan2，外环角度、内环速度，角度和角速度都不取电机自身反馈，而是取 Chassis_IMU_data->output.INS_angle[1] 和 INS_gyro[0] 这些“外部反馈”；关节控制用机体姿态做闭环*/
/*---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
    joint_motor_config.can_init_config.tx_id = 0x01;
    joint_motor_config.can_init_config.rx_id = 0x11;
    joint_motor_config.controller_setting_init_config.feedback_reverse_flag = FEEDBACK_DIRECTION_NORMAL;
    joint_motor_config.controller_param_init_config.current_feedforward_ptr = &joint_l_tor_feedforward;
    joint_l = DMMotorInit(&joint_motor_config);
    //左关节反馈方向正常，前馈绑定 joint_l_tor_feedforward，tx_id 0x01，rx_id 0x11 实例化成 joint_l
    joint_motor_config.can_init_config.tx_id = 0x02;
    joint_motor_config.can_init_config.rx_id = 0x12;
    joint_motor_config.controller_setting_init_config.feedback_reverse_flag = FEEDBACK_DIRECTION_REVERSE;
    joint_motor_config.controller_param_init_config.current_feedforward_ptr = &joint_r_tor_feedforward;
    joint_r = DMMotorInit(&joint_motor_config);
    //右关节反馈方向反向，前馈绑定 joint_r_tor_feedforward，tx_id 0x02，rx_id 0x12 实例化成 joint_r
/*---------------------------------------------------------------------------------初始化并使能超级电容------------------------------------------------------------------------------*/
    SuperCap_Init_Config_s supercap_config = {
        .can_config = {
            .can_handle = &hcan1,
        },
    };
    supercap = SuperCapInit(&supercap_config);
    SuperCapEnable(supercap);
/*----------------------------------------------GPIOE 的 PE13/PE9/PE11被配置成推挽输出，后面爬坡模式会用它们控制外设-------------------------------------------------------------------*/
    GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_9|GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_RESET);
    // HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_SET);
    #endif
/*由于之前在 ChassisTask 的运动解算里把左右后轮的 PID 输出当成了力控前馈来用，所以它们的积分项会积累一个值，这个值在某些情况下会比较大，导致底盘在低速时震荡。这里在初始化的时候把它们清零，之后如果需要用到 PID 的积分项再根据情况调整---------------------------------*/

    // motor_lb->motor_controller.speed_PID.Iout=0;motor_rb->motor_controller.speed_PID.Iout=0;
    // motor_lf->motor_controller.speed_PID.Iout=0;motor_rf->motor_controller.speed_PID.Iout=0;
/*---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
#if defined(ONE_BOARD) || defined(CHASSIS_BOARD) // 单板控制整车,则通过pubsub来传递消息SubRegister("chassis_cmd", ...) 和 PubRegister("chassis_feed", ...) 完成消息通道注册
    chassis_sub = SubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));
    chassis_pub = PubRegister("chassis_feed", sizeof(Chassis_Upload_Data_s));
 #endif // ONE_BOARD
    ;
}
/*---------------------------------------------------------------------------------------辅助函数------------------------------------------------------------------------------------*/

/**
 * @brief 标准摩擦补偿：定值随指令方向变号，带智能死区
 * @param velocity_cmd 速度指令（度/秒）
 * @param compensation 补偿定值
 * @param last_sign 上次方向状态指针（用于滞回）
 * @return 补偿力（与指令方向相反）
 */
/*--------------------------------------1.calculateFrictionCompensation()------->2.updateAllWheelsFriction----------------------------------------------------------------------*/
static float calculateFrictionCompensation(float velocity_cmd, float compensation, int8_t *last_sign)
{
    //无指令时：补偿归零，确保绝对静止
    if (fabsf(velocity_cmd) < 1.0f) {
        *last_sign = 0;
        return 0.0f;
    }
    
    int8_t current_sign;
    
    // 滞回判断：防止方向符号抖动
    if (fabsf(velocity_cmd) > FRICTION_KICK_THRESHOLD + FRICTION_HYSTERESIS) {
        current_sign = (velocity_cmd > 0.0f) ? 1 : -1;
        *last_sign = current_sign;
    } else if (fabsf(velocity_cmd) < FRICTION_KICK_THRESHOLD - FRICTION_HYSTERESIS) {
        // 低速启动区：给kick启动力
        if (*last_sign == 0) {
            current_sign = (velocity_cmd > 0.0f) ? 1 : -1;
            *last_sign = current_sign;
        } else {
            current_sign = *last_sign;  // 保持上次方向
        }
    } else {
        // 滞回带内：保持上次状态，防止翻符号
        current_sign = (*last_sign != 0) ? *last_sign : ((velocity_cmd > 0.0f) ? 1 : -1);
        *last_sign = current_sign;
    }
    return -compensation * (float)current_sign;
}
/*-----------------如果命令速度几乎为 0，就把补偿清零；否则根据速度正负和滞回区决定当前方向，再返回一个“与指令方向相反符号”的固定补偿值---------------*/

/**
 * @brief 更新四个轮子的摩擦补偿
 * @param velocity_cmd_* 各轮速度指令（度/秒）
 */
/*----------------------------------------2.updateAllWheelsFriction()------>3.MecanumCalculate----------------------------------------------------------------------*/
static void updateAllWheelsFriction(float velocity_cmd_lf, float velocity_cmd_rf, float velocity_cmd_lb, float velocity_cmd_rb)
{
    
    float target_lf = calculateFrictionCompensation(velocity_cmd_lf, friction_compensation_lf, &sign_lf);
    float target_rf = calculateFrictionCompensation(velocity_cmd_rf, friction_compensation_rf, &sign_rf);
    float target_lb = calculateFrictionCompensation(velocity_cmd_lb, friction_compensation_lb, &sign_lb);
    float target_rb = calculateFrictionCompensation(velocity_cmd_rb, friction_compensation_rb, &sign_rb);
    // 平滑过渡到目标值
    friction_lf_state += FRICTION_SMOOTHING * (target_lf - friction_lf_state);
    friction_rf_state += FRICTION_SMOOTHING * (target_rf - friction_rf_state);
    friction_lb_state += FRICTION_SMOOTHING * (target_lb - friction_lb_state);
    friction_rb_state += FRICTION_SMOOTHING * (target_rb - friction_rb_state);
    // 调试输出
    
}
/*-----------------给四个轮子分别算摩擦补偿目标，再做一阶平滑，避免补偿突变------------------------*/

/*-----------------max_abs4:取四个数里绝对值最大的一个------------------------------------------*/
static float max_abs4(float a, float b, float c, float d)
{
    float max_abs = fabsf(a);
    float tmp = fabsf(b);
    if (tmp > max_abs)
        max_abs = tmp;
    tmp = fabsf(c);
    if (tmp > max_abs)
        max_abs = tmp;
    tmp = fabsf(d);
    if (tmp > max_abs)
        max_abs = tmp;
    return max_abs;
}
/*-----------------返回四个数里绝对值最大的一个-------------------------*/
/*EstimateMecanumWzObs*/
static float EstimateMecanumWzObs(void)
{
    if (motor_lf == NULL || motor_rf == NULL || motor_lb == NULL || motor_rb == NULL) {
        return 0.0f;
    }

    const float w_lf = motor_lf->measure.speed_aps;
    const float w_rf = motor_rf->measure.speed_aps;
    const float w_lb = motor_lb->measure.speed_aps;
    const float w_rb = motor_rb->measure.speed_aps;

    const float wz_lf = (fabsf(LF_CENTER) > 1e-6f) ? (w_lf / LF_CENTER) : 0.0f;
    const float wz_rf = (fabsf(RF_CENTER) > 1e-6f) ? (w_rf / RF_CENTER) : 0.0f;
    const float wz_lb = (fabsf(LB_CENTER) > 1e-6f) ? (w_lb / LB_CENTER) : 0.0f;
    const float wz_rb = (fabsf(RB_CENTER) > 1e-6f) ? (w_rb / RB_CENTER) : 0.0f;

    return (wz_lf + wz_rf + wz_lb + wz_rb) * 0.25f;
}
/*---------------利用四个轮子的实际速度反推当前底盘角速度观测值------------*/
/*------------------------------------4.RotateModeTranslateScale()---->3.MecanumCalculate----------------------------------------------------*/
static float RotateModeTranslateScale(float rot_lf, float rot_rf, float rot_lb, float rot_rb,float trans_lf, float trans_rf, float trans_lb, float trans_rb)
{
    float target_scale = 1.0f;

    if (chassis_cmd_recv.chassis_mode == CHASSIS_ROTATE) {
        const float rot_peak = max_abs4(rot_lf, rot_rf, rot_lb, rot_rb);
        const float trans_peak = max_abs4(trans_lf, trans_rf, trans_lb, trans_rb);

        if (trans_peak > 1e-3f) {
            float wheel_headroom = CHASSIS_ROTATE_WHEEL_REF_LIMIT - rot_peak * CHASSIS_ROTATE_WZ_RESERVE_RATIO;
            if (wheel_headroom < 0.0f)
                wheel_headroom = 0.0f;

            target_scale = wheel_headroom / (trans_peak * CHASSIS_ROTATE_VXY_POWER_COEF);
            if (target_scale > 1.0f)
                target_scale = 1.0f;
            if (target_scale < CHASSIS_ROTATE_VXY_MIN_SCALE)
                target_scale = CHASSIS_ROTATE_VXY_MIN_SCALE;
        }

        // 旋转优先：若实际小陀螺角速度明显跟不上，继续下压平移权重
        {
            const float wz_cmd_abs = fabsf(chassis_cmd_recv.wz);
            if (wz_cmd_abs > 100.0f) {
                float wz_ratio = fabsf(EstimateMecanumWzObs()) / wz_cmd_abs;
                if (wz_ratio > 1.2f)
                    wz_ratio = 1.2f;
                rotate_wz_track_ratio_state += 0.10f * (wz_ratio - rotate_wz_track_ratio_state);

                if (rotate_wz_track_ratio_state < 0.95f) {
                    float protect_scale = rotate_wz_track_ratio_state / 0.95f;
                    if (protect_scale < 0.20f)
                        protect_scale = 0.20f;
                    target_scale *= protect_scale;
                }
            } else {
                rotate_wz_track_ratio_state += 0.10f * (1.0f - rotate_wz_track_ratio_state);
            }
        }

        rotate_vxy_scale_state += CHASSIS_ROTATE_VXY_SCALE_FILTER_ALPHA * (target_scale - rotate_vxy_scale_state);
    } else {
        rotate_vxy_scale_state = 1.0f;
        rotate_wz_track_ratio_state = 1.0f;
    }

    return rotate_vxy_scale_state;
}
/*------------------只在 CHASSIS_ROTATE 下生效。先看旋转已经占掉多少轮速余量，再决定平移量最多还能放多大；如果发现实际 wz 跟不上指令，会进一步压低平移比例，优先保自旋----------------------*/
/*---------------------------------------------------------3.MecanumCalculate()--->核心任务ChassisTask()-------------------------------------------------*/
static void MecanumCalculate()
{
    const float rot_lf = chassis_cmd_recv.wz * LF_CENTER;
    const float rot_rf = chassis_cmd_recv.wz * RF_CENTER;
    const float rot_lb = chassis_cmd_recv.wz * LB_CENTER;
    const float rot_rb = chassis_cmd_recv.wz * RB_CENTER;

    const float trans_lf = chassis_vx + chassis_vy;
    const float trans_rf = -chassis_vx + chassis_vy;
    const float trans_lb = chassis_vx - chassis_vy;
    const float trans_rb = -chassis_vx - chassis_vy;

    const float trans_scale = RotateModeTranslateScale(rot_lf, rot_rf, rot_lb, rot_rb,trans_lf, trans_rf, trans_lb, trans_rb);

    vt_lf = rot_lf + trans_lf * trans_scale;
    vt_rf = rot_rf + trans_rf * trans_scale;
    vt_lb = rot_lb + trans_lb * trans_scale;
    vt_rb = rot_rb + trans_rb * trans_scale;

    updateAllWheelsFriction(vt_lf, vt_rf, vt_lb, vt_rb);
}
//把底盘平移速度 chassis_vx/chassis_vy 和旋转速度 wz 分解成四个轮子的目标转速 vt_*。严格说这是“逆运动学”，虽然注释里写的是正运动学。
//-----------------------------------5.clamp_absf()-------->8.ChassisForceControlMecanum()------>核心任务 ChassisTask()-------------------------------------------------*/
static float clamp_absf(float value, float max_abs)
{
    if (value > max_abs)
        return max_abs;
    if (value < -max_abs)
        return -max_abs;
    return value;
}
//把一个值限制在[-max_abs, max_abs]范围内，保持符号不变

static float clamp_rangef(float value, float min_value, float max_value)
{
    if (value > max_value)
        return max_value;
    if (value < min_value)
        return min_value;
    return value;
}

//-----------------------------------6.ChassisForceReset()------>8.ChassisForceControlMecanum()------>核心任务 ChassisTask()-------------------------------------------------*/
static void ChassisForceReset(void)
{
    chassis_force_ff_lf = 0.0f;
    chassis_force_ff_rf = 0.0f;
    chassis_force_ff_lb = 0.0f;
    chassis_force_ff_rb = 0.0f;
}
//把四个轮子的力控前馈清零，通常在模式切换或者特殊动作结束时调用，防止残留的前馈导致底盘不受控制地动起来

static void ChassisForceSlewToTarget(float lf_target, float rf_target, float lb_target, float rb_target)
{
    chassis_force_ff_lf += clamp_absf(lf_target - chassis_force_ff_lf, CHASSIS_FORCE_CURRENT_FF_SLEW_STEP);
    chassis_force_ff_rf += clamp_absf(rf_target - chassis_force_ff_rf, CHASSIS_FORCE_CURRENT_FF_SLEW_STEP);
    chassis_force_ff_lb += clamp_absf(lb_target - chassis_force_ff_lb, CHASSIS_FORCE_CURRENT_FF_SLEW_STEP);
    chassis_force_ff_rb += clamp_absf(rb_target - chassis_force_ff_rb, CHASSIS_FORCE_CURRENT_FF_SLEW_STEP);
}

//----------------------------------7.ObserveMecanumBodySpeed()------>8.ChassisForceControlMecanum()------>核心任务 ChassisTask()-------------------------------------------------*/
static uint8_t ObserveMecanumBodySpeed(float *vx_obs, float *vy_obs, float *wz_obs)
{
    if (motor_lf == NULL || motor_rf == NULL || motor_lb == NULL || motor_rb == NULL) {
        *vx_obs = 0.0f;
        *vy_obs = 0.0f;
        *wz_obs = 0.0f;
        chassis_force_obs_inited = 0u;
        return 0u;
    }

    if (motor_lf->daemon == NULL || motor_rf->daemon == NULL ||
        motor_lb->daemon == NULL || motor_rb->daemon == NULL ||
        !DaemonIsOnline(motor_lf->daemon) ||
        !DaemonIsOnline(motor_rf->daemon) ||
        !DaemonIsOnline(motor_lb->daemon) ||
        !DaemonIsOnline(motor_rb->daemon)) {
        *vx_obs = 0.0f;
        *vy_obs = 0.0f;
        *wz_obs = 0.0f;
        chassis_force_obs_inited = 0u;
        return 0u;
    }

    const float w_lf = motor_lf->measure.speed_aps;
    const float w_rf = motor_rf->measure.speed_aps;
    const float w_lb = motor_lb->measure.speed_aps;
    const float w_rb = motor_rb->measure.speed_aps;

    float vx_raw = (w_lf - w_rf + w_lb - w_rb) * 0.25f;
    float vy_raw = (w_lf + w_rf - w_lb - w_rb) * 0.25f;

    const float wz_lf = (fabsf(LF_CENTER) > 1e-6f) ? (w_lf / LF_CENTER) : 0.0f;
    const float wz_rf = (fabsf(RF_CENTER) > 1e-6f) ? (w_rf / RF_CENTER) : 0.0f;
    const float wz_lb = (fabsf(LB_CENTER) > 1e-6f) ? (w_lb / LB_CENTER) : 0.0f;
    const float wz_rb = (fabsf(RB_CENTER) > 1e-6f) ? (w_rb / RB_CENTER) : 0.0f;
    float wz_raw = (wz_lf + wz_rf + wz_lb + wz_rb) * 0.25f;

    if (!chassis_force_obs_inited) {
        chassis_force_obs_vx_state = vx_raw;
        chassis_force_obs_vy_state = vy_raw;
        chassis_force_obs_wz_state = wz_raw;
        chassis_force_obs_inited = 1u;
    } else {
        chassis_force_obs_vx_state += CHASSIS_FORCE_OBS_FILTER_ALPHA * (vx_raw - chassis_force_obs_vx_state);
        chassis_force_obs_vy_state += CHASSIS_FORCE_OBS_FILTER_ALPHA * (vy_raw - chassis_force_obs_vy_state);
        chassis_force_obs_wz_state += CHASSIS_FORCE_OBS_FILTER_ALPHA * (wz_raw - chassis_force_obs_wz_state);
    }

    *vx_obs = (fabsf(chassis_force_obs_vx_state) < CHASSIS_FORCE_OBS_DEADBAND) ? 0.0f : chassis_force_obs_vx_state;
    *vy_obs = (fabsf(chassis_force_obs_vy_state) < CHASSIS_FORCE_OBS_DEADBAND) ? 0.0f : chassis_force_obs_vy_state;
    *wz_obs = (fabsf(chassis_force_obs_wz_state) < CHASSIS_FORCE_OBS_DEADBAND) ? 0.0f : chassis_force_obs_wz_state;
    return 1u;
}
//根据四个轮子的实际转速反推当前底盘的平移速度 vx_obs、vy_obs 和旋转速度 wz_obs，这些观测值会被 ChassisForceControlMecanum() 用来做力控前馈计算，实现对底盘动力的闭环控制
/*-----------------------------------8.ChassisForceControlMecanum()------>核心任务 ChassisTask()-------------------------------------------------*/
static void ChassisForceControlMecanum(void)
{
    float vx_obs, vy_obs, wz_obs;
    if (!ObserveMecanumBodySpeed(&vx_obs, &vy_obs, &wz_obs)) {
        ChassisForceSlewToTarget(0.0f, 0.0f, 0.0f, 0.0f);
        return;
    }

    /* 速度误差闭环成目标加速度（RMCS 5.2.4） */
    const float ax_cmd = clamp_absf((chassis_vx - vx_obs) * CHASSIS_FORCE_VEL_P, CHASSIS_FORCE_ACC_LIMIT);
    const float ay_cmd = clamp_absf((chassis_vy - vy_obs) * CHASSIS_FORCE_VEL_P, CHASSIS_FORCE_ACC_LIMIT);
    const float az_cmd = clamp_absf((chassis_cmd_recv.wz - wz_obs) * CHASSIS_FORCE_WZ_P, CHASSIS_FORCE_WZ_ACC_LIMIT);

    /* 目标加速度映射为底盘平移力和旋转力矩（RMCS 5.2.5） */
    const float fx = CHASSIS_FORCE_EQ_MASS * ax_cmd;
    const float fy = CHASSIS_FORCE_EQ_MASS * ay_cmd;
    const float tz = CHASSIS_FORCE_EQ_INERTIA * az_cmd;

    float lever = (fabsf(LF_CENTER) + fabsf(RF_CENTER) + fabsf(LB_CENTER) + fabsf(RB_CENTER)) * 0.25f;
    if (lever < 1e-3f)
        lever = 1.0f;
    const float fr = tz / lever;

    const float f_lf = 0.25f * (fx + fy + fr);
    const float f_rf = 0.25f * (-fx + fy + fr);
    const float f_lb = 0.25f * (fx - fy + fr);
    const float f_rb = 0.25f * (-fx - fy + fr);

    /* 轮作用力映射为电流前馈 */
    const float ff_lf_target = clamp_absf(f_lf * CHASSIS_FORCE_TO_CURRENT, CHASSIS_FORCE_CURRENT_FF_LIMIT);
    const float ff_rf_target = clamp_absf(f_rf * CHASSIS_FORCE_TO_CURRENT, CHASSIS_FORCE_CURRENT_FF_LIMIT);
    const float ff_lb_target = clamp_absf(f_lb * CHASSIS_FORCE_TO_CURRENT, CHASSIS_FORCE_CURRENT_FF_LIMIT);
    const float ff_rb_target = clamp_absf(f_rb * CHASSIS_FORCE_TO_CURRENT, CHASSIS_FORCE_CURRENT_FF_LIMIT);
    ChassisForceSlewToTarget(ff_lf_target, ff_rf_target, ff_lb_target, ff_rb_target);
}
//先算速度误差，再乘比例得到目标加速度，再映射成车体平移力 fx/fy 和旋转力矩 tz，最后分配到四个轮子并换算成电流前馈；这就是麦轮力控增强。
/*--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/


static ramp_t super_ramp;// 超电功率斜坡
static float Power_Output;// 最终的功率输出值，经过能量环和电压环修正后的结果
const float buffer_energy_loop_kp = 0.5f;// 缓冲能量环比例系数
const float cap_voltage_loop_kp = 20.0f;// 电容电压环比例系数

/*---------------9.Super_Cap_control---->核心任务 ChassisTask()-------------------*/
 void Super_Cap_control()
 {
    float buffer_power_rectification, cap_power_rectification;
    float buffer_energy_target, buffer_energy_actual, cap_voltage_target, cap_voltage_actual;

    Power_Output = chassis_cmd_recv.power_limit;
    buffer_energy_actual = chassis_cmd_recv.power_buffer;

    //缓冲能量环
    buffer_energy_target = 50.0f;
    buffer_power_rectification = (buffer_energy_actual - buffer_energy_target) * buffer_energy_loop_kp;
    Power_Output += buffer_power_rectification;

    //电容能量环
    if (SuperCapIsOnline(supercap))
    {
        cap_voltage_actual = SuperCapGetChassisVoltage(supercap);
        // Ignore invalid voltage samples to avoid pulling power limit to a large negative value.
        if (cap_voltage_actual > 5.0f && cap_voltage_actual < 35.0f) {
        if (chassis_cmd_recv.SuperCap_flag_from_user == SUPERCAP_USE)
            cap_voltage_target = 15.0f;
        else 
            cap_voltage_target = 23.0f;
        cap_power_rectification = (cap_voltage_actual - cap_voltage_target) * cap_voltage_loop_kp;
        if (cap_power_rectification > 50.0f) cap_power_rectification = 70.0f;
                    if (cap_power_rectification < -50.0f) cap_power_rectification = -50.0f;
        Power_Output += cap_power_rectification;
        }
    }
    
    Power_Output -= 10.0f; //静态功耗
        if (Power_Output < 0.0f) {
        Power_Output = 0.0f;
    }

    PowerControlupdate(Power_Output, 1.0f / REDUCTION_RATIO_WHEEL);// 把修正后的功率值送进能量环更新函数，得到新的速度限制值

    SuperCapSetPowerLimit(supercap, chassis_cmd_recv.power_limit);
     // 设定速度参考值
    DJIMotorSetRef(motor_lf, vt_lf);
    DJIMotorSetRef(motor_rf, vt_rf);
    DJIMotorSetRef(motor_lb, vt_lb);
    DJIMotorSetRef(motor_rb, vt_rb);
 }
//power_limit 起步，再根据裁判系统回传的 power_buffer 做一轮修正；如果超级电容在线，再根据电容电压是否偏离目标继续修正；减掉一部分静态功耗后，把结果送进 PowerControlupdate()；最后把四个轮子的目标转速 vt_* 写给电机

// 获取功率裆位

 static void Power_get()
 {
     //cap->cap_msg_g.power_limit = chassis_cmd_recv.power_limit - 30 + 30 * (cap->cap_msg_s.CapVot - 17.0f) / 6.0f;
 }

extern Power_Data_s power_data; // 电机功率数据;
extern float power_value;// 当前功率值
uint64_t can_send_count=0;// 发送计数器
float offset_angle_watch;
float text_vx=0;
static float freequence = 0,freequence_last=0; 
extern float half_to_float(uint16_t half);
extern Chassis_Ctrl_Cmd_s_uart chassis_rs485_recv;
 float follow_kp,follow_kd;
float super_speed=10000;
float textkd=6,text_speed=10000;
static float getRotato_K(uint8_t level){
    switch(level){
        case 1:
            return textkd;
            break;
        case 2:
            return 75;
            break;
            case 3:
            return 80;
            break;
            case 4:
            return 85;
            break;
            case 5:
            return 90;
            break;
            case 6:
            return 95;
            break;
            case 7:
            return 100;
            break;
            case 8:
            return 105;
            break;
            case 9:
            return 110;
            break;
            case 10:
            return 120;
            break;
        default:
            return 100;
}
}
extern uint16_t powerLim;
static float getWzspeed(uint16_t powerLimit){
    switch (powerLimit){
        case 70://------------------
        return 3500;
        break;
        case 75:
        return 3750;
        break;
        case 80://------------------
        return 4000;
        break;
        case 85:
        return 4175;
        break;
        case 90:
        return 4350;
        break;
        case 95:
        return 4525;
        break;
        case 100://------------------
        return 4700;
        break;
        case 105:
        return 4900;
        break;
        case 110:
        return 5100;
        break;
        case 120://------------------
        return 5300;
        break;
        default:
        return 3000;
        break;
    }
}

void joint_limit(float l_target, float r_target)
{
    if(l_target > JOINT_LEFT_UP_LIMIT)
        l_target = JOINT_LEFT_UP_LIMIT;
    if(l_target < JOINT_LEFT_DOWN_LIMIT)
        l_target = JOINT_LEFT_DOWN_LIMIT;
    joint_l->ctrl.pos_set = l_target;

    if(r_target > JOINT_RIGHT_UP_LIMIT)
        r_target = JOINT_RIGHT_UP_LIMIT;
    if(r_target < JOINT_RIGHT_DOWN_LIMIT)
        r_target = JOINT_RIGHT_DOWN_LIMIT;
    joint_r->ctrl.pos_set = r_target;
}
//把左右关节目标角度限制在机械允许范围内，再写入 joint_l->ctrl.pos_set 和 joint_r->ctrl.pos_set。


float offset_angle_watch;
float angle_l,angle_r;
volatile static float angle_avg;//, tor_avg;
// float angle_measure_all[50];//,tor_measure_all[10];
// float angle_measure_sum;//, tor_measure_sum;
float angle_target, angle_l_target, angle_r_target;
int16_t avg_count = 0, avg_i;
// float l_offset = -0.311236, r_offset = 0.0434394;
float l_offset = 0.621374, r_offset = 0.8515027;
float length_l_measure,length_r_measure,length_measure;
float length_target;
float angle_test = 0.07f;//0.17;
// float length_test = 0.2;
float leg_p = 0.3;

float safe_sqrt(float x)
{
    if(x < 0.0f)
    {
        return 0.0f;
    }
    return __builtin_sqrtf(x);
}

static float CalcRetractTorqueFeedforward(float joint_angle)
{
    float angle = joint_angle;
    float torque_ff;

    if (angle < LEG_RETRACT_TORQUE_MIN_ANGLE)
        angle = LEG_RETRACT_TORQUE_MIN_ANGLE;
    else if (angle > LEG_RETRACT_TORQUE_MAX_ANGLE)
        angle = LEG_RETRACT_TORQUE_MAX_ANGLE;

    torque_ff = LEG_RETRACT_TORQUE_FF_A * angle * angle
              + LEG_RETRACT_TORQUE_FF_B * angle
              + LEG_RETRACT_TORQUE_FF_C;

    if (torque_ff < 0.0f)
        torque_ff = 0.0f;
    else if (torque_ff > LEG_RETRACT_TORQUE_MAX)
        torque_ff = LEG_RETRACT_TORQUE_MAX;

    return torque_ff;
}

static uint8_t LegRetractManualBoostEnabled(void)
{
    if (!RemoteControlIsOnline())
        return 0u;

    if (rc_ctrl[TEMP].rc.switch_right == RC_SW_UP &&
        rc_ctrl[TEMP].rc.switch_left == RC_SW_DOWN)
        return 1u;

    if (rc_ctrl[TEMP].key[KEY_PRESS].z != 0u)
        return 1u;

    return 0u;
}

static uint8_t ChassisModeIsClimbSequence(chassis_mode_e mode)
{
    return (mode == CHASSIS_CLIMB ||
            mode == CHASSIS_CLIMB_RETRACT ||
            mode == CHASSIS_CLIMB_WITH_PULL ||
            mode == CHASSIS_CLIMB_WITH_PUSH ||
            mode == CHASSIS_FLY_SLOPE) ? 1u : 0u;
}

static uint8_t LegAutoRetractTrigger(float pitch_target)
{
    float pitch_err;
    float pitch_gyro_abs;
    float torque_avg_abs;
    uint8_t attitude_hit;
    uint8_t torque_hit;

    if (chassis_cmd_recv.chassis_mode != CHASSIS_CLIMB)
        return 0u;

    if (chassis_cmd_recv.vx < LEG_AUTO_RETRACT_VX_THRESHOLD)
        return 0u;

    pitch_err = Chassis_IMU_data->output.INS_angle[INS_PITCH_ADDRESS_OFFSET] - pitch_target;
    pitch_gyro_abs = fabsf(Chassis_IMU_data->INS_data.INS_gyro[INS_PITCH_ADDRESS_OFFSET]);
    torque_avg_abs = (fabsf(joint_l->measure.tor) + fabsf(joint_r->measure.tor)) * 0.5f;

    if (pitch_err > LEG_AUTO_RETRACT_PITCH_ERR_MAX ||
        pitch_gyro_abs > LEG_AUTO_RETRACT_PITCH_GYRO_MAX ||
        torque_avg_abs > LEG_AUTO_RETRACT_TORQUE_MAX_TRIGGER)
        return 0u;

    attitude_hit = (pitch_err > LEG_AUTO_RETRACT_PITCH_ERR_TRIGGER ||
                    pitch_gyro_abs > LEG_AUTO_RETRACT_PITCH_GYRO_TRIGGER) ? 1u : 0u;
    torque_hit = (torque_avg_abs > LEG_AUTO_RETRACT_TORQUE_TRIGGER) ? 1u : 0u;

    return (attitude_hit && torque_hit) ? 1u : 0u;
}

static uint8_t LegEdgeHitDetected(float pitch_target)
{
    float pitch_err;
    float pitch_gyro;
    float torque_l_abs;
    float torque_r_abs;
    float torque_max_abs;
    float joint_vel_l_abs;
    float joint_vel_r_abs;
    uint8_t joint_stalled;

    if (chassis_cmd_recv.chassis_mode != CHASSIS_CLIMB)
        return 0u;

    if (chassis_cmd_recv.vx < LEG_EDGE_HIT_VX_THRESHOLD)
        return 0u;

    pitch_err = Chassis_IMU_data->output.INS_angle[INS_PITCH_ADDRESS_OFFSET] - pitch_target;
    pitch_gyro = Chassis_IMU_data->INS_data.INS_gyro[INS_PITCH_ADDRESS_OFFSET];
    torque_l_abs = fabsf(joint_l->measure.tor);
    torque_r_abs = fabsf(joint_r->measure.tor);
    torque_max_abs = (torque_l_abs > torque_r_abs) ? torque_l_abs : torque_r_abs;
    joint_vel_l_abs = fabsf(joint_l->measure.vel);
    joint_vel_r_abs = fabsf(joint_r->measure.vel);
    joint_stalled = (joint_vel_l_abs < LEG_EDGE_HIT_JOINT_VEL_THRESHOLD ||
                     joint_vel_r_abs < LEG_EDGE_HIT_JOINT_VEL_THRESHOLD) ? 1u : 0u;

    if (torque_max_abs < LEG_EDGE_HIT_TORQUE_TRIGGER)
        return 0u;

    if (!joint_stalled)
        return 0u;

    if (pitch_err > LEG_EDGE_HIT_PITCH_ERR_TRIGGER ||
        pitch_gyro > LEG_EDGE_HIT_PITCH_GYRO_TRIGGER)
        return 1u;

    return 0u;
}

static void RecoverCan1AfterSuperCapOffline(void)
{
    static uint8_t supercap_seen_online = 0;
    static uint8_t recovery_armed = 0;

    if (supercap == NULL)
    {
        return;
    }

    if (SuperCapIsOnline(supercap))
    {
        supercap_seen_online = 1;
        recovery_armed = 1;
        return;
    }

    if (!supercap_seen_online || !recovery_armed)
    {
        return;
    }

    HAL_CAN_Stop(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_RX_FIFO1_MSG_PENDING);
    HAL_CAN_Start(&hcan1);
    recovery_armed = 0;
}

static void UpdateChassisDebugFeedback(void)
{
    uint8_t i;
    float vx_obs = 0.0f;
    float vy_obs = 0.0f;
    float wz_obs = 0.0f;

    ObserveMecanumBodySpeed(&vx_obs, &vy_obs, &wz_obs);
    chassis_feedback_data.real_vx = vx_obs;
    chassis_feedback_data.real_vy = vy_obs;
    chassis_feedback_data.real_wz = wz_obs;
    chassis_feedback_data.capget_power_limit = chassis_cmd_recv.power_limit;

    if (Chassis_IMU_data != NULL)
    {
        for (i = 0; i < 3; i++)
        {
            chassis_feedback_data.chassis_imu_data[i] = Chassis_IMU_data->output.INS_angle[i];
        }
        chassis_feedback_data.chassis_pitch = Chassis_IMU_data->output.INS_angle[INS_PITCH_ADDRESS_OFFSET];
    }
    else
    {
        for (i = 0; i < 3; i++)
        {
            chassis_feedback_data.chassis_imu_data[i] = 0.0f;
        }
        chassis_feedback_data.chassis_pitch = 0.0f;
    }

    if (motor_lf == NULL || motor_rf == NULL || motor_lb == NULL || motor_rb == NULL)
    {
        for (i = 0; i < 4; i++)
        {
            chassis_feedback_data.wheel_ref[i] = 0.0f;
            chassis_feedback_data.wheel_pid_output[i] = 0.0f;
            chassis_feedback_data.wheel_post_limit_current[i] = 0.0f;
            chassis_feedback_data.wheel_real_current[i] = 0.0f;
            chassis_feedback_data.wheel_speed_aps[i] = 0.0f;
            chassis_feedback_data.wheel_online_flag[i] = 0u;
        }
        return;
    }

    chassis_feedback_data.wheel_ref[LF] = vt_lf;
    chassis_feedback_data.wheel_ref[RF] = vt_rf;
    chassis_feedback_data.wheel_ref[RB] = vt_rb;
    chassis_feedback_data.wheel_ref[LB] = vt_lb;


    chassis_feedback_data.wheel_pid_output[LF] = motor_lf->motor_controller.speed_PID.Output;
    chassis_feedback_data.wheel_pid_output[RF] = motor_rf->motor_controller.speed_PID.Output;
    chassis_feedback_data.wheel_pid_output[RB] = motor_rb->motor_controller.speed_PID.Output;
    chassis_feedback_data.wheel_pid_output[LB] = motor_lb->motor_controller.speed_PID.Output;

    chassis_feedback_data.wheel_post_limit_current[LF] = motorset[LF];
    chassis_feedback_data.wheel_post_limit_current[RF] = motorset[RF];
    chassis_feedback_data.wheel_post_limit_current[RB] = motorset[RB];
    chassis_feedback_data.wheel_post_limit_current[LB] = motorset[LB];

    chassis_feedback_data.wheel_real_current[LF] = motor_lf->measure.real_current;
    chassis_feedback_data.wheel_real_current[RF] = motor_rf->measure.real_current;
    chassis_feedback_data.wheel_real_current[RB] = motor_rb->measure.real_current;
    chassis_feedback_data.wheel_real_current[LB] = motor_lb->measure.real_current;

    chassis_feedback_data.wheel_speed_aps[LF] = motor_lf->measure.speed_aps;
    chassis_feedback_data.wheel_speed_aps[RF] = motor_rf->measure.speed_aps;
    chassis_feedback_data.wheel_speed_aps[RB] = motor_rb->measure.speed_aps;
    chassis_feedback_data.wheel_speed_aps[LB] = motor_lb->measure.speed_aps;

    chassis_feedback_data.wheel_online_flag[LF] = DaemonIsOnline(motor_lf->daemon);
    chassis_feedback_data.wheel_online_flag[RF] = DaemonIsOnline(motor_rf->daemon);
    chassis_feedback_data.wheel_online_flag[RB] = DaemonIsOnline(motor_rb->daemon);
    chassis_feedback_data.wheel_online_flag[LB] = DaemonIsOnline(motor_lb->daemon);
}
/*--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------------- 机器人底盘控制核心任务 -----------------------------------------------------------------------*/
void ChassisTask()
{
    // 后续增加没收到消息的处理(双板的情况)
    // 获取新的控制信息
    RecoverCan1AfterSuperCapOffline();
#if defined(CHASSIS_BOARD) || defined(ONE_BOARD)
    SubGetMessage(chassis_sub, &chassis_cmd_recv);
    /*-----------------------------------------------------------------------------------------------------------------------------------------------*/
    // chassis_cmd_recv_half_float = *(Chassis_Ctrl_Cmd_s_half_float *)CANCommGet(chasiss_can_comm);
    // chassis_cmd_recv.power_limit=1000;
    // chassis_cmd_recv.chassis_power=1000;
    // chassis_cmd_recv.power_buffer=1000;
    // chassis_cmd_recv.vx=chassis_rs485_recv.vx;//half_to_float(chassis_cmd_recv_half_float.vx);
    // chassis_cmd_recv.vy=chassis_rs485_recv.vy;//half_to_float(chassis_cmd_recv_half_float.vy);
    // chassis_cmd_recv.wz=chassis_rs485_recv.wz;//half_to_float(chassis_cmd_recv_half_float.wz);
    // chassis_cmd_recv.gimbal_error_angle=chassis_rs485_recv.gimbal_error_angle;
    // chassis_cmd_recv.offset_angle=chassis_rs485_recv.offset_angle;
    //chassis_cmd_recv.offset_angle=half_to_float(chassis_cmd_recv_half_float.offset_angle);
    //chassis_cmd_recv.SuperCap_flag_from_user=chassis_cmd_recv_half_float.SuperCap_flag_from_user;
    // chassis_cmd_recv.chassis_mode=chassis_rs485_recv.chassis_mode;//chassis_cmd_recv_half_float.chassis_mode;
    /*-----------------------------------------------------------------------------------------------------------------------------------------------*/
    // 裁判系统底盘电源位做去抖，避免单帧抖动导致瞬时无力
    /*------------------------------------------------------------------------------------------------------------------------------------------------*/
    static uint16_t mains_power_off_cnt = 0;
    static uint8_t referee_power_state_valid = 0u;
    if (referee_data_for_ui != NULL && referee_data_for_ui->GameRobotState.robot_id != 0u) {
        referee_power_state_valid = 1u;
    }
    if (referee_power_state_valid) {
        if (referee_data_for_ui->GameRobotState.mains_power_chassis_output == 0) {
            if (mains_power_off_cnt < 1000)
                mains_power_off_cnt++;
        } else {
            mains_power_off_cnt = 0;
        }
        if (mains_power_off_cnt > 30)
            chassis_cmd_recv.chassis_mode = CHASSIS_ZERO_FORCE;
    } else {
        mains_power_off_cnt = 0;
    }
    if (chassis_cmd_recv.chassis_mode == CHASSIS_ZERO_FORCE){ // 如果出现重要模块离线或遥控器设置为急停,让电机停止
    
        ChassisForceReset();
        DJIMotorStop(motor_lf);
        DJIMotorStop(motor_rf);
        DJIMotorStop(motor_lb);
        DJIMotorStop(motor_rb);
        DJIMotorStop(sync_belt_motor_l);
        DJIMotorStop(sync_belt_motor_r);
        DMMotorStop(joint_l);
        DMMotorStop(joint_r);
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_RESET);
        // HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_SET);
        motor_lf->motor_controller.speed_PID.MaxOut = 0;
        motor_rf->motor_controller.speed_PID.MaxOut = 0;
        motor_lb->motor_controller.speed_PID.MaxOut = 0;
        motor_rb->motor_controller.speed_PID.MaxOut = 0;
        sync_belt_motor_l->motor_controller.speed_PID.MaxOut = 0;
        sync_belt_motor_r->motor_controller.speed_PID.MaxOut = 0;

    } else { // 正常工作
        DJIMotorEnable(motor_lf);
        DJIMotorEnable(motor_rf);
        DJIMotorEnable(motor_lb);
        DJIMotorEnable(motor_rb);
        DMMotorEnable1(joint_l);
        DMMotorEnable1(joint_r);
        motor_lf->motor_controller.speed_PID.MaxOut = 16000;
        motor_rf->motor_controller.speed_PID.MaxOut = 16000;
        motor_lb->motor_controller.speed_PID.MaxOut = 16000;
        motor_rb->motor_controller.speed_PID.MaxOut = 16000;
        sync_belt_motor_l->motor_controller.speed_PID.MaxOut = 16000;
        sync_belt_motor_r->motor_controller.speed_PID.MaxOut = 16000;
    }
/*-----------------------------------------------------------爬坡模式处理-------------------------------------------------------------------------------------*/
    static float offset_angle;
    static float sin_theta, cos_theta;
    static float current_speed_vw, vw_set;
    static ramp_t rotate_ramp;
    static float dipAngle = 0;
    static float dipAngleTarget = 0;
    static float length_target_state = 0.0f;
    static uint8_t length_target_state_inited = 0u;
    static float follow_pitch_err_state = 0.0f;
    static uint8_t follow_pitch_err_inited = 0u;
    static float retract_length_target_state = 0;
    static float sync_belt_ref_state = 0;
    static uint8_t retract_target_initialized = 0;
    static uint8_t follow_joint_zero_triggered = 0;
    static uint16_t retract_limit_cnt_l = 0u;
    static uint16_t retract_limit_cnt_r = 0u;
    static uint8_t retract_limit_latched = 0u;
    static uint16_t auto_retract_cnt = 0u;
    static uint16_t auto_retract_cooldown_cnt = 0u;
    static uint16_t auto_retract_detect_cnt = 0u;
    static uint8_t auto_retract_armed = 1u;
    static uint16_t edge_hit_cnt = 0u;
    static uint16_t edge_hit_preload_cnt = 0u;
    static uint16_t edge_hit_retract_cnt = 0u;
    static uint8_t edge_hit_armed = 1u;
    static uint16_t manual_preload_cnt = 0u;
    static uint8_t manual_retract_last = 0u;
    static uint16_t retract_entry_boost_cnt = 0u;
    static uint8_t manual_extend_last = 0u;
    static uint8_t fly_slope_last = 0u;
    static uint8_t fly_slope_phase = 0u;
    static uint8_t fly_slope_slope_seen = 0u;
    static uint16_t fly_slope_phase_cnt = 0u;
    static float fly_slope_length_target_state = 0.0f;
    static uint8_t fly_slope_length_target_inited = 0u;
    static leg_mode_e leg_mode_last = LEG_ACTIVE_SUSPENSION;
    uint8_t manual_retract_request = 0u;
    uint8_t manual_preload_active = 0u;
    uint8_t auto_retract_active = 0u;
    uint8_t edge_hit_preload_active = 0u;
    uint8_t edge_hit_retract_active = 0u;
    uint8_t retract_entry_boost_active = 0u;
    uint8_t climb_sequence_mode = 0u;
    uint8_t manual_extend_active = (chassis_cmd_recv.leg_length_cmd > 0.5f) ? 1u : 0u;
    uint8_t fly_slope_active = (chassis_cmd_recv.chassis_mode == CHASSIS_FLY_SLOPE) ? 1u : 0u;
    uint8_t fly_slope_backward_active =
        (fly_slope_active &&
         (chassis_cmd_recv.vx * LEG_FLY_SLOPE_BACKWARD_VX_SIGN > LEG_FLY_SLOPE_BACKWARD_VX_THRESHOLD)) ? 1u : 0u;
    float fly_slope_dip_target = LEG_FLY_SLOPE_CONTACT_DIP_TARGET;
    float fly_slope_leg_p = LEG_FLY_SLOPE_CONTACT_LEG_P;
    float fly_slope_length_add = LEG_FLY_SLOPE_CONTACT_LENGTH_ADD;
    float fly_slope_length_abs_target = LEG_FLY_SLOPE_CONTACT_LENGTH_TARGET;
    float fly_slope_joint_assist = LEG_FLY_SLOPE_CONTACT_JOINT_ASSIST;
    float chassis_follow_kp_target = 105.0f;

    // offset_angle       = chassis_cmd_recv.offset_angle + chassis_cmd_recv.gimbal_error_angle;
    // offset_angle_watch = offset_angle;

    if(chassis_cmd_recv.chassis_mode == CHASSIS_CLIMB || chassis_cmd_recv.chassis_mode == CHASSIS_CLIMB_RETRACT)
        // offset_angle =(chassis_cmd_recv.offset_angle >= 0 ? chassis_cmd_recv.offset_angle - 180 : chassis_cmd_recv.offset_angle + 180);
        offset_angle = 0;
    else
        // offset_angle =chassis_cmd_recv.offset_angle;
        offset_angle = 0;
    offset_angle =chassis_cmd_recv.offset_angle;
    // chassis_cmd_recv.offset_angle = 0;
/*---------------------------------------------------------------------底盘模式切换核心----------------------------------------------------------------------------*/
    // 根据控制模式设定旋转速度
    if (chassis_cmd_recv.chassis_mode != CHASSIS_FOLLOW_GIMBAL_YAW)
        follow_joint_zero_triggered = 0;

    switch (chassis_cmd_recv.chassis_mode) {
        case CHASSIS_NO_FOLLOW:
            chassis_cmd_recv.wz = 0.0f;
            powerLim = 0;
            cos_theta = 1.0f;
            sin_theta = 0.0f;
            leg_mode  = LEG_ACTIVE_SUSPENSION;
            ramp_init(&rotate_ramp, 250);
            break;
        case CHASSIS_FOLLOW_GIMBAL_YAW:
            powerLim = 0;
            if (!follow_joint_zero_triggered) {          
                l_offset = 0.0f;
                r_offset = 0.0f;
                follow_joint_zero_triggered = 1;
            }
            chassis_cmd_recv.wz = PIDCalculate(&Chassis_Follow_PID, offset_angle, 0);
            cos_theta = arm_cos_f32(-chassis_cmd_recv.offset_angle * DEGREE_2_RAD);
            sin_theta = arm_sin_f32(-chassis_cmd_recv.offset_angle * DEGREE_2_RAD);
            leg_mode  = LEG_ACTIVE_SUSPENSION;
            ramp_init(&rotate_ramp, 250);
            break;
        case CHASSIS_ROTATE: // 自旋,同时保持全向机动;当前wz维持定值,后续增加不规则的变速策略
        
        // vw_set = chassis_cmd_recv.vw_set;
        // vxy_k=chassis_cmd_recv.wz_K;
            // if (cap->cap_msg_s.SuperCap_open_flag_from_real == SUPERCAP_PMOS_OPEN) {
            
                powerLim=chassis_cmd_recv.power_limit;
                // chassis_cmd_recv.power_limit=110;
                if (chassis_cmd_recv.SuperCap_flag_from_user == SUPERCAP_USE) {
                    vw_set = 5000;
                } else {
                    vw_set = 3000;
                }
                vxy_k =1;
                // vxy_k=textkd;
                // 26.6f*chassis_cmd_recv.power_limit+1537;
            chassis_vw       = (current_speed_vw + (vw_set - current_speed_vw) * ramp_calc(&rotate_ramp));
            current_speed_vw = chassis_vw;

            chassis_cmd_recv.wz = chassis_vw;
            cos_theta           = arm_cos_f32((-chassis_cmd_recv.offset_angle /*+ 22*/) * DEGREE_2_RAD); // 矫正小陀螺偏心
            sin_theta           = arm_sin_f32((-chassis_cmd_recv.offset_angle /*+ 22*/) * DEGREE_2_RAD);
            leg_mode            = LEG_ACTIVE_SUSPENSION;
            break;
        //进入小陀螺模式；如果允许用超级电容，就把目标自旋速度设成 5000，否则 3000；再通过 ramp_calc() 平滑地把当前 wz 拉到目标值
        case CHASSIS_REVERSE_ROTATE:
            chassis_cmd_recv.wz = -5000;
            cos_theta           = arm_cos_f32((-chassis_cmd_recv.offset_angle /*+ 22*/) * DEGREE_2_RAD); // 矫正小陀螺偏心
            sin_theta           = arm_sin_f32((-chassis_cmd_recv.offset_angle /*+ 22*/) * DEGREE_2_RAD);
            leg_mode            = LEG_ACTIVE_SUSPENSION;
            break;
        //反向小陀螺，直接给 wz=-5000
        case CHASSIS_CLIMB:
        case CHASSIS_CLIMB_WITH_PULL:
        case CHASSIS_CLIMB_WITH_PUSH:
            chassis_cmd_recv.wz = PIDCalculate(&Chassis_Follow_PID, offset_angle, 0);
            // chassis_cmd_recv.wz = 0;
            cos_theta           = arm_cos_f32(-chassis_cmd_recv.offset_angle * DEGREE_2_RAD);
            sin_theta           = arm_sin_f32(-chassis_cmd_recv.offset_angle * DEGREE_2_RAD);
            leg_mode            = LEG_CLIMB;
            ramp_init(&rotate_ramp, 250);
            switch(chassis_cmd_recv.chassis_mode)
            {
                case CHASSIS_CLIMB:
                    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_RESET);
                    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_RESET);
                    // HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_SET);
                    break;
                case CHASSIS_CLIMB_WITH_PULL:
                    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_SET);
                    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_SET);
                    // HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_RESET);
                    break;
                case CHASSIS_CLIMB_WITH_PUSH:
                    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_RESET);
                    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_SET);
                    // HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_SET);
                    break;
                default:
                    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_RESET);
                    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_RESET);
                    // HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_SET);
                    break;
            }
            break;
        //仍然用跟随 PID 保持航向，但腿模式切到 LEG_CLIMB，并通过 PE9/PE11 的不同高低电平组合控制外部机构是普通爬坡、拉、还是推。
        case CHASSIS_FLY_SLOPE:
            chassis_cmd_recv.wz = PIDCalculate(&Chassis_Follow_PID, offset_angle, 0);
            cos_theta           = 1.0f;
            sin_theta           = 0.0f;
            leg_mode            = LEG_CLIMB;
            ramp_init(&rotate_ramp, 250);
            HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_RESET);
            break;
        case CHASSIS_CLIMB_RETRACT:
            chassis_cmd_recv.wz = PIDCalculate(&Chassis_Follow_PID, offset_angle, 0);
            // chassis_cmd_recv.wz = 0;
            cos_theta           = arm_cos_f32(-chassis_cmd_recv.offset_angle * DEGREE_2_RAD);
            sin_theta           = arm_sin_f32(-chassis_cmd_recv.offset_angle * DEGREE_2_RAD);
            leg_mode            = LEG_CLIMB_RETRACT;
            ramp_init(&rotate_ramp, 250);
            break;
        default:
            break;
        //航向同样跟随，腿模式换成 LEG_CLIMB_RETRACT，用于回收/收腿。
    }

    climb_sequence_mode = ChassisModeIsClimbSequence(chassis_cmd_recv.chassis_mode);
    if (fly_slope_active) {
        float pitch_now = Chassis_IMU_data->output.INS_angle[INS_PITCH_ADDRESS_OFFSET];

        if (!fly_slope_last) {
            fly_slope_phase = 0u;
            fly_slope_phase_cnt = 0u;
            fly_slope_slope_seen = 0u;
            fly_slope_length_target_inited = 0u;
        } else if (fly_slope_phase_cnt < 65535u) {
            fly_slope_phase_cnt++;
        }

        if (pitch_now > LEG_FLY_SLOPE_SLOPE_SEEN_PITCH)
            fly_slope_slope_seen = 1u;

        if (fly_slope_phase == 0u &&
            ((fly_slope_slope_seen && pitch_now < LEG_FLY_SLOPE_TAKEOFF_PITCH) ||
             fly_slope_phase_cnt > LEG_FLY_SLOPE_PRE_EXTEND_DELAY_COUNT)) {
            fly_slope_phase = 1u;
            fly_slope_phase_cnt = 0u;
        }

        if (fly_slope_phase == 0u) {
            fly_slope_dip_target = LEG_FLY_SLOPE_CONTACT_DIP_TARGET;
            fly_slope_leg_p = LEG_FLY_SLOPE_CONTACT_LEG_P;
            fly_slope_length_add = LEG_FLY_SLOPE_CONTACT_LENGTH_ADD;
            fly_slope_length_abs_target = LEG_FLY_SLOPE_CONTACT_LENGTH_TARGET;
            fly_slope_joint_assist = LEG_FLY_SLOPE_CONTACT_JOINT_ASSIST;
        } else {
            fly_slope_dip_target = LEG_FLY_SLOPE_PRE_EXTEND_DIP_TARGET;
            fly_slope_leg_p = LEG_FLY_SLOPE_PRE_EXTEND_LEG_P;
            fly_slope_length_add = LEG_FLY_SLOPE_PRE_EXTEND_LENGTH_ADD;
            fly_slope_length_abs_target = LEG_FLY_SLOPE_PRE_EXTEND_LENGTH_TARGET;
            fly_slope_joint_assist = LEG_FLY_SLOPE_PRE_EXTEND_JOINT_ASSIST;
        }

        if (fly_slope_backward_active) {
            fly_slope_dip_target = LEG_FLY_SLOPE_BACKWARD_DIP_TARGET;
            fly_slope_leg_p = leg_p;
            fly_slope_length_add = 0.0f;
            fly_slope_length_abs_target = LEG_FLY_SLOPE_LENGTH_MIN;
            fly_slope_joint_assist = 0.0f;
        }
    } else {
        fly_slope_phase = 0u;
        fly_slope_phase_cnt = 0u;
        fly_slope_slope_seen = 0u;
        fly_slope_length_target_inited = 0u;
    }
    fly_slope_last = fly_slope_active;

    if (chassis_cmd_recv.chassis_mode == CHASSIS_CLIMB_RETRACT) {
        auto_retract_cnt = 0u;
        auto_retract_cooldown_cnt = 0u;
        auto_retract_detect_cnt = 0u;
        auto_retract_armed = 0u;
        edge_hit_cnt = 0u;
        edge_hit_preload_cnt = 0u;
        edge_hit_retract_cnt = 0u;
        edge_hit_armed = 0u;
    }

    if (manual_extend_active || fly_slope_active) {
        auto_retract_cnt = 0u;
        auto_retract_cooldown_cnt = 0u;
        auto_retract_detect_cnt = 0u;
        auto_retract_armed = 0u;
        edge_hit_cnt = 0u;
        edge_hit_preload_cnt = 0u;
        edge_hit_retract_cnt = 0u;
        edge_hit_armed = 0u;
    } else if (manual_extend_last && chassis_cmd_recv.chassis_mode == CHASSIS_CLIMB) {
        auto_retract_armed = 1u;
        edge_hit_armed = 1u;
    }
    manual_extend_last = manual_extend_active;

    if (chassis_cmd_recv.chassis_mode == CHASSIS_CLIMB && !manual_extend_active && !fly_slope_active) {
        if (auto_retract_cooldown_cnt > 0u)
            auto_retract_cooldown_cnt--;

        if (auto_retract_cnt == 0u &&
            auto_retract_armed &&
            auto_retract_cooldown_cnt == 0u) {
            if (LegAutoRetractTrigger(LEG_CLIMB_DIP_TARGET)) {
                if (auto_retract_detect_cnt < LEG_AUTO_RETRACT_DETECT_COUNT)
                    auto_retract_detect_cnt++;
            } else {
                auto_retract_detect_cnt = 0u;
            }

            if (auto_retract_detect_cnt >= LEG_AUTO_RETRACT_DETECT_COUNT) {
                auto_retract_cnt = LEG_AUTO_RETRACT_DURATION_COUNT;
                auto_retract_armed = 0u;
                edge_hit_armed = 0u;
                edge_hit_cnt = 0u;
                auto_retract_detect_cnt = 0u;
            }
        } else {
            auto_retract_detect_cnt = 0u;
        }

        if (auto_retract_cnt > 0u) {
            auto_retract_cnt--;
            auto_retract_active = 1u;
            leg_mode = LEG_CLIMB_RETRACT;
            if (auto_retract_cnt == 0u)
                auto_retract_cooldown_cnt = LEG_AUTO_RETRACT_COOLDOWN_COUNT;
        }
    } else if (!climb_sequence_mode) {
        auto_retract_cnt = 0u;
        auto_retract_cooldown_cnt = 0u;
        auto_retract_detect_cnt = 0u;
        auto_retract_armed = 1u;
    }

    if (chassis_cmd_recv.chassis_mode == CHASSIS_CLIMB && !manual_extend_active && !fly_slope_active) {
        if (edge_hit_preload_cnt == 0u &&
            edge_hit_retract_cnt == 0u &&
            edge_hit_armed) {
            if (LegEdgeHitDetected(LEG_CLIMB_DIP_TARGET)) {
                if (edge_hit_cnt < LEG_EDGE_HIT_DETECT_COUNT)
                    edge_hit_cnt++;
            } else {
                edge_hit_cnt = 0u;
            }

            if (edge_hit_cnt >= LEG_EDGE_HIT_DETECT_COUNT) {
                edge_hit_preload_cnt = LEG_EDGE_HIT_PRELOAD_COUNT;
                edge_hit_retract_cnt = LEG_EDGE_HIT_RETRACT_COUNT;
                edge_hit_armed = 0u;
                edge_hit_cnt = 0u;
                auto_retract_cnt = 0u;
                auto_retract_active = 0u;
                auto_retract_armed = 0u;
                auto_retract_detect_cnt = 0u;
            }
        }

        if (edge_hit_preload_cnt > 0u) {
            edge_hit_preload_active = 1u;
            edge_hit_preload_cnt--;
            leg_mode = LEG_CLIMB_RETRACT;
        } else if (edge_hit_retract_cnt > 0u) {
            edge_hit_retract_active = 1u;
            edge_hit_retract_cnt--;
            leg_mode = LEG_CLIMB_RETRACT;
        }
    } else if (!climb_sequence_mode) {
        edge_hit_cnt = 0u;
        edge_hit_preload_cnt = 0u;
        edge_hit_retract_cnt = 0u;
        edge_hit_armed = 1u;
    }

    if (fly_slope_active && leg_mode == LEG_CLIMB_RETRACT) {
        leg_mode = LEG_CLIMB;
        retract_entry_boost_cnt = 0u;
    }

    if (leg_mode == LEG_CLIMB_RETRACT) {
        if (leg_mode_last != LEG_CLIMB_RETRACT)
            retract_entry_boost_cnt = LEG_RETRACT_ENTRY_BOOST_COUNT;

        if (retract_entry_boost_cnt > 0u) {
            retract_entry_boost_active = 1u;
            retract_entry_boost_cnt--;
        }
    } else {
        retract_entry_boost_cnt = 0u;
    }
    leg_mode_last = leg_mode;

    manual_retract_request = (leg_mode == LEG_CLIMB_RETRACT) ? LegRetractManualBoostEnabled() : 0u;
    if (manual_retract_request) {
        if (!manual_retract_last)
            manual_preload_cnt = LEG_MANUAL_PRELOAD_COUNT;
    } else {
        manual_preload_cnt = 0u;
    }
    manual_retract_last = manual_retract_request;

    if (manual_preload_cnt > 0u) {
        manual_preload_active = 1u;
        manual_preload_cnt--;
    }
    // {
    //     float sync_belt_left_ref  = 0.0f;
    //     float sync_belt_right_ref = 0.0f;

    //     ChassisGetSyncBeltRef(&chassis_cmd_recv, &sync_belt_left_ref, &sync_belt_right_ref);
    //     DJIMotorSetRef(sync_belt_motor_l, sync_belt_left_ref);
    //     DJIMotorSetRef(sync_belt_motor_r, sync_belt_right_ref);

    //     if (ChassisSyncBeltModeEnabled(&chassis_cmd_recv)) {
    //         DJIMotorEnable(sync_belt_motor_l);
    //         DJIMotorEnable(sync_belt_motor_r);
    //     } else {
    //         DJIMotorStop(sync_belt_motor_l);
    //         DJIMotorStop(sync_belt_motor_r);
    //     }
    // }
        //决定腿的姿态控制策略
    switch(leg_mode)
    {
        case LEG_ACTIVE_SUSPENSION:
            dipAngleTarget = 0.06f;
            joint_l->motor_settings.feedforward_flag = CURRENT_FEEDFORWARD;
            joint_r->motor_settings.feedforward_flag = CURRENT_FEEDFORWARD;
            joint_l->ctrl.kp_set = LEG_ACTIVE_POS_KP;
            joint_l->ctrl.kd_set = LEG_ACTIVE_POS_KD;
            joint_r->ctrl.kp_set = LEG_ACTIVE_POS_KP;
            joint_r->ctrl.kd_set = LEG_ACTIVE_POS_KD;
            // joint_l->ctrl.kp_set = 150;
            // joint_l->ctrl.kd_set = 2;
            // joint_l->ctrl.tor_set = 7;
            // joint_r->ctrl.kp_set = 150;
            // joint_r->ctrl.kd_set = 2;
            // joint_r->ctrl.tor_set = -7;
            chassis_follow_kp_target = 105.0f;
            break;
            //dipAngle=0，保留关节力矩前馈，跟随 PID 的 Kp 保持 105。
        case LEG_CLIMB:
            dipAngleTarget = LEG_CLIMB_DIP_TARGET;
            joint_l->motor_settings.feedforward_flag = CURRENT_FEEDFORWARD;
            joint_r->motor_settings.feedforward_flag = CURRENT_FEEDFORWARD;
            joint_l->ctrl.kp_set = 0.0f;
            joint_l->ctrl.kd_set = 0.0f;
            joint_r->ctrl.kp_set = 0.0f;
            joint_r->ctrl.kd_set = 0.0f;
            // joint_l->ctrl.kp_set = 150;
            // joint_l->ctrl.kd_set = 2;
            // joint_l->ctrl.tor_set = 7;
            // joint_r->ctrl.kp_set = 150;
            // joint_r->ctrl.kd_set = 2;
            // joint_r->ctrl.tor_set = -7;
            chassis_follow_kp_target = 105.0f;
            break;
            //意思是让机身带一点前倾/下沉目标姿态
        case LEG_CLIMB_RETRACT:
            dipAngleTarget = LEG_RETRACT_DIP_TARGET;
            joint_l->motor_settings.feedforward_flag = CURRENT_FEEDFORWARD;
            joint_r->motor_settings.feedforward_flag = CURRENT_FEEDFORWARD;
            joint_l->ctrl.kp_set = LEG_RETRACT_POS_KP;
            joint_l->ctrl.kd_set = LEG_RETRACT_POS_KD;
            joint_r->ctrl.kp_set = LEG_RETRACT_POS_KP;
            joint_r->ctrl.kd_set = LEG_RETRACT_POS_KD;
            // joint_l->ctrl.kp_set = 50;
            // joint_l->ctrl.kd_set = 4;
            // joint_l->ctrl.tor_set = -6;
            // joint_r->ctrl.kp_set = 50;
            // joint_r->ctrl.kd_set = 4;
            // joint_r->ctrl.tor_set = 6;
            chassis_follow_kp_target = 50.0f;
            break;
            //dipAngle=-0.1，关闭关节电流前馈，并把跟随 PID 的 Kp 降到 50，让动作更软一点。
        default:
            dipAngleTarget = 0.0f;
            joint_l->motor_settings.feedforward_flag = FEEDFORWARD_NONE;
            joint_r->motor_settings.feedforward_flag = FEEDFORWARD_NONE;
            joint_l->ctrl.kp_set = 0.0f;
            joint_l->ctrl.kd_set = 0.0f;
            joint_r->ctrl.kp_set = 0.0f;
            joint_r->ctrl.kd_set = 0.0f;
            chassis_follow_kp_target = 105.0f;
            break;
    }
    if (manual_preload_active || edge_hit_preload_active) {
        dipAngleTarget = LEG_MANUAL_PRELOAD_DIP_TARGET;
        chassis_follow_kp_target = 105.0f;
    }
    if ((leg_mode == LEG_ACTIVE_SUSPENSION || leg_mode == LEG_CLIMB) && manual_extend_active) {
        dipAngleTarget = LEG_MANUAL_EXTEND_DIP_TARGET;
        chassis_follow_kp_target = 105.0f;
    }
    if (fly_slope_active) {
        dipAngleTarget = fly_slope_dip_target;
        joint_l->ctrl.kp_set = LEG_FLY_SLOPE_POS_KP;
        joint_l->ctrl.kd_set = LEG_FLY_SLOPE_POS_KD;
        joint_r->ctrl.kp_set = LEG_FLY_SLOPE_POS_KP;
        joint_r->ctrl.kd_set = LEG_FLY_SLOPE_POS_KD;
        chassis_follow_kp_target = 105.0f;
    }
    dipAngle += clamp_absf(dipAngleTarget - dipAngle, LEG_DIP_SLEW_STEP);
    Chassis_Follow_PID.Kp += clamp_absf(chassis_follow_kp_target - Chassis_Follow_PID.Kp, LEG_FOLLOW_KP_SLEW_STEP);
/*--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
    //把左右关节编码器角度统一到同一物理方向，并扣掉安装零偏。
    angle_l = joint_l->measure.pos - l_offset;
    angle_r = -joint_r->measure.pos + r_offset;
    angle_avg = (angle_l + angle_r) / 2;//左右关节平均角
/*--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
    // tor_avg = (joint_l->measure.tor - joint_r->measure.tor) / 2;
    // angle_measure_all[avg_count] = (angle_l + angle_r) / 2;
    // tor_measure_all[avg_count] = (joint_l->measure.tor - joint_r->measure.tor) / 2;
    // avg_count++;
    // if(avg_count >= 10)
    // {
    //     angle_measure_sum = 0;
    //     // tor_measure_sum = 0;
    //     for(avg_i = 0; avg_i < avg_count; avg_i++)
    //     {
    //         angle_measure_sum += angle_measure_all[avg_i];
    //         // tor_measure_sum += tor_measure_all[avg_i];
    //     }
    //     angle_avg = angle_measure_sum / avg_count;
    //     // tor_avg = tor_measure_sum / avg_count;
    //     avg_count = 0;
    // }
/*--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
    
    length_l_measure = -0.05814 * angle_l * angle_l + 0.2072 *angle_l + 0.107;//用二次多项式把关节角度换算成腿长
    length_r_measure = -0.05814 * angle_r * angle_r + 0.2072 *angle_r + 0.107;

    length_measure = (length_l_measure + length_r_measure)/2;                //左右腿平均长度
    {
        uint8_t manual_extend_leg_active = ((leg_mode == LEG_ACTIVE_SUSPENSION || leg_mode == LEG_CLIMB) && manual_extend_active) ? 1u : 0u;
        uint8_t fly_slope_leg_active = (fly_slope_active && leg_mode == LEG_CLIMB) ? 1u : 0u;
        uint8_t follow_suspension_leg_active =
            (chassis_cmd_recv.chassis_mode == CHASSIS_FOLLOW_GIMBAL_YAW &&
             leg_mode == LEG_ACTIVE_SUSPENSION &&
             !manual_extend_active) ? 1u : 0u;
        float leg_p_used = fly_slope_leg_active ? fly_slope_leg_p :
                           (manual_extend_leg_active ? LEG_MANUAL_EXTEND_LEG_P : leg_p);
        float pitch_err = Chassis_IMU_data->output.INS_angle[1] - dipAngle;
        float length_target_raw;

        if (follow_suspension_leg_active) {
            if (!follow_pitch_err_inited) {
                follow_pitch_err_state = 0.0f;
                follow_pitch_err_inited = 1u;
            }
            follow_pitch_err_state += LEG_FOLLOW_PITCH_ERR_FILTER_ALPHA * (pitch_err - follow_pitch_err_state);

            if (follow_pitch_err_state > LEG_FOLLOW_PITCH_ERR_DEADBAND)
                pitch_err = follow_pitch_err_state - LEG_FOLLOW_PITCH_ERR_DEADBAND;
            else if (follow_pitch_err_state < -LEG_FOLLOW_PITCH_ERR_DEADBAND)
                pitch_err = follow_pitch_err_state + LEG_FOLLOW_PITCH_ERR_DEADBAND;
            else
                pitch_err = 0.0f;
        } else {
            follow_pitch_err_state = 0.0f;
            follow_pitch_err_inited = 0u;
        }

        length_target_raw = length_measure - leg_p_used * pitch_err;

        if (fly_slope_leg_active) {
            float fly_slope_extend = fly_slope_length_add +
                                     leg_p_used * (dipAngle - Chassis_IMU_data->output.INS_angle[1]);
            float fly_slope_extend_target;
            if (fly_slope_extend < fly_slope_length_add)
                fly_slope_extend = fly_slope_length_add;

            fly_slope_extend_target = length_measure + fly_slope_extend;
            if (fly_slope_extend_target < fly_slope_length_abs_target)
                fly_slope_extend_target = fly_slope_length_abs_target;
            length_target_raw = fly_slope_extend_target;
        }

        if (fly_slope_leg_active) {
            length_target_raw = clamp_rangef(length_target_raw,
                                             LEG_FLY_SLOPE_LENGTH_MIN,
                                             LEG_FLY_SLOPE_LENGTH_MAX);
            if (!fly_slope_length_target_inited) {
                fly_slope_length_target_state = length_measure;
                fly_slope_length_target_inited = 1u;
            }
            if (fly_slope_length_target_state < length_measure)
                fly_slope_length_target_state = length_measure;

            if (fly_slope_backward_active) {
                fly_slope_length_target_state += clamp_absf(length_target_raw - fly_slope_length_target_state,
                                                            LEG_FLY_SLOPE_BACKWARD_LENGTH_SLEW_STEP);
            } else if (length_target_raw > fly_slope_length_target_state) {
                float fly_slope_extend_step = length_target_raw - fly_slope_length_target_state;
                if (fly_slope_extend_step > LEG_FLY_SLOPE_LENGTH_SLEW_STEP)
                    fly_slope_extend_step = LEG_FLY_SLOPE_LENGTH_SLEW_STEP;
                fly_slope_length_target_state += fly_slope_extend_step;
            }
            length_target = fly_slope_length_target_state;
            length_target_state_inited = 0u;
            length_target_state = length_measure;
        } else if (manual_extend_leg_active) {
            if (!length_target_state_inited) {
                length_target_state = length_measure;
                length_target_state_inited = 1u;
            }
            length_target_state += clamp_absf(length_target_raw - length_target_state,
                                             LEG_MANUAL_EXTEND_LENGTH_SLEW_STEP);
            length_target = length_target_state;
        } else if (follow_suspension_leg_active) {
            length_target_raw = clamp_rangef(length_target_raw,
                                             LEG_ACTIVE_LENGTH_TARGET_MIN,
                                             LEG_ACTIVE_FOLLOW_LENGTH_MAX);
            if (!length_target_state_inited) {
                length_target_state = length_measure;
                length_target_state_inited = 1u;
            }
            if (length_target_raw < length_target_state) {
                length_target_state += clamp_absf(length_target_raw - length_target_state,
                                                  LEG_ACTIVE_LENGTH_RETRACT_SLEW_STEP);
            } else {
                length_target_state += clamp_absf(length_target_raw - length_target_state,
                                                  LEG_ACTIVE_LENGTH_EXTEND_SLEW_STEP);
            }
            length_target = length_target_state;
            fly_slope_length_target_inited = 0u;
            fly_slope_length_target_state = length_measure;
        } else {
            length_target_state_inited = 0u;
            length_target_state = length_measure;
            fly_slope_length_target_inited = 0u;
            fly_slope_length_target_state = length_measure;
            length_target = length_target_raw;
        }
    }
    angle_target = (-0.2072 + safe_sqrt(0.2072 * 0.2072 + 4 * 0.05814 * (0.107 - length_target)))/(-2 * 0.05814);//把目标腿长反解回目标关节角；这里用了 safe_sqrt() 防止判别式为负。
    angle_l_target = angle_target + l_offset;//再把统一角转换回左右关节各自的命令角度
    angle_r_target = r_offset - angle_target;
    if (fly_slope_active && fly_slope_dip_target > 0.001f) {
        float fly_slope_assist_ratio = dipAngle / fly_slope_dip_target;
        float fly_slope_assist_angle;

        if (fly_slope_assist_ratio < 0.0f)
            fly_slope_assist_ratio = 0.0f;
        else if (fly_slope_assist_ratio > 1.0f)
            fly_slope_assist_ratio = 1.0f;

        fly_slope_assist_angle = fly_slope_joint_assist * fly_slope_assist_ratio;
        angle_l_target += fly_slope_assist_angle;
        angle_r_target -= fly_slope_assist_angle;
    }
    // if (leg_mode == LEG_CLIMB_RETRACT) {
    //     if (!retract_target_initialized) {
    //         retract_length_target_state = length_measure;
    //         retract_target_initialized = 1;
    //         Leg_Retract_Length_PID.Iout = 0.0f;
    //         Leg_Retract_Length_PID.ITerm = 0.0f;
    //         Leg_Retract_Length_PID.Last_Err = 0.0f;
    //         Leg_Retract_Length_PID.Last_Output = 0.0f;
    //         Leg_Retract_Length_PID.Last_Dout = 0.0f;
    //     }
    //     retract_length_target_state += clamp_absf(LEG_RETRACT_TARGET_LENGTH_MIN - retract_length_target_state,
    //                                               LEG_RETRACT_TARGET_LENGTH_STEP);
    //     if (retract_length_target_state > length_measure)
    //         retract_length_target_state = length_measure;
    // } else {
    //     retract_length_target_state = length_measure;
    //     retract_target_initialized = 0;
    //     Leg_Retract_Length_PID.Iout = 0.0f;
    //     Leg_Retract_Length_PID.ITerm = 0.0f;
    //     Leg_Retract_Length_PID.Last_Err = 0.0f;
    //     Leg_Retract_Length_PID.Last_Output = 0.0f;
    //     Leg_Retract_Length_PID.Last_Dout = 0.0f;
    // }

    length_diff = length_l_measure - length_r_measure;//绠楀乏鍙宠吙闀垮害宸?
    if (leg_mode == LEG_CLIMB_RETRACT) {
        float retract_torque_l = 0.0f;
        float retract_torque_r = 0.0f;
        uint8_t retract_manual_boost = (manual_retract_request && !manual_preload_active) ? 1u : 0u;
        uint8_t retract_boost_for_limit = (retract_manual_boost ||
                                           retract_entry_boost_active ||
                                           edge_hit_retract_active ||
                                           auto_retract_active) ? 1u : 0u;

        if (!retract_limit_latched) {
            if (retract_boost_for_limit &&
                fabsf(joint_l->measure.tor) > LEG_RETRACT_LIMIT_TORQUE_THRESHOLD &&
                fabsf(joint_l->measure.vel) < LEG_RETRACT_LIMIT_VEL_THRESHOLD) {
                if (retract_limit_cnt_l < 65535u)
                    retract_limit_cnt_l++;
            } else {
                retract_limit_cnt_l = 0u;
            }

            if (retract_boost_for_limit &&
                fabsf(joint_r->measure.tor) > LEG_RETRACT_LIMIT_TORQUE_THRESHOLD &&
                fabsf(joint_r->measure.vel) < LEG_RETRACT_LIMIT_VEL_THRESHOLD) {
                if (retract_limit_cnt_r < 65535u)
                    retract_limit_cnt_r++;
            } else {
                retract_limit_cnt_r = 0u;
            }

            if (retract_limit_cnt_l >= LEG_RETRACT_LIMIT_DETECT_COUNT ||
                retract_limit_cnt_r >= LEG_RETRACT_LIMIT_DETECT_COUNT) {
                retract_limit_latched = 1u;
            }
        }

        if (retract_limit_latched) {
            retract_torque_l = CalcRetractTorqueFeedforward(angle_l);
            retract_torque_r = CalcRetractTorqueFeedforward(angle_r);

            if (retract_torque_l < LEG_RETRACT_HOLD_TORQUE_MIN)
                retract_torque_l = LEG_RETRACT_HOLD_TORQUE_MIN;
            if (retract_torque_r < LEG_RETRACT_HOLD_TORQUE_MIN)
                retract_torque_r = LEG_RETRACT_HOLD_TORQUE_MIN;

            if (retract_torque_l > LEG_RETRACT_TORQUE_MAX)
                retract_torque_l = LEG_RETRACT_TORQUE_MAX;
            if (retract_torque_r > LEG_RETRACT_TORQUE_MAX)
                retract_torque_r = LEG_RETRACT_TORQUE_MAX;
        } else if (manual_preload_active || edge_hit_preload_active) {
            retract_torque_l = -LEG_MANUAL_PRELOAD_TORQUE;
            retract_torque_r = -LEG_MANUAL_PRELOAD_TORQUE;
        } else if (retract_manual_boost) {
            retract_torque_l = CalcRetractTorqueFeedforward(angle_l) + LEG_RETRACT_MANUAL_BOOST_TORQUE;
            retract_torque_r = CalcRetractTorqueFeedforward(angle_r) + LEG_RETRACT_MANUAL_BOOST_TORQUE;

            if (retract_torque_l > LEG_RETRACT_MANUAL_TORQUE_LIMIT)
                retract_torque_l = LEG_RETRACT_MANUAL_TORQUE_LIMIT;
            if (retract_torque_r > LEG_RETRACT_MANUAL_TORQUE_LIMIT)
                retract_torque_r = LEG_RETRACT_MANUAL_TORQUE_LIMIT;
        } else if (edge_hit_retract_active) {
            retract_torque_l = CalcRetractTorqueFeedforward(angle_l) + LEG_EDGE_HIT_BOOST_TORQUE;
            retract_torque_r = CalcRetractTorqueFeedforward(angle_r) + LEG_EDGE_HIT_BOOST_TORQUE;

            if (retract_torque_l > LEG_EDGE_HIT_TORQUE_LIMIT)
                retract_torque_l = LEG_EDGE_HIT_TORQUE_LIMIT;
            if (retract_torque_r > LEG_EDGE_HIT_TORQUE_LIMIT)
                retract_torque_r = LEG_EDGE_HIT_TORQUE_LIMIT;
        } else if (auto_retract_active) {
            retract_torque_l = CalcRetractTorqueFeedforward(angle_l) + LEG_AUTO_RETRACT_BOOST_TORQUE;
            retract_torque_r = CalcRetractTorqueFeedforward(angle_r) + LEG_AUTO_RETRACT_BOOST_TORQUE;

            if (retract_torque_l > LEG_AUTO_RETRACT_TORQUE_LIMIT)
                retract_torque_l = LEG_AUTO_RETRACT_TORQUE_LIMIT;
            if (retract_torque_r > LEG_AUTO_RETRACT_TORQUE_LIMIT)
                retract_torque_r = LEG_AUTO_RETRACT_TORQUE_LIMIT;
        } else if (retract_entry_boost_active) {
            retract_torque_l = CalcRetractTorqueFeedforward(angle_l) + LEG_RETRACT_ENTRY_BOOST_TORQUE;
            retract_torque_r = CalcRetractTorqueFeedforward(angle_r) + LEG_RETRACT_ENTRY_BOOST_TORQUE;

            if (retract_torque_l > LEG_RETRACT_ENTRY_TORQUE_LIMIT)
                retract_torque_l = LEG_RETRACT_ENTRY_TORQUE_LIMIT;
            if (retract_torque_r > LEG_RETRACT_ENTRY_TORQUE_LIMIT)
                retract_torque_r = LEG_RETRACT_ENTRY_TORQUE_LIMIT;
        }

        length_diff_tor = 0.0f;
        joint_l_tor_feedforward = retract_torque_l;
        joint_r_tor_feedforward = retract_torque_r;
    } else if (fly_slope_active) {
        length_diff_tor = 0.0f;
        joint_l_tor_feedforward = 0.0f;
        joint_r_tor_feedforward = 0.0f;
    } else {
        retract_limit_cnt_l = 0u;
        retract_limit_cnt_r = 0u;
        retract_limit_latched = 0u;
        length_diff_tor = 0.0f;
        if (chassis_cmd_recv.chassis_mode == CHASSIS_FOLLOW_GIMBAL_YAW &&
            leg_mode == LEG_ACTIVE_SUSPENSION &&
            !manual_extend_active &&
            length_measure > LEG_ACTIVE_LENGTH_PROTECT_START) {
            float protect_torque = (length_measure - LEG_ACTIVE_LENGTH_PROTECT_START) *
                                   LEG_ACTIVE_LENGTH_PROTECT_KP;

            protect_torque = clamp_rangef(protect_torque,
                                          0.0f,
                                          LEG_ACTIVE_LENGTH_PROTECT_MAX_TORQUE);
            protect_torque *= LEG_ACTIVE_LENGTH_PROTECT_TORQUE_SIGN;

            joint_l_tor_feedforward = protect_torque;
            joint_r_tor_feedforward = protect_torque;
        } else {
            joint_l_tor_feedforward = 0.0f;
            joint_r_tor_feedforward = 0.0f;
        }
    }
    
    joint_l->ctrl.vel_set = 0.0f;
    joint_r->ctrl.vel_set = 0.0f;
    if(leg_mode == LEG_CLIMB_RETRACT) {
        if (retract_limit_latched) {
            joint_l->ctrl.kp_set = 0.0f;
            joint_l->ctrl.kd_set = 0.0f;
            joint_r->ctrl.kp_set = 0.0f;
            joint_r->ctrl.kd_set = 0.0f;
            joint_limit(joint_l->measure.pos, joint_r->measure.pos);
        } else {
            joint_limit(l_offset + angle_test, r_offset - angle_test);
        }
    } else {
        joint_limit(angle_l_target, angle_r_target);
    }

    if (fly_slope_active) {
        joint_l->motor_settings.outer_loop_type = ANGLE_LOOP;
        joint_l->motor_settings.close_loop_type = SPEED_LOOP | ANGLE_LOOP;
        joint_r->motor_settings.outer_loop_type = ANGLE_LOOP;
        joint_r->motor_settings.close_loop_type = SPEED_LOOP | ANGLE_LOOP;
        joint_l->motor_controller.pid_ref = dipAngle;
        joint_r->motor_controller.pid_ref = dipAngle;
    } else {
        joint_l->motor_settings.outer_loop_type = ANGLE_LOOP;
        joint_l->motor_settings.close_loop_type = SPEED_LOOP | ANGLE_LOOP;
        joint_r->motor_settings.outer_loop_type = ANGLE_LOOP;
        joint_r->motor_settings.close_loop_type = SPEED_LOOP | ANGLE_LOOP;
        joint_l->motor_controller.pid_ref = dipAngle;   //两个关节控制器共同跟一个“机身俯仰参考”
        joint_r->motor_controller.pid_ref = dipAngle;
    }
/*--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
    // 根据云台和底盘的角度offset将控制量映射到底盘坐标系上
    // 底盘逆时针旋转为角度正方向;云台命令的方向以云台指向的方向为x,采用右手系(x指向正北时y在正东)
    chassis_vx = chassis_cmd_recv.vx * cos_theta - chassis_cmd_recv.vy * sin_theta;//把上层命令从云台坐标系旋转到底盘坐标系
    chassis_vy = chassis_cmd_recv.vx * sin_theta + chassis_cmd_recv.vy * cos_theta;
    //chassis_vx*=vxy_k;
    //chassis_vy*=vxy_k;
/*--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
    // 根据控制模式进行正运动学解算,计算底盘输出
    MecanumCalculate();//把 chassis_vx/chassis_vy/wz 转成四轮目标转速 vt_*，并更新摩擦补偿状态。

    {
        float sync_belt_ref_target = 0.0f;

        switch (chassis_cmd_recv.chassis_mode) {
            case CHASSIS_CLIMB:
            case CHASSIS_CLIMB_WITH_PULL:
            case CHASSIS_CLIMB_WITH_PUSH:
            case CHASSIS_CLIMB_RETRACT:
            case CHASSIS_FLY_SLOPE:
            {
                float sync_belt_forward_add_ref = SYNC_BELT_SWITCH_SPEED_REF;
                if (edge_hit_preload_active || edge_hit_retract_active)
                    sync_belt_forward_add_ref = LEG_EDGE_HIT_SYNC_BELT_REF;
                else if (auto_retract_active)
                    sync_belt_forward_add_ref = LEG_AUTO_RETRACT_SYNC_BELT_REF;

                if (chassis_cmd_recv.chassis_mode == CHASSIS_FLY_SLOPE) {
                    sync_belt_ref_target = LEG_SYNC_BELT_FLY_SLOPE_REF * LEG_SYNC_BELT_FLY_SLOPE_REF_SIGN;
                } else {
                    if (chassis_vx > SYNC_BELT_DIRECTION_DEADBAND)
                        sync_belt_ref_target = LEG_SYNC_BELT_PRELOAD_REF + sync_belt_forward_add_ref;
                    else if (chassis_vx < -SYNC_BELT_DIRECTION_DEADBAND)
                        sync_belt_ref_target = -(LEG_SYNC_BELT_PRELOAD_REF + sync_belt_forward_add_ref);
                    else
                        sync_belt_ref_target = LEG_SYNC_BELT_PRELOAD_REF;
                }

                if (sync_belt_ref_target > LEG_SYNC_BELT_MAX_REF)
                    sync_belt_ref_target = LEG_SYNC_BELT_MAX_REF;
                else if (sync_belt_ref_target < -LEG_SYNC_BELT_MAX_REF)
                    sync_belt_ref_target = -LEG_SYNC_BELT_MAX_REF;
                break;
            }
            default:
                break;
        }

        sync_belt_ref_state += clamp_absf(sync_belt_ref_target - sync_belt_ref_state, LEG_SYNC_BELT_SLEW_STEP);

        if (fabsf(sync_belt_ref_state) > 1.0f) {
            float sync_belt_left_ref  = sync_belt_ref_state * SYNC_BELT_LEFT_REF_SIGN;
            float sync_belt_right_ref = sync_belt_ref_state * SYNC_BELT_RIGHT_REF_SIGN;
            DJIMotorSetRef(sync_belt_motor_l, sync_belt_left_ref);
            DJIMotorSetRef(sync_belt_motor_r, sync_belt_right_ref);
            DJIMotorEnable(sync_belt_motor_l);
            DJIMotorEnable(sync_belt_motor_r);
        } else {
            sync_belt_ref_state = 0.0f;
            DJIMotorStop(sync_belt_motor_l);
            DJIMotorStop(sync_belt_motor_r);
        }
    }

    if (chassis_cmd_recv.mecanum_force_enable && (chassis_cmd_recv.chassis_mode != CHASSIS_ZERO_FORCE))
        ChassisForceControlMecanum();
    else
        ChassisForceReset();
    //如果麦轮力控开关打开，并且不处于急停模式，就执行力控计算；否则就把力控前馈清零，等于没用力控。
/*--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
    // 根据裁判系统的反馈数据和电容数据对输出限幅并设定闭环参考值
    Super_Cap_control();//根据裁判系统回传的 power_buffer 和超级电容的电压，修正底盘功率输出，并把结果送进 PowerControlupdate()；最后把四个轮子的目标转速 vt_* 写给电机
    SuperCapTask();//超级电容的通信和控制任务，确保数据更新和命令下发的时序正确；如果放在 ChassisTask() 里直接调用 SuperCap_control()，可能会因为通信延迟导致数据不同步。
#endif                                                         // CHASSIS_BOARD || ONE_BOARD
    // 获得给电容传输的电容吸取功率等级
    // Power_get();
    // 给电容传输数据
    // SuperCapSend(cap, (uint8_t *)&cap->cap_msg_g);
    float cap_voltage = SuperCapGetChassisVoltage(supercap);
    uint8_t cap_online = SuperCapIsOnline(supercap);
    // 推送反馈消息
    memcpy(&chassis_feedback_data.cap_voltage, &cap_voltage, sizeof(float));//把电容电压写入反馈数据结构
    memcpy(&chassis_feedback_data.cap_online_flag, &cap_online, sizeof(uint8_t));//把电容在线状态写入反馈数据结构
    memcpy(&chassis_feedback_data.chassis_power_output, &Power_Output, sizeof(float));//把底盘最终功率输出写入反馈数据结构
    chassis_feedback_data.chassis_voltage = cap_voltage;
    UpdateChassisDebugFeedback();
   
#ifdef ONE_BOARD
    PubPushMessage(chassis_pub, (void *)&chassis_feedback_data);
#endif
#ifdef CHASSIS_BOARD
    PubPushMessage(chassis_pub, (void *)&chassis_feedback_data);//把反馈数据通过发布者发出去，给上层的监控界面或者其他模块用；如果是双板结构，还可以通过 CAN 把数据发给另一个板。
    // memcpy(&chassis_feedback_data.power_cal,&power_data.total_power,sizeof(float));
    // memcpy(&chassis_feedback_data.power_real,&power_value,sizeof(float));
    // memcpy(&chassis_feedback_data.angle,Chassis_IMU_data->output.INS_angle,3*sizeof(float));
    //CANCommSend(chasiss_can_comm, (void *)&chassis_feedback_data);
#endif // CHASSIS_BOARD

}
