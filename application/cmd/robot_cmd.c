// app
#include <math.h>
#include "robot_board.h"
#include "robot_params.h"
#include "robot_types.h"
#include "robot_def.h"
#include "robot_cmd.h"
#include "omni_UI.h"
// module
#include "wit_c_sdk.h"
#include "IIC_CONTROL.h"
#include "hwt606.h"
#include "remote_control.h"
#include "ins_task.h"
#include "master_process.h"
#include "message_center.h"
#include "general_def.h"
#include "dji_motor.h"
#include "referee_UI.h"
#include "referee_init.h"
#include "fifo.h"
#include "tool.h"
#include "super_cap.h"
#include "unicomm.h"
#include "AHRS_MiddleWare.h"
#include "rm_referee.h"
#include "crc_ref.h"
// bsp
#include "bsp_dwt.h"
#include "bsp_log.h"
#include "vofa.h"
#define RC_LOST (rc_data[TEMP].rc.switch_left == 0 && rc_data[TEMP].rc.switch_right == 0)

// 私有宏,自动将编码器转换成角度值
#define YAW_ALIGN_ANGLE (YAW_CHASSIS_ALIGN_ECD * ECD_ANGLE_COEF_DJI) // 对齐时的角度,0-360
#if PITCH_FEED_TYPE                                                  // Pitch电机反馈数据源为陀螺仪
#define PTICH_HORIZON_ANGLE 0                                        // PITCH水平时电机的角度
#if PITCH_ECD_UP_ADD
#define PITCH_LIMIT_ANGLE_UP   (((PITCH_POS_UP_LIMIT_ECD > PITCH_HORIZON_ECD) ? (PITCH_POS_UP_LIMIT_ECD - PITCH_HORIZON_ECD) : (PITCH_POS_UP_LIMIT_ECD + 8192 - PITCH_HORIZON_ECD)) * ECD_ANGLE_COEF_DJI)       // 云台竖直方向最大角度 0-360
#define PITCH_LIMIT_ANGLE_DOWN (((PITCH_POS_DOWN_LIMIT_ECD < PITCH_HORIZON_ECD) ? (PITCH_POS_DOWN_LIMIT_ECD - PITCH_HORIZON_ECD) : (PITCH_POS_DOWN_LIMIT_ECD - 8192 - PITCH_HORIZON_ECD)) * ECD_ANGLE_COEF_DJI) // 云台竖直方向最小角度 0-360
#else
#define PITCH_LIMIT_ANGLE_UP   (((PITCH_POS_UP_LIMIT_ECD < PITCH_HORIZON_ECD) ? (PITCH_POS_UP_LIMIT_ECD - PITCH_HORIZON_ECD) : (PITCH_POS_UP_LIMIT_ECD - 8192 - PITCH_HORIZON_ECD)) * ECD_ANGLE_COEF_DJI)       // 云台竖直方向最大角度 0-360
#define PITCH_LIMIT_ANGLE_DOWN (((PITCH_POS_DOWN_LIMIT_ECD > PITCH_HORIZON_ECD) ? (PITCH_POS_DOWN_LIMIT_ECD - PITCH_HORIZON_ECD) : (PITCH_POS_DOWN_LIMIT_ECD + 8192 - PITCH_HORIZON_ECD)) * ECD_ANGLE_COEF_DJI) // 云台竖直方向最小角度 0-360
#endif
#else           // PITCH电机反馈数据源为编码器
                                                
#define PTICH_HORIZON_ANGLE    (PITCH_HORIZON_ECD * ECD_ANGLE_COEF_DJI) // PITCH水平时电机的角度,0-360
#define PITCH_LIMIT_ANGLE_UP   (PITCH_POS_MAX_ECD * ECD_ANGLE_COEF_DJI) // 云台竖直方向最大角度 0-360
#define PITCH_LIMIT_ANGLE_DOWN (PITCH_POS_MIN_ECD * ECD_ANGLE_COEF_DJI) // 云台竖直方向最小角度 0-360
#endif
// 双板控制链路超时判定（毫秒）
// 短暂抖动仅停射击，长时间失联才进入无力保护
#define RS485_CTRL_LINK_WARN_TIMEOUT_MS       120u
#define RS485_CTRL_LINK_ZERO_FORCE_TIMEOUT_MS 600u
// 串口Host守护阈值，避免过短导致频繁误判离线
#define RS485_HOST_DAEMON_RELOAD              30u
// 键鼠爬坡时限制前后速度，避免麦轮过快抢在同步带前顶坡
#define KEYBOARD_CLIMB_SPEED                  12000.0f
// 底盘平移加速斜坡，避免跟随模式急加速激发腿部俯仰补偿
#define CHASSIS_CMD_SPEED_SLEW_STEP_X           80.0f
#define CHASSIS_CMD_SPEED_SLEW_STEP_Y           70.0f
#define CHASSIS_CMD_SPEED_STOP_DEADBAND        50.0f
// 键鼠手动伸腿控制：X切换，底盘侧用该状态切换伸腿姿态目标
#define KEYBOARD_LEG_EXTEND_ENABLE_CMD        1.0f
static uint32_t rs485_last_rx_ms = 0;
static uint8_t rs485_link_online_once = 0;
    float p[4];
/* cmd应用包含的模块实例指针和交互信息存储*/
#if defined(GIMBAL_BOARD) || defined(ONE_BOARD) // 双板通信和视觉通信都在GIMBAL_BOARD和ONE_BOARD上,如果是底盘板就不初始化这两个通信了
#include "can_comm.h"
static CANCommInstance *cmd_can_comm; // 双板通信
#endif
#ifdef CHASSIS_BOARD
static Publisher_t *chassis_cmd_pub;   // 底盘控制消息发布者
static Subscriber_t *chassis_feed_sub; // 底盘反馈信息订阅者
#endif                                 // ONE_BOARD
static float nuc_pitch,nuc_yaw;
static Chassis_Ctrl_Cmd_s chassis_cmd_send;      // 发送给底盘应用的信息,包括控制信息和UI绘制相关
static Chassis_Ctrl_Cmd_s_half_float chassis_cmd_send_half_float;
static Chassis_Upload_Data_s chassis_fetch_data; // 从底盘应用接收的反馈信息信息,底盘功率枪口热量与底盘运动状态等
static Chassis_Ctrl_Cmd_s_uart chassis_cmd_send_uart;
static Chassis_Upload_Data_s_uart chassis_fetch_data_uart;
extern referee_info_t *referee_data_for_ui;
static RC_ctrl_t *rc_data; // 遥控器数据,初始化时返回
gimbal_mode_e gimbal_mode_last;
HostInstance *host_instance; // 上位机接口
HostInstance *IMU_instance; // 接口
HostInstance *rs485_master_instance; // 接口
HostInstance *rs485_slaver_instance; // 接口
// 这里的四元数以wxyz的顺序
static uint8_t vision_recv_data[9];  // 从视觉上位机接收的数据-绝对角度，第9个字节作为识别到目标的标志位
#define NUC_SEND_SIZE 43u
uint8_t vision_send_data[NUC_SEND_SIZE]; // 给视觉上位机发送的数据
uint8_t chasssis_ctrl_data[UNICOMM_CTRL_FRAME_LEN];
uint8_t chasssis_update_data[UNICOMM_UPLOAD_FRAME_LEN];
static Publisher_t *gimbal_cmd_pub  ;            // 云台控制消息发布者
static Subscriber_t *gimbal_feed_sub;          // 云台反馈信息订阅者
static Gimbal_Ctrl_Cmd_s gimbal_cmd_send;      // 传递给云台的控制信息
static Gimbal_Upload_Data_s gimbal_fetch_data; // 从云台获取的反馈信息
extern float gimbal_pitch_vel_measure;

static Publisher_t *shoot_cmd_pub;           // 发射控制消息发布者
static Subscriber_t *shoot_feed_sub;         // 发射反馈信息订阅者
static Shoot_Ctrl_Cmd_s shoot_cmd_send;      // 传递给发射的控制信息
static Shoot_Upload_Data_s shoot_fetch_data; // 从发射获取的反馈信息
int Shoot_Aim_Angle = 0;
uint16_t Switch_Left_Last;
static Publisher_t *ui_cmd_pub;        // UI控制消息发布者
static Subscriber_t *ui_feed_sub;      // UI反馈信息订阅者
static UI_Cmd_s ui_cmd_send;           // 传递给UI的控制信息
static UI_Upload_Data_s ui_fetch_data; // 从UI获取的反馈信息

static Robot_Status_e robot_state; // 机器人整体工作状态
int Trig_Time = 0;	//发射触发时间
static referee_info_t *referee_data; // 用于获取裁判系统的数据
static hwt606_info_t *HWT606_data;
uint8_t UI_SendFlag = 1; // UI发送标志位
#define MECANUM_FORCE_UI_FLAG_BIT 0x80u
#define UI_SEND_FLAG_MASK         0x7Fu
#define VISION_WORK_IDLE          0u
#define VISION_WORK_AIM           1u
#define VISION_WORK_SMALL_RUNE    2u
#define VISION_WORK_BIG_RUNE      3u
extern uint16_t g_power_set ; // 功率设置
uint8_t auto_rune; // 自瞄打符标志位
static uint8_t keyboard_vision_work_mode = VISION_WORK_IDLE;

float rec_yaw, rec_pitch;
float fire_advice;
float vision_yaw_vel = 0; // 视觉提供的yaw速度前馈
float vision_yaw_acc = 0; // 视觉提供的yaw加速度前馈, deg/s^2
static uint8_t vision_angle_fire_enable = ROBOTCMD_VISION_ANGLE_FIRE_DEFAULT_ENABLE;
static uint8_t vision_angle_fire_bullets = ROBOTCMD_VISION_ANGLE_FIRE_DEFAULT_BULLETS;
// #define Chassis_Ctrl_Cmd_s_uart_size sizeof(Chassis_Ctrl_Cmd_s_uart)
// #define Chassis_Upload_Data_s_uart_size sizeof(Chassis_Upload_Data_s_uart)
uint8_t SuperCap_flag_from_user = 0; // 超电标志位
uint8_t rc_update_flag = 0;//遥控器数据更新标志位（防止同一个周期多次触发）
static uint8_t mecanum_force_ctrl_enable = 0u; // 麦轮力控独立开关（与 chassis_mode 解耦）
static uint8_t keyboard_head_tail_reverse = 0u;
static int8_t keyboard_sync_belt_dir = 0;
static uint8_t keyboard_sync_belt_key_last = 0u;

static chassis_mode_e GetRemoteClimbMode(void)
{
    return (rc_data[TEMP].rc.switch_left == RC_SW_DOWN) ? CHASSIS_CLIMB_RETRACT : CHASSIS_CLIMB;
}

void RobotCMDSetVisionAngleFireEnable(uint8_t enable)
{
    vision_angle_fire_enable = (enable != 0u) ? 1u : 0u;
}

uint8_t RobotCMDGetVisionAngleFireEnable(void)
{
    return vision_angle_fire_enable;
}

void RobotCMDSetVisionAngleFireBullets(uint8_t bullets)
{
    vision_angle_fire_bullets = (bullets >= ROBOTCMD_VISION_FIRE_TRIPLE) ?
                                ROBOTCMD_VISION_FIRE_TRIPLE :
                                ROBOTCMD_VISION_FIRE_SINGLE;
}

uint8_t RobotCMDGetVisionAngleFireBullets(void)
{
    return vision_angle_fire_bullets;
}

static loader_mode_e RobotCMDGetVisionFireLoadMode(void)
{
    if ((uint8_t)fire_advice != ROBOTCMD_VISION_FIRE_ADVICE_VALUE) {
        return LOAD_STOP;
    }

#if ROBOTCMD_VISION_FIRE_AIM_GATE_ENABLE
    {
        if (gimbal_fetch_data.gimbal_imu_data == NULL) {
            return LOAD_STOP;
        }

        float yaw_err = gimbal_cmd_send.yaw_version -
                        gimbal_fetch_data.gimbal_imu_data->output.INS_angle_deg[INS_YAW_ADDRESS_OFFSET];
        const float pitch_err = gimbal_cmd_send.pitch_version -
                                gimbal_fetch_data.gimbal_imu_data->output.INS_angle[INS_PITCH_ADDRESS_OFFSET];

        if (yaw_err > 180.0f)
            yaw_err -= 360.0f;
        else if (yaw_err < -180.0f)
            yaw_err += 360.0f;

        if (fabsf(yaw_err) > ROBOTCMD_VISION_FIRE_YAW_ERR_GATE_DEG ||
            fabsf(pitch_err) > ROBOTCMD_VISION_FIRE_PITCH_ERR_GATE_RAD) {
            return LOAD_STOP;
        }
    }
#endif

    if (vision_angle_fire_enable == 0u) {
        return LOAD_BURSTFIRE;
    }

    return (vision_angle_fire_bullets >= ROBOTCMD_VISION_FIRE_TRIPLE) ? LOAD_3_BULLET : LOAD_1_BULLET;
}

static uint8_t RobotCMDInMouseKeyMode(void)
{
    return (uint8_t)(switch_is_up(rc_data[TEMP].rc.switch_left) &&
                     switch_is_down(rc_data[TEMP].rc.switch_right));
}

static uint8_t CMDChassisModeIsClimb(chassis_mode_e mode)
{
    return (mode == CHASSIS_CLIMB ||
            mode == CHASSIS_CLIMB_RETRACT ||
            mode == CHASSIS_CLIMB_WITH_PULL ||
            mode == CHASSIS_CLIMB_WITH_PUSH) ? 1u : 0u;
}

static uint8_t CMDChassisModeIsClimbUi(chassis_mode_e mode)
{
    return (mode == CHASSIS_CLIMB ||
            mode == CHASSIS_CLIMB_RETRACT ||
            mode == CHASSIS_CLIMB_WITH_PULL ||
            mode == CHASSIS_CLIMB_WITH_PUSH ||
            mode == CHASSIS_FLY_SLOPE) ? 1u : 0u;
}

static int8_t CMDGetSyncBeltUiState(void)
{
    if (chassis_cmd_send.sync_belt_cmd > 0)
        return 1;
    if (chassis_cmd_send.sync_belt_cmd < 0)
        return -1;
    if (CMDChassisModeIsClimbUi(chassis_cmd_send.chassis_mode) == 0u)
        return 0;
    if (chassis_cmd_send.vx < -50.0f)
        return -1;
    return 1;
}

static void ApplyAutoAimNoFollowMode(void)
{
    if (gimbal_cmd_send.nuc_mode == version_control &&
        chassis_cmd_send.chassis_mode == CHASSIS_FOLLOW_GIMBAL_YAW) {
        chassis_cmd_send.chassis_mode = CHASSIS_NO_FOLLOW;
    }
}

static void LimitKeyboardClimbSpeed(void)
{
    if (!CMDChassisModeIsClimb(chassis_cmd_send.chassis_mode))
        return;

    if (chassis_cmd_send.vx > KEYBOARD_CLIMB_SPEED)
        chassis_cmd_send.vx = KEYBOARD_CLIMB_SPEED;
    else if (chassis_cmd_send.vx < -KEYBOARD_CLIMB_SPEED)
        chassis_cmd_send.vx = -KEYBOARD_CLIMB_SPEED;
}

static float WrapAngle180(float angle)
{
    while (angle > 180.0f)
        angle -= 360.0f;
    while (angle < -180.0f)
        angle += 360.0f;
    return angle;
}

static void ApplyKeyboardHeadTailSwitch(void)
{
    if (keyboard_head_tail_reverse)
        chassis_cmd_send.offset_angle = WrapAngle180(chassis_cmd_send.offset_angle + 180.0f);
}

static void ApplyKeyboardHeadTailMoveSwitch(void)
{
    if (keyboard_head_tail_reverse) {
        chassis_cmd_send.vx = -chassis_cmd_send.vx;
        chassis_cmd_send.vy = -chassis_cmd_send.vy;
    }
}

static void KeyboardLegExtendSet(void)
{
    chassis_cmd_send.leg_length_cmd =
        (rc_data[TEMP].key_count[KEY_PRESS][Key_X] % 2) ? KEYBOARD_LEG_EXTEND_ENABLE_CMD : 0.0f;
}

static void KeyboardSyncBeltSet(void)
{
    uint8_t key_g_now = (rc_data[TEMP].key[KEY_PRESS].g &&
                         !rc_data[TEMP].key[KEY_PRESS].ctrl &&
                         !rc_data[TEMP].key[KEY_PRESS].shift) ? 1u : 0u;

    if (key_g_now && !keyboard_sync_belt_key_last) {
        // G 键按下沿三态循环：停 -> 正转 -> 反转 -> 停。
        if (keyboard_sync_belt_dir == 0)
            keyboard_sync_belt_dir = 1;
        else if (keyboard_sync_belt_dir > 0)
            keyboard_sync_belt_dir = -1;
        else
            keyboard_sync_belt_dir = 0;
    }
    keyboard_sync_belt_key_last = key_g_now;

    if (chassis_cmd_send.chassis_mode == CHASSIS_ZERO_FORCE)
        keyboard_sync_belt_dir = 0;
    chassis_cmd_send.sync_belt_cmd = keyboard_sync_belt_dir;
}

static uint8_t RobotCMDGetVisionWorkMode(void)
{
    if (rc_data[TEMP].mouse.press_r && keyboard_vision_work_mode == VISION_WORK_IDLE)
        return VISION_WORK_AIM;

    return keyboard_vision_work_mode;
}

static uint8_t RobotCMDGetVisionTxMode(void)
{
    uint8_t mode = RobotCMDGetVisionWorkMode();

    if (!RobotCMDInMouseKeyMode())
        return (gimbal_cmd_send.nuc_mode == version_control) ? VISION_WORK_AIM : VISION_WORK_IDLE;

    if (mode == VISION_WORK_IDLE && gimbal_cmd_send.nuc_mode == version_control)
        mode = VISION_WORK_AIM;

    return mode;
}

static void KeyboardVisionModeSet(void)
{
    keyboard_vision_work_mode = (uint8_t)(rc_data[TEMP].key_count[KEY_PRESS][Key_B] % 4u);
    auto_rune = (keyboard_vision_work_mode >= VISION_WORK_SMALL_RUNE) ? 1u : 0u;
    gimbal_cmd_send.nuc_mode = (RobotCMDGetVisionWorkMode() == VISION_WORK_IDLE) ? none_version_control : version_control;
}

static float ApproachFloat(float current, float target, float step)
{
    float diff = target - current;

    if (diff > step)
        return current + step;
    if (diff < -step)
        return current - step;
    return target;
}

static uint8_t ChassisModeUseSpeedRamp(chassis_mode_e mode)
{
    return (mode == CHASSIS_FOLLOW_GIMBAL_YAW ||
            mode == CHASSIS_NO_FOLLOW ||
            mode == CHASSIS_ROTATE ||
            mode == CHASSIS_REVERSE_ROTATE ||
            mode == CHASSIS_MECANUM_FORCE) ? 1u : 0u;
}

static float ApplyChassisAccelRamp(float state, float target, float accel_step)
{
    if (fabsf(target) < CHASSIS_CMD_SPEED_STOP_DEADBAND)
        return target;

    if (fabsf(state) < CHASSIS_CMD_SPEED_STOP_DEADBAND)
        state = 0.0f;

    // 只保留加速缓变；松键、降速、反向切换不再缓慢拖到目标，避免溜车。
    if ((target * state) <= 0.0f)
        return ApproachFloat(0.0f, target, accel_step);

    if (fabsf(target) > fabsf(state))
        return ApproachFloat(state, target, accel_step);

    return target;
}

static void RobotCMDApplyChassisSpeedRamp(void)
{
    static float speed_x_state = 0.0f;
    static float speed_y_state = 0.0f;

    if (chassis_cmd_send.chassis_mode == CHASSIS_ZERO_FORCE) {
        speed_x_state = 0.0f;
        speed_y_state = 0.0f;
        chassis_cmd_send.vx = 0.0f;
        chassis_cmd_send.vy = 0.0f;
        return;
    }

    if (!ChassisModeUseSpeedRamp(chassis_cmd_send.chassis_mode)) {
        speed_x_state = chassis_cmd_send.vx;
        speed_y_state = chassis_cmd_send.vy;
        return;
    }

    speed_x_state = ApplyChassisAccelRamp(speed_x_state, chassis_cmd_send.vx,
                                          CHASSIS_CMD_SPEED_SLEW_STEP_X);
    speed_y_state = ApplyChassisAccelRamp(speed_y_state, chassis_cmd_send.vy,
                                          CHASSIS_CMD_SPEED_SLEW_STEP_Y);
    chassis_cmd_send.vx = speed_x_state;
    chassis_cmd_send.vy = speed_y_state;
}

void RobotCMDSetMecanumForceCtrl(uint8_t enable)
{
    mecanum_force_ctrl_enable = (enable != 0u) ? 1u : 0u;
}

uint8_t RobotCMDGetMecanumForceCtrl(void)
{
    return mecanum_force_ctrl_enable;
}

// float imu_angle[3];
// float imu_gyro[3];
static PIDInstance PITCH_version_PID = {
    .Kp            = 0.005,   // 25,//25, // 50,//70, // 4.5
    .Ki            = 0,    // 0
    .Kd            = 0.008, // 0.0,  // 0.07,  // 0
    .DeadBand      = 0.3,  // 0.75,  //跟随模式设置了死区，防止抖动
    .IntegralLimit = 30,
    .Improve       = PID_OutputFilter|PID_Derivative_On_Measurement|PID_Integral_Limit|PID_Trapezoid_Intergral/*PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement*/,
    .MaxOut        = 0.04,
    .Output_LPF_RC=0.95,
};
static PIDInstance YAW_version_PID = {
    .Kp            = 0.0037,//0.02,//0.033,   // 25,//25, // 50,//70, // 4.5
    .Ki            = 0,    // 0
    .Kd            = 0.00055,//0.001,//0.003, // 0.0,  // 0.07,  // 0
    .DeadBand      = 0.75,  // 0.75,  //跟随模式设置了死区，防止抖动
    .IntegralLimit = 3000,
    .Improve       = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement|PID_OutputFilter,
    .MaxOut        = 0.1,//0.05,//0.045,
    .Output_LPF_RC=0.7,
};
float alphaA=0.1,alphaB=0.7;
// 视觉提供的yaw速度前馈进行滤波，防止过大过小导致震荡
float nuc_yaw_filiter(float input){
    static float input_last;
    float output=0;
    if(input<0)output=alphaB*input+(1-alphaB)*input_last;
    else output=alphaB*input+(1-alphaB)*input_last;
    input_last=output;
    return output;
}
float nuc_pitch_filiter(float input){
    static float input_last;
    float output=alphaA*input+(1-alphaA)*input_last;
    input_last=output;
    return output;
}
//视觉数据接收回调函数，从视觉上位机接收数据并存储在vision_recv_data数组中，第9个字节作为识别到目标的标志位，调用时会先喂狗以防止守护进程误判离线。
void HOST_RECV_CALLBACK()
{
    memcpy(vision_recv_data, host_instance->comm_instance, host_instance->RECV_SIZE);
    vision_recv_data[8] = 1;
}
//FIFO写入函数
static void FIFO_WRITE(uint16_t recv_len){
    DaemonReload(host_instance->daemon);         // 先喂狗
    if (recv_len == 0u) {
        return;
    }
    if (recv_len > FIFO_SIZE) {
        recv_len = FIFO_SIZE;
    }
    WritePacketToFIFO(&NUC_fifo, host_instance->comm_instance, (uint8_t)recv_len);
}
uint16_t float_to_half(float f) {
    uint32_t bit_pattern;
    memcpy(&bit_pattern, &f, sizeof(f));  // 获取float的二进制表示

    uint32_t sign = (bit_pattern >> 31) & 0x1;
    uint32_t exponent = (bit_pattern >> 23) & 0xFF;
    uint32_t mantissa = bit_pattern & 0x7FFFFF;

    // 处理非规范化数
    if (exponent == 0) {
        return (sign << 15); // 非规范化数
    }

    // 处理无穷大和NaN
    if (exponent == 0xFF) {
        return (sign << 15) | (0x1F << 10) | (mantissa >> 13);
    }

    // 规范化数的转换
    int half_exponent = exponent - 127 + 15;
    if (half_exponent > 0x1F) half_exponent = 0x1F;  // 最大指数
    if (half_exponent < 0) half_exponent = 0;  // 最小指数

    uint16_t half = (sign << 15) | (half_exponent << 10) | (mantissa >> 13);
    return half;
}
//欧拉角到四元数转换
void EularAngleToQuaternion(float Y, float P, float R, float *q)
{
    float cosPitch, cosYaw, cosRoll, sinPitch, sinYaw, sinRoll;
    // Y /= 57.295779513f;
    // P /= 57.295779513f;
    // R /= 57.295779513f;
    cosPitch = arm_cos_f32(P / 2);
    cosYaw   = arm_cos_f32(Y / 2);
    cosRoll  = arm_cos_f32(R / 2);
    sinPitch = arm_sin_f32(P / 2);
    sinYaw   = arm_sin_f32(Y / 2);
    sinRoll  = arm_sin_f32(R / 2);
q[0] = cosYaw * cosPitch * cosRoll + sinYaw * sinPitch * sinRoll;
q[1] = cosYaw * cosPitch * sinRoll - sinYaw * sinPitch * cosRoll;
q[2] = cosYaw * sinPitch * cosRoll + sinYaw * cosPitch * sinRoll;
q[3] = sinYaw * cosPitch * cosRoll - cosYaw * sinPitch * sinRoll;
}

// 半精度浮点数转单精度浮点数
float half_to_float(uint16_t half) {
    uint32_t sign = (half >> 15) & 0x1;           // 符号位
    uint32_t exponent = (half >> 10) & 0x1F;      // 指数位
    uint32_t mantissa = half & 0x3FF;             // 尾数位

    // 处理非规范化数
    if (exponent == 0) {
        // 非规范化数处理（需要补零）
        return sign ? -0.0f : 0.0f;
    }

    // 处理无穷大和NaN
    if (exponent == 0x1F) {
        if (mantissa == 0) {
            return sign ? -INFINITY : INFINITY;
        } else {
            return NAN;
        }
    }

    // 规范化数的转换
    int32_t float_exp = (int32_t)(exponent) - 15 + 127; // 调整指数偏移量
    uint32_t float_mantissa = mantissa << 13;           // 扩展尾数至23位

    // 生成32位浮点数（符号 + 指数 + 尾数）
    uint32_t float_bits = (sign << 31) | (float_exp << 23) | float_mantissa;

    float result;
    memcpy(&result, &float_bits, sizeof(result));  // 将位模式转换为浮点数
    return result;
}
// 将 float 拆分为 4 个 uint8_t 字节（手动处理高低位）
void float_to_uint8_manual(float value, uint8_t *bytes) {
    uint32_t as_int;
    memcpy(&as_int, &value, sizeof(float)); // 将 float 转换为 uint32_t

    *bytes = (uint8_t)(as_int & 0xFF);         // 第 1 个字节（低位）
    *(bytes+1) = (uint8_t)((as_int >> 8) & 0xFF);  // 第 2 个字节
    *(bytes+2) = (uint8_t)((as_int >> 16) & 0xFF); // 第 3 个字节
    *(bytes+3) = (uint8_t)((as_int >> 24) & 0xFF); // 第 4 个字节（高位）
}

// 将 4 个 uint8_t 字节解析回 float（手动处理高低位）
float uint8_to_float_manual(uint8_t *bytes) {
    uint32_t as_int = 0;

    as_int |= ((uint32_t)bytes[0]);
    as_int |= ((uint32_t)bytes[1] << 8);
    as_int |= ((uint32_t)bytes[2] << 16);
    as_int |= ((uint32_t)bytes[3] << 24);

    float value;
    memcpy(&value, &as_int, sizeof(float)); // 将 uint32_t 转换回 float
    return value;
}

// 将 int32_t 转换为 uint8_t 数组（长度 4）
void int32_to_uint8_array(int32_t value, uint8_t *array) {
    array[0] = (uint8_t)((value >> 24) & 0xFF); // 高位字节
    array[1] = (uint8_t)((value >> 16) & 0xFF);
    array[2] = (uint8_t)((value >> 8) & 0xFF);
    array[3] = (uint8_t)(value & 0xFF);        // 低位字节
}

// 从 uint8_t 数组还原为 int32_t
int32_t uint8_array_to_int32(const uint8_t *array) {
    return (int32_t)(
        ((int32_t)array[0] << 24) | // 高位字节
        ((int32_t)array[1] << 16) |
        ((int32_t)array[2] << 8) |
        ((int32_t)array[3])
    );
}
typedef union {
    uint8_t bytes[4];
    float value;
} FloatUnion;
float bytesToFloat(uint8_t* data) {
    FloatUnion u;
    for(int k=0;k<4;k++)u.bytes[k] = data[k];
    return u.value;
}
volatile uint32_t time_cnt;
float delta_t,delta_t_last;
void chassis_ctrl_485(Chassis_Ctrl_Cmd_s_uart data)
{
    static uint8_t master_tx_buf[UNICOMM_CTRL_FRAME_LEN];
    if (UniCommPackChassisCtrl(&data, master_tx_buf, sizeof(master_tx_buf)) == UNICOMM_CTRL_FRAME_LEN) {
        HAL_UART_Transmit_DMA(&huart1, master_tx_buf, UNICOMM_CTRL_FRAME_LEN);
    }
}
void chassis_update_485(Chassis_Upload_Data_s_uart data)
{
    static uint8_t slaver_tx_buf[UNICOMM_UPLOAD_FRAME_LEN];
    if (UniCommPackChassisUpload(&data, slaver_tx_buf, sizeof(slaver_tx_buf)) == UNICOMM_UPLOAD_FRAME_LEN) {
        HAL_UART_Transmit_DMA(&huart1, slaver_tx_buf, UNICOMM_UPLOAD_FRAME_LEN);
    }
}
void  chassis_update(){
chassis_update_485(chassis_fetch_data_uart);
}
// void getImu_485(){
//     static uint8_t imu_tx_buf[4];
//     imu_tx_buf[0]=0X50;
//     imu_tx_buf[1]=0X5A;
//     imu_tx_buf[2]=0XCC;
//     imu_tx_buf[3]=0XFF;
//     HAL_UART_Transmit_DMA(&huart1,imu_tx_buf,4);
// }
// uint8_t imu_rx_data[33];
// short angle[3],gyro[3],accel[3];
// void HWT606Decode(uint8_t *data){
    
//     if(data[0]==0x55&&data[1]==0x51){
//         if(hwt606_check_sum(data))for(int k=0;k<3;k++)accel[k]=(short)((short)data[2*k+3]<<8|data[2*k+2])*16*9.8/32768;
//     }   
//     if(data[11]==0x55&&data[12]==0x52){
//         if(hwt606_check_sum(data+11))for(int k=0;k<3;k++)gyro[k]=(short)((short)data[2*k+14]<<8|data[2*k+13])*2000/32768;
//     }
//     if(data[22]==0x55&&data[23]==0x53){
//         if(hwt606_check_sum(data+22))for(int k=0;k<3;k++)angle[k]=(short)((short)data[2*k+25]<<8|data[2*k+3+24])*180/32768;
//     }
// }
uint8_t ImuData[82];
float cnt;
// void getIMU(){
//     DaemonReload(IMU_instance->daemon);         // 先喂狗
//     cnt=DWT_GetDeltaT(&time_cnt); // 自动初始化
//     memcpy(ImuData,IMU_instance->comm_instance,82);
//     if(ImuData[0]==0x5A&&ImuData[1]==0xA5){
//         for(int k=0;k<3;k++){
//             imu_angle[k]=bytesToFloat(ImuData+4*k+54);
//             imu_gyro[k]=ANGLE_TO_RAD*bytesToFloat(ImuData+4*k+30);
//         }
//     }
//     imu_angle[1]*=-1;
// }
void rs485_master_writefifo()
{
    DaemonReload(rs485_master_instance->daemon);         // 先喂狗 
    WritePacketToFIFO(&rs485_master_fifo,rs485_master_instance->comm_instance,UNICOMM_UPLOAD_FRAME_LEN);
}
void rs485_slaver_writefifo()
{
    DaemonReload(rs485_slaver_instance->daemon);         // 先喂狗  
    chassis_update_485(chassis_fetch_data_uart);
    WritePacketToFIFO(&rs485_slaver_fifo,rs485_slaver_instance->comm_instance,UNICOMM_CTRL_FRAME_LEN);
}

uint8_t text_rx[29];
//-------------------------------------------------------------imu-----------------------------------------------------------------
/* 私有类型定义 --------------------------------------------------------------*/
float fAcc[3], fGyro[3], fAngle[3];
#define ACC_UPDATE		0x01
#define GYRO_UPDATE		0x02
#define ANGLE_UPDATE	0x04
#define MAG_UPDATE		0x08
#define READ_UPDATE		0x80
static volatile char s_cDataUpdate = 0, s_cCmd = 0xff;
static void CopeSensorData(uint32_t uiReg, uint32_t uiRegNum)
{
	int i;
    for(i = 0; i < uiRegNum; i++)
    {
        switch(uiReg)
        {
//            case AX:
//            case AY:
            case AZ:
				s_cDataUpdate |= ACC_UPDATE;
            break;
//            case GX:
//            case GY:
            case GZ:
				s_cDataUpdate |= GYRO_UPDATE;
            break;
//            case HX:
//            case HY:
            case HZ:
				s_cDataUpdate |= MAG_UPDATE;
            break;
//            case Roll:
//            case Pitch:
            case Yaw:
				s_cDataUpdate |= ANGLE_UPDATE;
            break;
            default:
				s_cDataUpdate |= READ_UPDATE;
			break;
        }
		uiReg++;
    }
}
static void AutoScanSensor(void)
{
	int i, iRetry;
	
	for(i = 0; i < 0x7F; i++)
	{
		WitInit(WIT_PROTOCOL_I2C, i);
		iRetry = 2;
		do
		{
			s_cDataUpdate = 0;
			WitReadReg(AX, 3);
			osDelay(5);
			if(s_cDataUpdate != 0)
			{
				printf("find %02X addr sensor\r\n", i);
				//ShowHelp();
				return ;
			}
			iRetry--;
		}while(iRetry);		
	}
	printf("can not find sensor\r\n");
	printf("please check your connection\r\n");
}
uint8_t *uart1_rx;
uint8_t usb_rxBufffer[11];
uint8_t* usbRxPointer;
void usb_rxCallback(){
    memcpy(usb_rxBufffer,usbRxPointer,11);
}
//-------------------------------------------------------------imu-----------------------------------------------------------------
void RobotCMDInit()
{
    DWT_GetDeltaT(&time_cnt); // 自动初始化
    gimbal_cmd_pub  = PubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
    gimbal_feed_sub = SubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));
    shoot_cmd_pub   = PubRegister("shoot_cmd", sizeof(Shoot_Ctrl_Cmd_s));
    shoot_feed_sub  = SubRegister("shoot_feed", sizeof(Shoot_Upload_Data_s));
#if defined(ONE_BOARD) || defined(CHASSIS_BOARD)
    ui_cmd_pub  = PubRegister("ui_cmd", sizeof(UI_Cmd_s));
    ui_feed_sub = SubRegister("ui_feed", sizeof(UI_Upload_Data_s));
    chassis_cmd_send.power_limit=100;//预设功率
    // g_power_set=&chassis_cmd_send.power_limit;
    referee_data = RefereeHardwareInit(&huart6); // 裁判系统初始化,会同时初始化UI
    
    HostInstanceConf host_conf = {
        .usart_handle=&huart1,
        .callback  = rs485_slaver_writefifo,
        .comm_mode = HOST_USART,
        .RECV_SIZE = UNICOMM_CTRL_FRAME_LEN,
    };
    
    rs485_slaver_instance = HostInit(&host_conf, RS485_HOST_DAEMON_RELOAD); // 双板通信串口
    uart1_rx=rs485_slaver_instance->comm_instance;
    chassis_cmd_pub  = PubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));
    chassis_feed_sub = SubRegister("chassis_feed", sizeof(Chassis_Upload_Data_s));
    
#endif

    rc_data = RemoteControlInit(&huart3); // 修改为对应串口,注意如果是自研板dbus协议串口需选用添加了反相器的那个
#if defined(ONE_BOARD) || defined(GIMBAL_BOARD)
    HostInstanceConf host_conf0 = {

        .callback  = FIFO_WRITE,
        .comm_mode = HOST_VCP,
        .RECV_SIZE = NUC_RECV_SIZE,
    };
    host_instance = HostInit(&host_conf0,90); // 视觉通信串口

    HostInstanceConf host_conf = {
        .usart_handle=&huart1,
        .callback  = rs485_master_writefifo,
        .comm_mode = HOST_USART,
        .RECV_SIZE = UNICOMM_UPLOAD_FRAME_LEN,
    };
    rs485_master_instance = HostInit(&host_conf, RS485_HOST_DAEMON_RELOAD); // 双板通信串口

    // HostInstanceConf host_conf2 = {
    //     .usart_handle=&huart6,
    //     .callback  = getIMU,
    //     .comm_mode = HOST_USART,
    //     .RECV_SIZE = 82,
    // };
    // IMU_instance = HostInit(&host_conf2,2); // 陀螺仪通信串口
    // ui_cmd_pub  = PubRegister("ui_cmd", sizeof(UI_Cmd_s));
    // ui_feed_sub = SubRegister("ui_feed", sizeof(UI_Upload_Data_s));

    // CANComm_Init_Config_s comm_conf = {
    //     .can_config = {
    //         .can_handle = &hcan2,
    //         .tx_id      = 0x312,
    //         .rx_id      = 0x311,
    //     },
    //     .recv_data_len = 8,
    //     .send_data_len = 8,
    // };
    // cmd_can_comm = CANCommInit(&comm_conf);
#if PITCH_FEED_TYPE
    gimbal_cmd_send.pitch = 0;
#else
    gimbal_cmd_send.pitch = PTICH_HORIZON_ANGLE;
#endif
   gimbal_cmd_send.pitch = PTICH_HORIZON_ANGLE;
#endif
    chassis_cmd_send.chassis_mode = CHASSIS_FOLLOW_GIMBAL_YAW;
    chassis_cmd_send.mecanum_force_enable = RobotCMDGetMecanumForceCtrl();
robot_state = ROBOT_READY; // 启动时机器人进入工作模式,后续加入所有应用初始化完成之后再进入
}

/**
 * @brief  判断各种ID，选择客户端ID
 * @param  referee_info_t *referee_info
 * @retval none
 * @attention
 */
static void DeterminRobotID()
{
    // id小于7是红色,大于7是蓝色,0为红色，1为蓝色   #define Robot_Red 0    #define Robot_Blue 1
    referee_data->referee_id.Robot_Color       = referee_data->GameRobotState.robot_id == 1 ? Robot_Red : Robot_Blue;
    referee_data->referee_id.Cilent_ID         = 0x0100 + referee_data->GameRobotState.robot_id; // 计算客户端ID
    referee_data->referee_id.Robot_ID          = referee_data->GameRobotState.robot_id;          // 计算机器人ID
    referee_data->referee_id.Receiver_Robot_ID = 0;
}

float yaw_control;   // 遥控器YAW自由度输入值
float pitch_control; // 遥控器PITCH自由度输入值

/**
 * @brief 根据gimbal app传回的当前电机角度计算和零位的误差
 *        单圈绝对角度的范围是0~360,说明文档中有图示
 * @todo  将单圈角度修改为-180~180
 *
 */
static void  CalcOffsetAngle()
{
    // 别名angle提高可读性,不然太长了不好看,虽然基本不会动这个函数
    static float angle;
    static float gimbal_yaw_current_angle;                                                // 云台yaw轴当前角度
    static float gimbal_yaw_set_angle;                                                    // 云台yaw轴目标角度
    angle                               = chassis_fetch_data_uart.yaw_motor_single_round_angle;//gimbal_fetch_data.yaw_motor_single_round_angle; // 从云台获取的当前yaw电机单圈角度
    gimbal_yaw_current_angle            = gimbal_fetch_data.gimbal_imu_data->output.INS_angle_deg[INS_YAW_ADDRESS_OFFSET];//从云台陀螺仪获取的当前Yaw轴角度
    gimbal_yaw_set_angle                = yaw_control;
    chassis_cmd_send.gimbal_error_angle = gimbal_yaw_set_angle - gimbal_yaw_current_angle; // 云台误差角

#if YAW_ECD_GREATER_THAN_4096 // 如果大于180度
    if (angle < 180.0f + YAW_ALIGN_ANGLE && angle >= YAW_ALIGN_ANGLE - 180.0f)
        chassis_cmd_send.offset_angle =- (angle - YAW_ALIGN_ANGLE);
    else
        chassis_cmd_send.offset_angle =- (angle - YAW_ALIGN_ANGLE + 360.0f);
#else // 小于180度
    if (angle >= YAW_ALIGN_ANGLE - 180.0f && angle <= YAW_ALIGN_ANGLE + 180.0f) {
        chassis_cmd_send.offset_angle = -(angle - YAW_ALIGN_ANGLE);
    } else {
        chassis_cmd_send.offset_angle = -(angle - YAW_ALIGN_ANGLE - 360.0f);
    }
#endif
}
/**
 * @brief 对Pitch轴角度变化进行动态限位
 *
 */
float pitch_control_max,pitch_control_min;
static void PitchAngle_ActiveLimit()
{
    
    // if(gimbal_cmd_send.gimbal_mode==GIMBAL_MOTOR_MODE){
    //     pitch_control_max=205;
    //     pitch_control_min=155;
    // }
    // else {
    // pitch_control_max=gimbal_fetch_data.gimbal_imu_data->output.INS_angle_deg[1]+(PITCH_POS_UP_LIMIT_ECD-gimbal_fetch_data.pitch_ecd)*ECD_ANGLE_COEF_DJI;
    // pitch_control_min=gimbal_fetch_data.gimbal_imu_data->output.INS_angle_deg[1]-(gimbal_fetch_data.pitch_ecd-PITCH_POS_DOWN_LIMIT_ECD)*ECD_ANGLE_COEF_DJI;
    // // }
    // if(pitch_control>pitch_control_max)      pitch_control=pitch_control_max;
    // if(pitch_control<pitch_control_min)pitch_control=pitch_control_min;

}
/**
 * @brief 对Pitch轴角度变化进行限位
 *
 */
static void PitchAngleLimit()
{
    float limit_min, limit_max;
#if PITCH_INS_FEED_TYPE
    limit_min = -20.0f;//PITCH_LIMIT_ANGLE_DOWN * DEGREE_2_RAD;
    limit_max = 37.0f;//PITCH_LIMIT_ANGLE_UP * DEGREE_2_RAD;
#else
    limit_min = -24;//PITCH_LIMIT_ANGLE_DOWN;
    limit_max = 34;//PITCH_LIMIT_ANGLE_UP;
#endif

#if PITCH_ECD_UP_ADD // 云台抬升,反馈值增
    if (pitch_control > limit_max)
        pitch_control = limit_max;
    if (pitch_control < limit_min)
        pitch_control = limit_min;
#else
    if (pitch_control < limit_max)
        pitch_control = limit_max;
    if (pitch_control > limit_min)
        pitch_control = limit_min;
#endif

//     gimbal_cmd_send.pitch = pitch_control;
}

/**
 * @brief 云台Yaw轴反馈值改单圈角度后过圈处理
 *
 */
static void YawControlProcess()
{
    // if (yaw_control - ECD_ANGLE_COEF_DJI*gimbal_fetch_data.yaw_ecd > 180) {
    //     yaw_control -= 360;
    // } else if (yaw_control - ECD_ANGLE_COEF_DJI*gimbal_fetch_data.yaw_ecd < -180) {
    //     yaw_control += 360;
    // }
    // if(chassis_cmd_send.chassis_mode==CHASSIS_NO_FOLLOW){
    //     if (yaw_control - chassis_fetch_data_uart.yaw_total_angle > 180) {
    //     yaw_control -= 360;
    // } else if (yaw_control - chassis_fetch_data_uart.yaw_total_angle < -180) {
    //     yaw_control += 360;
    // }}
    // else {
    if (yaw_control - gimbal_fetch_data.gimbal_imu_data->output.INS_angle_deg[INS_YAW_ADDRESS_OFFSET] > 180) { //读取陀螺仪数据进行过圈处理
        yaw_control -= 360;
    } else if (yaw_control - gimbal_fetch_data.gimbal_imu_data->output.INS_angle_deg[INS_YAW_ADDRESS_OFFSET] < -180) {
        yaw_control += 360;
    }
}

static float heat_coef;

#define ROBOTCMD_ONE_BULLET_HEAT_COST 10.0f
#define ROBOTCMD_HEAT_SAFE_MARGIN     2.0f

static uint8_t RobotCMDIsFeedMode(loader_mode_e mode)
{
    return (mode == LOAD_1_BULLET ||
            mode == LOAD_BURSTFIRE ||
            mode == LOAD_3_BULLET) ? 1u : 0u;
}

static void HeatControl()
{
    if (shoot_cmd_send.friction_mode == FRICTION_OFF) {
        shoot_cmd_send.load_mode = LOAD_STOP;
        heat_coef = 0.0f;
        return;
    }

    if (RobotCMDIsFeedMode(shoot_cmd_send.load_mode) == 0u) {
        return;
    }

    const float cooling_limit = (float)shoot_cmd_send.shooter_cooling_limit;
    if (cooling_limit <= 1.0f) {
        return;
    }

    const float referee_heat = (float)shoot_cmd_send.shooter_referee_heat;
    const float local_heat = shoot_fetch_data.shooter_local_heat;
    const float heat_now = (local_heat > referee_heat) ? local_heat : referee_heat;
    const float min_heat_to_fire = ROBOTCMD_ONE_BULLET_HEAT_COST + ROBOTCMD_HEAT_SAFE_MARGIN;

    // Some referee protocol/layout combinations report this limit as a small
    // cooling value (for example 40). Treat that as invalid for hard blocking,
    // otherwise a valid heat reading around 40 would lock the loader forever.
    if (cooling_limit <= min_heat_to_fire * 2.0f) {
        return;
    }

    heat_coef = (cooling_limit - heat_now) / cooling_limit;
    if (heat_coef < 0.0f)
        heat_coef = 0.0f;
    else if (heat_coef > 1.0f)
        heat_coef = 1.0f;

    // if ((cooling_limit - heat_now) < min_heat_to_fire) {
    //     shoot_cmd_send.load_mode = LOAD_STOP;
    // }
}

// 底盘模式
static uint8_t rc_mode[6];
#define CHASSIS_FREE     0
#define CHASSIS_ROTATION 1
#define CHASSIS_FOLLOW   2
#define SHOOT_FRICTION   3
#define SHOOT_LOAD       4
#define VISION_MODE      5

/**
 * @brief  紧急停止,包括遥控器左上侧拨轮打满/重要模块离线/双板通信失效等
 *
 * @todo   后续修改为遥控器离线则电机停止(关闭遥控器急停),通过给遥控器模块添加daemon实现
 */
static void EmergencyHandler()
{
    gimbal_cmd_send.gimbal_mode   = GIMBAL_ZERO_FORCE;
    gimbal_mode_last=GIMBAL_ZERO_FORCE;
    chassis_cmd_send.chassis_mode = CHASSIS_ZERO_FORCE;
    chassis_cmd_send.leg_length_cmd = 0.0f;
    keyboard_sync_belt_dir = 0;
    keyboard_sync_belt_key_last = 0u;
    chassis_cmd_send.sync_belt_cmd = 0;
    RobotCMDSetMecanumForceCtrl(1);
    //RobotCMDSetMecanumForceCtrl(0);
    chassis_cmd_send.mecanum_force_enable = 0u;
    shoot_cmd_send.friction_mode  = FRICTION_OFF;
    shoot_cmd_send.load_mode      = LOAD_STOP;
    shoot_cmd_send.shoot_mode     = SHOOT_OFF;
    SuperCap_flag_from_user     = SUPERCAP_UNUSE;
    memset(rc_mode, 1, sizeof(uint8_t));
    memset(rc_mode + 1, 0, sizeof(uint8_t) * 4);
    LOGERROR("[CMD] emergency stop!");
}
float pitch_nuc,yaw_nuc;
extern int load_mode_private;
static short nuc_pitch_last,nuc_yaw_last;
float kp,kd;
float pitch_shoot_angle=16.4;
float alpha_nuc_pitch=0.1,alpha_nuc_yaw=0.02,nuc_yaw_d=0.63,nuc_pitch_d=0.1,yaw_filiter=0.23,pitch_filiter,yaw_nuc_err,yaw_control_last;
float last=0,alpha=0.7;
float low_pass_filiter(float input){
    
    float output=alpha*input+(1-alpha)*last;
    last=output;
    return output;
}

void gimbal_limit(void)
{
    if(pitch_control > 0.4) pitch_control = 0.4;
    if(pitch_control < -0.4) pitch_control = -0.4;
    if(gimbal_cmd_send.pitch_version > 0.4) gimbal_cmd_send.pitch_version = 0.4;
    if(gimbal_cmd_send.pitch_version < -0.4) gimbal_cmd_send.pitch_version = -0.4;
}
//float pitch_control_max,pitch_control_min;
/**
 * @brief 控制输入为遥控器(调试时)的模式和控制量设置
 */
static void RemoteControlSet()
{
    // 遥控模式下的默认状态：
    // 1) 云台使用陀螺仪模式
    // 2) 发射总开关开启，射频先给默认值
    shoot_cmd_send.shoot_mode   = SHOOT_ON; // 发射机构常开
    gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;
    shoot_cmd_send.shoot_rate   = 16; // 射频默认30Hz
    chassis_cmd_send.leg_length_cmd = 0.0f;
    keyboard_sync_belt_dir = 0;
    keyboard_sync_belt_key_last = 0u;
    chassis_cmd_send.sync_belt_cmd = 0;

    // 拨轮只保留给视觉自瞄：上抬沿触发开/关（仅触发一次，靠 rc_update_flag 防抖）

    if (rc_update_flag == 1)
    {
        if (rc_data[TEMP].rc.switch_right != RC_SW_UP &&
            rc_data[TEMP].rc.dial > 250 && rc_data[LAST].rc.dial < 250)
        {
            if (gimbal_cmd_send.nuc_mode != version_control)
                gimbal_cmd_send.nuc_mode = version_control;
            else
                gimbal_cmd_send.nuc_mode = none_version_control;
        }

        // if (rc_data[TEMP].rc.dial > 250 && rc_data[LAST].rc.dial < 250)
        // {
        //     if (SuperCap_flag_from_user == SUPERCAP_UNUSE)
        //         SuperCap_flag_from_user = SUPERCAP_USE;
        //     else
        //         SuperCap_flag_from_user = SUPERCAP_UNUSE;
        // }
        // gimbal_cmd_send.nuc_mode = none_version_control;
        
        // 左侧三段开关：摩擦轮与发射触发逻辑
        // UP:  中->上切换摩擦轮开关
        // MID: 下->中停止供弹
        // DOWN: 中->下触发发射（非自瞄连发/自瞄按 fire_advice）
        switch (rc_data[TEMP].rc.switch_left)
        {
            case RC_SW_UP:

                if (rc_data[LAST].rc.switch_left == RC_SW_MID)//左中到上开关摩擦轮
                {
                    if (shoot_cmd_send.friction_mode == FRICTION_ON)
                        shoot_cmd_send.friction_mode = FRICTION_OFF;
                    else
                        shoot_cmd_send.friction_mode = FRICTION_ON;
                }
                break;

            case RC_SW_DOWN:
                // 右拨杆上位时，左拨杆下位只作为收腿输入，不触发射击。
                if (rc_data[TEMP].rc.switch_right == RC_SW_UP) {
                    break;
                }

                if (gimbal_cmd_send.nuc_mode == none_version_control)
                {
                    if (rc_data[LAST].rc.switch_left == RC_SW_MID && shoot_cmd_send.friction_mode == FRICTION_ON)//左中到下且开摩擦轮时连发
                    {
                        shoot_cmd_send.load_mode = LOAD_BURSTFIRE;
                    }
                }
                else
                {
                    shoot_cmd_send.load_mode = RobotCMDGetVisionFireLoadMode();
                }
                break;
            case RC_SW_MID:
                if (rc_data[LAST].rc.switch_left == RC_SW_DOWN)
                {
                    shoot_cmd_send.load_mode = LOAD_STOP;
                }
                break;
        }

        rc_update_flag = 0;
    }

    if (rc_data[TEMP].rc.switch_left == RC_SW_DOWN &&
        rc_data[TEMP].rc.switch_right != RC_SW_UP &&
        shoot_cmd_send.friction_mode == FRICTION_ON) {
        if (gimbal_cmd_send.nuc_mode == version_control)
            shoot_cmd_send.load_mode = RobotCMDGetVisionFireLoadMode();
        else
            shoot_cmd_send.load_mode = LOAD_BURSTFIRE;
    }

    // 右侧三段开关：底盘模式主切换（固定档位映射）
    // 该映射按“当前档位”每周期生效，不依赖 rc_update_flag。
    // UP:   爬坡家族。左拨杆下位时收腿，其他位置普通爬坡
    // MID:  跟随云台
    // DOWN: 小陀螺
    switch (rc_data[TEMP].rc.switch_right)
    {
        case RC_SW_UP:
            chassis_cmd_send.chassis_mode = GetRemoteClimbMode();
            break;
        case RC_SW_DOWN:
            chassis_cmd_send.chassis_mode = CHASSIS_ROTATE;
            break;
        case RC_SW_MID:
            chassis_cmd_send.chassis_mode = CHASSIS_FOLLOW_GIMBAL_YAW;
            break;
        default:
            chassis_cmd_send.chassis_mode = CHASSIS_FOLLOW_GIMBAL_YAW;
            break;
    }

    // 云台控制输入来源：
    // none_version_control: 直接使用遥控器摇杆累计控制
    // version_control:      使用上位机给定角度作为基准, 仍叠加遥控器即时微调
//    HeatControl();
    if(gimbal_cmd_send.nuc_mode == version_control)
    {
        pitch_control = gimbal_cmd_send.pitch_version;
        yaw_control = gimbal_cmd_send.yaw_version;
    }
    pitch_control += PITCH_K* (float)rc_data[TEMP].rc.rocker_l1 ;
    yaw_control -= /*0.05**/YAW_K * (float)rc_data[TEMP].rc.rocker_l_ ;
    
    // 右摇杆映射到底盘平移速度
    chassis_cmd_send.vx = 70.0f * (float)rc_data[TEMP].rc.rocker_r1; // 水平方向
    chassis_cmd_send.vy = 70.0f * (float)rc_data[TEMP].rc.rocker_r_; // 竖直方向
    // chassis_cmd_send.wz = 7.0f * (float)rc_data[TEMP].rc.rocker_l_;
//     // // 云台参数
//     // 云台软件限位
    // PitchAngle_ActiveLimit();// PITCH限位
    gimbal_limit();
    YawControlProcess();
    
     gimbal_cmd_send.yaw   = yaw_control;
     gimbal_cmd_send.pitch = pitch_control;    
}

/**
 * @brief 键盘设定速度
 *
 */
static void ChassisSet()
{
    // 键鼠控制时给底盘一个基准模式，后续可被 KeyGetMode 覆盖（例如 C 键切小陀螺）
    chassis_cmd_send.chassis_mode = CHASSIS_FOLLOW_GIMBAL_YAW;
    // chassis_cmd_send.chassis_mode = CHASSIS_ZERO_FORCE;
    // chassis_cmd_send.power_limit=100;//临时更改

    float target_speed_x = 0.0f;
    float target_speed_y = 0.0f;

    // 前后方向目标
    if (rc_data[TEMP].key[KEY_PRESS].w) {
        target_speed_x = CHASSIS_SPEED;
    } else if (rc_data[TEMP].key[KEY_PRESS].s) {
        target_speed_x = -CHASSIS_SPEED;
    } else if (rc_data[TEMP].key[KEY_PRESS_WITH_CTRL].w) {
        target_speed_x = 1000.0f;
    } else if (rc_data[TEMP].key[KEY_PRESS_WITH_CTRL].s) {
        target_speed_x = -1000.0f;
    }

    // 左右方向目标
    if (rc_data[TEMP].key[KEY_PRESS].a) {
        target_speed_y = CHASSIS_SPEED;
    } else if (rc_data[TEMP].key[KEY_PRESS].d) {
        target_speed_y = -CHASSIS_SPEED;
    } else if (rc_data[TEMP].key[KEY_PRESS_WITH_CTRL].a) {
        target_speed_y = 1000.0f;
    } else if (rc_data[TEMP].key[KEY_PRESS_WITH_CTRL].d) {
        target_speed_y = -1000.0f;
    }

    chassis_cmd_send.vx = target_speed_x;
    chassis_cmd_send.vy = target_speed_y;
    KeyboardLegExtendSet();
    KeyboardSyncBeltSet();
}
float yaw_kx=500,pitch_ky=1000;
/**
 * @brief 鼠标移动云台
 *
 */
float freequence,pitch_vision_delta;
float lob_rate=0.1;
static void GimbalSet()
{
    (void)YAW_version_PID;
    // 键鼠控制下，云台默认用陀螺仪模式：
    // 鼠标输入提供增量控制；若开启自瞄则直接使用视觉给定值。

    // 相对角度控制
    // memcpy(&rec_yaw, vision_recv_data, sizeof(float));
    // memcpy(&rec_pitch, vision_recv_data + 4, sizeof(float));
    // pitch_control=gimbal_fetch_data.gimbal_imu_data->output.INS_angle_deg[0]+gimbal_cmd_send.pitch_version;
    // 按住鼠标右键且视觉识别到目标
    // if(lob_mode)
    // {
    //     gimbal_cmd_send.gimbal_mode = GIMBAL_MOTOR_MODE;
    //     yaw_control -= rc_data[TEMP].mouse.x / 500.0f * lob_rate;
    //     pitch_control += rc_data[TEMP].mouse.y / 1000.0f * lob_rate;
    // }
    // else
    // {
        gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;
        if(gimbal_cmd_send.nuc_mode == version_control)
        {
            yaw_control = gimbal_cmd_send.yaw_version;
            pitch_control = gimbal_cmd_send.pitch_version;
        }
        yaw_control -= rc_data[TEMP].mouse.x / 500.0f;
        pitch_control -= rc_data[TEMP].mouse.y / 9000.0f;
    // }
    pitch_vision_delta=gimbal_cmd_send.pitch_version/freequence;
    static float pitch_rotato_vision,yaw_rotato_vision;
    // E 键/右键按住时，根据视觉误差做补偿（PID）
    if(rc_data[TEMP].key[KEY_PRESS].e){
        if(fabs(gimbal_cmd_send.pitch_version)<16){
            pitch_rotato_vision=PIDCalculate(&PITCH_version_PID,gimbal_cmd_send.pitch_version,0);
            // pitch_control-=PIDCalculate(&PITCH_version_PID,gimbal_cmd_send.pitch_version,0);
            if(fabs(pitch_rotato_vision)<0.035)pitch_control-=pitch_rotato_vision;}
        // if(fabs(gimbal_cmd_send.pitch_version)<16)pitch_control-=PIDCalculate(&PITCH_version_PID,gimbal_cmd_send.pitch_version,0);
        (void)yaw_rotato_vision;
    }
    else if (rc_data[TEMP].mouse.press_r){
        // pitch_control-=PIDCalculate(&PITCH_version_PID,gimbal_cmd_send.pitch_version,0);
        // yaw_control-=PIDCalculate(&YAW_version_PID,gimbal_cmd_send.yaw_version,0);
        if(fabs(gimbal_cmd_send.pitch_version)<16){
            
            pitch_rotato_vision=PIDCalculate(&PITCH_version_PID,gimbal_cmd_send.pitch_version,0);
            // pitch_control-=PIDCalculate(&PITCH_version_PID,gimbal_cmd_send.pitch_version,0);
            if(fabs(pitch_rotato_vision)<0.035)pitch_control-=pitch_rotato_vision;
            
        }
    }
    
    // if(fabs(gimbal_cmd_send.pitch_version)<16)pitch_control-=PIDCalculate(&PITCH_version_PID,gimbal_cmd_send.pitch_version,0);
    // if(fabs(gimbal_cmd_send.yaw_version)<20)yaw_control-=PIDCalculate(&YAW_version_PID,gimbal_cmd_send.yaw_version,0);
    YawControlProcess();
    gimbal_limit();
    gimbal_cmd_send.yaw   = yaw_control;
    gimbal_cmd_send.pitch = pitch_control;
}

/**
 * @brief 键鼠设定机器人发射模式
 *
 */
static uint8_t mouse_r_last=0;
static void ShootSet()
{
    // 发射逻辑说明：
    // 1) 仅摩擦轮开启时，左键触发发射
    // 2) 视觉模式下按 fire_advice 决定连发/停发
    // 3) Ctrl+G 强制反转处理卡弹
    shoot_cmd_send.shoot_mode = SHOOT_ON;
    shoot_cmd_send.shoot_rate = 30; // 射频默认30Hz

    // 仅在摩擦轮开启时有效
    if (shoot_cmd_send.friction_mode == FRICTION_ON) {
        // 打弹，单击左键单发，长按连发
        if (rc_data[TEMP].mouse.press_l) 
        {
            shoot_cmd_send.load_mode = LOAD_BURSTFIRE;
            // 打符，单发
            // if (auto_rune == 1) {
            //     shoot_cmd_send.load_mode = LOAD_1_BULLET;
            // } else {
            //     shoot_cmd_send.load_mode = LOAD_BURSTFIRE;
            // }
        } else {
            if (gimbal_cmd_send.nuc_mode == none_version_control)
            {
                shoot_cmd_send.load_mode = LOAD_STOP;
            }
            else
            {
                shoot_cmd_send.load_mode = RobotCMDGetVisionFireLoadMode();
            }
            
        }
    } else {
        shoot_cmd_send.load_mode = LOAD_STOP;
    }
    if (rc_data[TEMP].key[KEY_PRESS_WITH_CTRL].g){
        shoot_cmd_send.load_mode = LOAD_REVERSE;
    }
    gimbal_cmd_send.nuc_mode = (RobotCMDGetVisionWorkMode() == VISION_WORK_IDLE) ? none_version_control : version_control;
    mouse_r_last = rc_data[TEMP].mouse.press_r;
    HeatControl();
}

/**
 * @brief 键盘处理模式标志位
 *
 */

static void KeyGetMode()
{
    // 键盘模式切换：
    // C: 底盘 跟随<->旋转
    // Q: 爬坡模式 开/关
    // Z: 按住手动收腿
    // X: 手动伸腿姿态 开/关
    // F: 头尾互换 + 飞坡模式 开/关
    // G: 同步带手动正转/反转/停止
    // Ctrl+G: 拨弹反转处理卡弹
    // V: 摩擦轮 开/关
    // Shift: 超电使能
    // B: 视觉模式 空闲->自瞄->小符->大符
    // Ctrl: 打符模式标志
    switch (rc_data[TEMP].key_count[KEY_PRESS][Key_C] % 2) {
        case 1:
            if (chassis_cmd_send.chassis_mode == CHASSIS_FOLLOW_GIMBAL_YAW)
                chassis_cmd_send.chassis_mode = CHASSIS_ROTATE;
            break;
        case 0:
            if (chassis_cmd_send.chassis_mode == CHASSIS_ROTATE)
                chassis_cmd_send.chassis_mode = CHASSIS_FOLLOW_GIMBAL_YAW;
            break;
    }
    switch (rc_data[TEMP].key_count[KEY_PRESS][Key_Q] % 2) {
        case 1:
            chassis_cmd_send.chassis_mode = CHASSIS_CLIMB;
            break;
        case 0:
        default:
            break;
    }
    keyboard_head_tail_reverse = (rc_data[TEMP].key_count[KEY_PRESS][Key_F] % 2) ? 1u : 0u;
    if (rc_data[TEMP].key_count[KEY_PRESS][Key_F] % 2) {
        chassis_cmd_send.chassis_mode = CHASSIS_FLY_SLOPE;
    }
    if (rc_data[TEMP].key[KEY_PRESS].z) {
        chassis_cmd_send.chassis_mode = CHASSIS_CLIMB_RETRACT;
    }
    ApplyKeyboardHeadTailMoveSwitch();
    LimitKeyboardClimbSpeed();
    switch (rc_data[TEMP].key_count[KEY_PRESS][Key_V] % 2) {
        case 1:
            if (shoot_cmd_send.friction_mode != FRICTION_ON)
                shoot_cmd_send.friction_mode = FRICTION_ON;
            break;
        case 0:
            shoot_cmd_send.friction_mode = FRICTION_OFF;
            break;
    }
    // switch (rc_data[TEMP].key_count[KEY_PRESS][Key_Z] % 2) {
    //     chassis_mode_e temp;
    //     temp=chassis_cmd_send.chassis_mode;
    //     case 1:
    //         if (chassis_cmd_send.chassis_mode == CHASSIS_REVERSE_ROTATE)
    //             chassis_cmd_send.chassis_mode = temp;
    //         break;
    //     case 0:
    //         if (chassis_cmd_send.chassis_mode == temp)
    //             chassis_cmd_send.chassis_mode = CHASSIS_REVERSE_ROTATE;
    //         break;
    // }
    switch (rc_data[TEMP].key[KEY_PRESS].r) {
        case 1:
            if (UI_SendFlag == 1) {
                UI_SendFlag = 0;
            }
            break;
        case 0:
            UI_SendFlag = 1;
            break;
    }
    
    if (keyboard_vision_work_mode < VISION_WORK_SMALL_RUNE)
        auto_rune = rc_data[TEMP].key[KEY_PRESS].ctrl ? 1u : 0u;
    switch (rc_data[TEMP].key[KEY_PRESS].shift) {
        case 1:
            SuperCap_flag_from_user = SUPERCAP_USE;
            break;
        case 0:
            SuperCap_flag_from_user = SUPERCAP_UNUSE;
            break;
    }
}

/**
 * @brief 机器人复位函数，按下Ctrl+Shift+r
 *
 *
 */
static void RobotReset()
{
    if (rc_data[TEMP].key[KEY_PRESS].shift && rc_data[TEMP].key[KEY_PRESS].ctrl && rc_data[TEMP].key[KEY_PRESS].r) {
        osDelay(1000);
        __set_FAULTMASK(1);
        NVIC_SystemReset(); // 软件复位
    }
    if (rc_data[TEMP].key[KEY_PRESS].shift && rc_data[TEMP].key[KEY_PRESS].ctrl && rc_data[TEMP].key[KEY_PRESS].e) {
    chassis_cmd_send_uart.reset_flag=7;
    }
}

/**
 * @brief 软件复位
 *
 */
static void MouseKeySet()
{
    // 键鼠控制主流程：
    // 底盘->模式键处理->云台->发射->复位检查
    ChassisSet();
    KeyboardVisionModeSet();
    KeyGetMode();
    GimbalSet();
    ShootSet();
    
    // PitchAngle_ActiveLimit();
    RobotReset(); // 机器人复位处理
}
static float  getkey_time= 0;
extern int32_t shoot_count; 
// extern referee_info_t *referee_data_for_ui;
int a=0;
uint8_t fifo_pack[NUC_RECV_SIZE];
uint8_t nuc_rx[8];
// short nuc_pitch,nuc_yaw;
void Version_devode()
{
    float pitch_version,yaw_version;
    pitch_version=2*3.14/8192*(float)(short)(nuc_rx[1]|nuc_rx[2]<<8);
    yaw_version=360.0/8192*(float)(short)(nuc_rx[3]|nuc_rx[4]<<8);
    if(fabs(yaw_version)>0.0001||yaw_version==0)gimbal_cmd_send.yaw_version=yaw_version;
    if(fabs(pitch_version)<0.0001||pitch_version==0)gimbal_cmd_send.pitch_version=pitch_version;

} 
#define checkNUC_num 20
float vision_filiter=1;
float checkNUCarray[2][checkNUC_num];
uint8_t ifTrueVision(float angle,uint8_t mode){
    checkNUCarray[mode][checkNUC_num]=angle;
    for(size_t i=0;i<checkNUC_num;i++){
        checkNUCarray[mode][i]=checkNUCarray[mode][i+1];
        }
    if(fabs(checkNUCarray[mode][0]-angle)< vision_filiter )return 1;
    else return 0;
}

 short pitch_version_cyc,yaw_version_cyc;
 float pitchChange,yawChange;
volatile static float distance;
float fp_pitch,fp_yaw,pitch_vel = 0;
float vision_pitch_acc = 0;
//-----------------------------------------------------------------------------------------NUC版本解码-----------------------------------------------------------------------------------------//
#define version_decode_to_angle 0.0439453125f
void USB_Version_devode(){
    float yaw_vel,pitch_acc,yaw_acc;
    uint8_t mode;

    if (fifo_pack[0] != 'C' || fifo_pack[1] != 'B') {
        return;
    }
    mode = fifo_pack[2];
    if (mode > 2u) {
        return;
    }
    fire_advice = mode;
    
    fp_yaw = uint8_to_float_manual(fifo_pack + 3);
    yaw_vel = RAD_2_DEGREE * uint8_to_float_manual(fifo_pack + 7);
    yaw_acc = RAD_2_DEGREE * uint8_to_float_manual(fifo_pack + 11);
    fp_pitch = uint8_to_float_manual(fifo_pack + 15);
    pitch_vel = uint8_to_float_manual(fifo_pack + 19);
    pitch_acc = uint8_to_float_manual(fifo_pack + 23);
    if (fire_advice == 0)
    {
        vision_yaw_vel = 0.0f;
        vision_yaw_acc = 0.0f;
        pitch_vel = 0.0f;
        vision_pitch_acc = 0.0f;
        gimbal_cmd_send.pitch_version = pitch_control;
        gimbal_cmd_send.yaw_version = yaw_control;
    }
    else
    {
        vision_yaw_vel = yaw_vel;
        vision_yaw_acc = yaw_acc;
        vision_pitch_acc = pitch_acc;
        gimbal_cmd_send.pitch_version = fp_pitch;
        gimbal_cmd_send.yaw_version = RAD_2_DEGREE * fp_yaw;
    }
}

uint8_t p_data[52];
float extract_value(const char *data, const char *key) {
    char *start = strstr(data, key);  // 查找键名的位置
    if (!start) return 0.0;           // 如果未找到，返回0

    start += strlen(key) + 1;         // 跳过键名和冒号
    return atof(start);               // 转换为浮点数
}
Chassis_Ctrl_Cmd_s_uart chassis_rs485_recv;
float voltage,current,power_value,energy;
extern Power_Data_s power_data; // 电机功率数据;
CAN_TxHeaderTypeDef can2_txheader;
uint8_t nuc_can_rx[8];
extern bool FIFO_READ_POWER(FIFOQueue *fifo, uint8_t *data, uint32_t len);
extern uint8_t calculate_checksum(uint8_t *data, uint8_t length);
CANInstance *nuc_Cantx;
CAN_TxHeaderTypeDef can2_txheader;
uint32_t can2_txmailbox;
float chassis_wzSet,chassis_wzK;
/* 机器人核心控制任务，当前由 FreeRTOS 以 1ms 周期调度（约 1000Hz） */
float refree_power_choice(uint8_t level){
    switch(level){
        case 1:
            return 70;
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
            return 70;
    }
}
float yawSpeedFeed;
float yaw_pid[3];
volatile static bool FIFO_state;

static void RobotCMDApplyControlInput(void)
{
    // 控制源仲裁：
    // 左上+右下 -> 键鼠
    // 双下或遥控丢失 -> 急停
    // 其他 -> 遥控模式
    if (RobotCMDInMouseKeyMode())
        MouseKeySet();
    else if (RC_LOST || (switch_is_down(rc_data[TEMP].rc.switch_left) && switch_is_down(rc_data[TEMP].rc.switch_right)))
        EmergencyHandler();
    else
        RemoteControlSet();

    ApplyAutoAimNoFollowMode();
    chassis_cmd_send.mecanum_force_enable = RobotCMDGetMecanumForceCtrl();
}

static void RobotCMDUpdateShootReferee(uint16_t cooling_rate, uint16_t referee_heat, uint16_t cooling_limit)
{
    memcpy(&shoot_cmd_send.shooter_heat_cooling_rate, &cooling_rate, sizeof(uint16_t));
    memcpy(&shoot_cmd_send.shooter_referee_heat, &referee_heat, sizeof(uint16_t));
    memcpy(&shoot_cmd_send.shooter_cooling_limit, &cooling_limit, sizeof(uint16_t));
    memcpy(&shoot_cmd_send.bullet_speed, &referee_data->ShootData.bullet_speed, sizeof(float));
}

#if defined(CHASSIS_BOARD)
static void RobotCMDTaskChassisBoard(void)
{
    //Judge_Uart();
    DeterminRobotID();
    RobotCMDApplyControlInput();
    uint32_t now_ms = HAL_GetTick();
    uint8_t rs485_ctrl_updated = 0u;
    SubGetMessage(gimbal_feed_sub, &gimbal_fetch_data);
    SubGetMessage(shoot_feed_sub, &shoot_fetch_data);
    SubGetMessage(chassis_feed_sub, (void *)&chassis_fetch_data);

    chassis_rs485_recv.reset_flag = 0;
    if (FIFO_Read_chassis_ctrl(&rs485_slaver_fifo, chasssis_ctrl_data, UNICOMM_CTRL_FRAME_LEN, UNICOMM_FRAME_HEAD, UNICOMM_FRAME_TYPE_CTRL_CMD) &&
        UniCommUnpackChassisCtrl(chasssis_ctrl_data, UNICOMM_CTRL_FRAME_LEN, &chassis_rs485_recv)) {
        rs485_last_rx_ms = now_ms;
        rs485_link_online_once = 1;
        rs485_ctrl_updated = 1u;
    }

    if (chassis_rs485_recv.reset_flag == 7) {
        osDelay(1000);
        __set_FAULTMASK(1);
        NVIC_SystemReset();
    }

    chassis_fetch_data_uart.yaw_motor_single_round_angle = gimbal_fetch_data.yaw_motor_single_round_angle;
    chassis_fetch_data_uart.yaw_total_angle = gimbal_fetch_data.yaw_total_angle;
    chassis_fetch_data_uart.yaw_ecd = gimbal_fetch_data.yaw_ecd;

    chassis_fetch_data_uart.chassis_yaw_gyro = gimbal_fetch_data.gimbal_imu_data->INS_data.INS_gyro[INS_YAW_ADDRESS_OFFSET];//陀螺仪反馈的底盘YAW角速度

    chassis_fetch_data_uart.chassis_pitch_angle = gimbal_fetch_data.gimbal_imu_data->output.INS_angle[1];
    chassis_fetch_data_uart.initial_speed = referee_data->ShootData.bullet_speed;
    chassis_fetch_data_uart.yaw_motor_real_current = gimbal_fetch_data.yaw_motor_real_current;
    chassis_fetch_data_uart.color = referee_data->referee_id.Robot_Color;
    chassis_fetch_data_uart.yaw_angle_pidout = gimbal_fetch_data.yaw_angle_pidout;

    {
        uint8_t local_mouse_key_mode = RobotCMDInMouseKeyMode();

        if (local_mouse_key_mode) {
            CalcOffsetAngle();
            ApplyKeyboardHeadTailSwitch();
            if (rs485_ctrl_updated) {
                shoot_cmd_send.load_mode = chassis_rs485_recv.load_mode;
                shoot_cmd_send.shoot_mode = chassis_rs485_recv.shoot_mode;
                shoot_cmd_send.shoot_count = chassis_rs485_recv.shoot_count;
                shoot_cmd_send.friction_mode = chassis_rs485_recv.friction_mode;
                gimbal_cmd_send.nuc_mode = chassis_rs485_recv.nuc_mode;
            }
        }
        if (!local_mouse_key_mode)
        {
            gimbal_cmd_send.yaw = chassis_rs485_recv.yaw_control;
            gimbal_cmd_send.gimbal_mode = chassis_rs485_recv.gimbal_mode;
            gimbal_cmd_send.nuc_mode = chassis_rs485_recv.nuc_mode;
            gimbal_cmd_send.yaw_version = chassis_rs485_recv.nuc_yaw;

            shoot_cmd_send.load_mode = chassis_rs485_recv.load_mode;
            shoot_cmd_send.shoot_mode = chassis_rs485_recv.shoot_mode;
            shoot_cmd_send.shoot_count = chassis_rs485_recv.shoot_count;
            shoot_cmd_send.friction_mode = chassis_rs485_recv.friction_mode;

            chassis_cmd_send.vx = chassis_rs485_recv.vx;
            chassis_cmd_send.vy = chassis_rs485_recv.vy;
            chassis_cmd_send.wz = chassis_rs485_recv.wz;
            if (rc_data[TEMP].rc.switch_right != RC_SW_UP)
                chassis_cmd_send.chassis_mode = chassis_rs485_recv.chassis_mode;
            chassis_cmd_send.offset_angle = chassis_rs485_recv.offset_angle;
            chassis_cmd_send.gimbal_error_angle = chassis_rs485_recv.gimbal_error_angle;
            chassis_cmd_send.leg_length_cmd = chassis_rs485_recv.leg_length_cmd;
            chassis_cmd_send.SuperCap_flag_from_user = chassis_rs485_recv.superCap_flag;
            chassis_cmd_send.mecanum_force_enable = (chassis_rs485_recv.UI_SendFlag & MECANUM_FORCE_UI_FLAG_BIT) ? 1u : 0u;
            chassis_cmd_send.sync_belt_cmd = chassis_rs485_recv.sync_belt_cmd;
        }
    }
    ApplyAutoAimNoFollowMode();
    // 最终态保护：右拨杆在 UP 时，底盘模式始终由本地左右拨杆判定
    // 防止前面链路（如串口同步）覆盖爬坡/收腿选择。
    if (rc_data[TEMP].rc.switch_right == RC_SW_UP) {
        chassis_cmd_send.chassis_mode = GetRemoteClimbMode();
    }

    RobotCMDUpdateShootReferee(referee_data->GameRobotState.shooter_id1_42mm_cooling_rate,
                               referee_data->PowerHeatData.shooter_17mm_heat0,
                               referee_data->GameRobotState.shooter_id1_42mm_cooling_limit);
    chassis_cmd_send.power_limit = referee_data->GameRobotState.chassis_power_limit;
    g_power_set = chassis_cmd_send.power_limit;
    chassis_cmd_send.level = referee_data->GameRobotState.robot_level;
    chassis_cmd_send.power_buffer = referee_data->PowerHeatData.chassis_power_buffer;
    if (RobotCMDInMouseKeyMode())
        chassis_cmd_send.SuperCap_flag_from_user = SuperCap_flag_from_user;
    SuperCap_flag_from_user = chassis_cmd_send.SuperCap_flag_from_user;

    uint32_t rs485_offline_ms = rs485_link_online_once ? (now_ms - rs485_last_rx_ms) : 0u;
    if (rs485_link_online_once && (rs485_offline_ms > RS485_CTRL_LINK_WARN_TIMEOUT_MS)) {
        shoot_cmd_send.load_mode = LOAD_STOP;
        shoot_cmd_send.shoot_mode = SHOOT_OFF;
    }
    if (rs485_link_online_once && (rs485_offline_ms > RS485_CTRL_LINK_ZERO_FORCE_TIMEOUT_MS)) {
        gimbal_cmd_send.gimbal_mode = GIMBAL_ZERO_FORCE;
        chassis_cmd_send.chassis_mode = CHASSIS_ZERO_FORCE;
        chassis_cmd_send.sync_belt_cmd = 0;
    }

    RobotCMDApplyChassisSpeedRamp();
    PubPushMessage(chassis_cmd_pub, (void *)&chassis_cmd_send);

    referee_data_for_ui = referee_data;
    {
        uint8_t ui_send_flag_rx = (uint8_t)(chassis_rs485_recv.UI_SendFlag & UI_SEND_FLAG_MASK);
        memcpy(&ui_cmd_send.ui_send_flag, &ui_send_flag_rx, sizeof(uint8_t));
    }
    memcpy(&ui_cmd_send.chassis_mode, &chassis_cmd_send.chassis_mode, sizeof(chassis_mode_e));
    memcpy(&ui_cmd_send.chassis_attitude_angle, &gimbal_fetch_data.yaw_motor_single_round_angle, sizeof(uint16_t));
    memcpy(&ui_cmd_send.friction_mode, &shoot_cmd_send.friction_mode, sizeof(friction_mode_e));
    memcpy(&ui_cmd_send.rune_mode, &auto_rune, sizeof(uint8_t));
    memcpy(&ui_cmd_send.SuperCap_mode, &chassis_cmd_send.SuperCap_flag_from_user, sizeof(uint8_t));
    memcpy(&ui_cmd_send.supercap_voltage, &chassis_fetch_data.cap_voltage, sizeof(float));
    memcpy(&ui_cmd_send.Chassis_Ctrl_power, &chassis_fetch_data.chassis_power_output, sizeof(float));
    memcpy(&ui_cmd_send.Chassis_power_limit, &referee_data->GameRobotState.chassis_power_limit, sizeof(uint16_t));
    memcpy(&ui_cmd_send.Shooter_heat, &referee_data->PowerHeatData.shooter_17mm_heat0, sizeof(uint16_t));
    memcpy(&ui_cmd_send.Heat_Limit, &referee_data->GameRobotState.shooter_id1_42mm_cooling_limit, sizeof(uint16_t));
    memcpy(&ui_cmd_send.cap_online_flag, &chassis_fetch_data.cap_online_flag, sizeof(uint8_t));
    memcpy(&ui_cmd_send.nuc_flag, &chassis_rs485_recv.nuc_mode, sizeof(uint8_t));
    memcpy(&ui_cmd_send.robot_level, &referee_data->GameRobotState.robot_level, sizeof(uint8_t));
    ui_cmd_send.sync_belt_state = CMDGetSyncBeltUiState();
    ui_cmd_send.vision_work_mode = RobotCMDInMouseKeyMode() ?
                                   RobotCMDGetVisionTxMode() :
                                   chassis_rs485_recv.vision_work_mode;
    ui_cmd_send.climb_mode = CMDChassisModeIsClimbUi(chassis_cmd_send.chassis_mode);
    PubPushMessage(ui_cmd_pub, (void *)&ui_cmd_send);
}
#endif

#if defined(GIMBAL_BOARD)
static void RobotCMDTaskGimbalBoard(void)
{
    
    FIFO_state = FIFO_Read_FrameCRC16(&NUC_fifo, fifo_pack, NUC_RECV_SIZE, 'C', 'B');
    if (FIFO_state)
        USB_Version_devode();
        

    if (FIFO_Read_chassis_ctrl(&rs485_master_fifo, chasssis_update_data, UNICOMM_UPLOAD_FRAME_LEN, UNICOMM_FRAME_HEAD, UNICOMM_FRAME_TYPE_UPLOAD) &&
        UniCommUnpackChassisUpload(chasssis_update_data, UNICOMM_UPLOAD_FRAME_LEN, &chassis_fetch_data_uart)) {
        if (chassis_fetch_data_uart.color)
            vision_send_data[8] = 1;
        else
            vision_send_data[8] = 0;
    }

    SubGetMessage(shoot_feed_sub, &shoot_fetch_data);
    SubGetMessage(gimbal_feed_sub, &gimbal_fetch_data);
    chassis_cmd_send_uart.reset_flag = 0;

    RobotCMDApplyControlInput();
    CalcOffsetAngle();
    if (RobotCMDInMouseKeyMode())
        ApplyKeyboardHeadTailSwitch();

    RobotCMDUpdateShootReferee(referee_data->GameRobotState.shooter_id1_42mm_cooling_rate,
                               referee_data->PowerHeatData.shooter_17mm_heat0,
                               referee_data->GameRobotState.shooter_id1_42mm_cooling_limit);

    RobotCMDApplyChassisSpeedRamp();
    chassis_cmd_send_uart.vx = chassis_cmd_send.vx;
    chassis_cmd_send_uart.vy = chassis_cmd_send.vy;
    chassis_cmd_send_uart.wz = chassis_cmd_send.wz;
    chassis_cmd_send_uart.offset_angle = chassis_cmd_send.offset_angle;
    chassis_cmd_send_uart.gimbal_error_angle = chassis_cmd_send.gimbal_error_angle;
    chassis_cmd_send_uart.leg_length_cmd = chassis_cmd_send.leg_length_cmd;
    chassis_cmd_send_uart.chassis_mode = chassis_cmd_send.chassis_mode;
    chassis_cmd_send_uart.sync_belt_cmd = chassis_cmd_send.sync_belt_cmd;
    chassis_cmd_send_uart.gimbal_mode = gimbal_cmd_send.gimbal_mode;
    chassis_cmd_send_uart.yaw_control = gimbal_cmd_send.yaw;

    chassis_cmd_send_uart.yaw_angle = gimbal_fetch_data.gimbal_imu_data->output.INS_angle_deg[INS_YAW_ADDRESS_OFFSET];//欧拉角输出
    chassis_cmd_send_uart.yaw_gyro = -gimbal_fetch_data.gimbal_imu_data->INS_data.INS_gyro[INS_YAW_ADDRESS_OFFSET];//

    chassis_cmd_send_uart.yaw_vel = vision_yaw_vel;    // 将视觉yaw速度通过RS485发送给下板
    chassis_cmd_send_uart.yaw_acc = vision_yaw_acc;    // 将视觉yaw加速度通过RS485发送给下板
    //chassis_cmd_send_uart.yaw_vel = 100;    // 将视觉yaw速度通过RS485发送给下板

    chassis_cmd_send_uart.nuc_yaw = gimbal_cmd_send.yaw_version;
    chassis_cmd_send_uart.shoot_mode = shoot_cmd_send.shoot_mode;
    chassis_cmd_send_uart.load_mode = shoot_cmd_send.load_mode;
    chassis_cmd_send_uart.shoot_count = shoot_count;
    chassis_cmd_send_uart.friction_mode = shoot_cmd_send.friction_mode;
    chassis_cmd_send_uart.nuc_mode = gimbal_cmd_send.nuc_mode;
    chassis_cmd_send_uart.vision_work_mode = RobotCMDGetVisionTxMode();
    chassis_cmd_send_uart.UI_SendFlag = (uint8_t)(UI_SendFlag & UI_SEND_FLAG_MASK);
    if (chassis_cmd_send.mecanum_force_enable) {
        chassis_cmd_send_uart.UI_SendFlag |= MECANUM_FORCE_UI_FLAG_BIT;
    }
    chassis_cmd_send_uart.superCap_flag = SuperCap_flag_from_user;
    chassis_ctrl_485(chassis_cmd_send_uart);

    PubPushMessage(gimbal_cmd_pub, (void *)&gimbal_cmd_send);

    vision_send_data[0] = 'C';
    vision_send_data[1] = 'B';
    vision_send_data[2] = RobotCMDGetVisionTxMode();

    {
        const float nuc_yaw_rep103   = gimbal_fetch_data.gimbal_imu_data->output.INS_angle[INS_YAW_ADDRESS_OFFSET];
        const float nuc_pitch_rep103 = gimbal_fetch_data.gimbal_imu_data->output.INS_angle[INS_PITCH_ADDRESS_OFFSET];
        const float nuc_roll_rep103  = gimbal_fetch_data.gimbal_imu_data->output.INS_angle[INS_ROLL_ADDRESS_OFFSET];

        EularAngleToQuaternion(nuc_yaw_rep103,   // Yaw, REP-103
                               nuc_pitch_rep103, // Pitch, REP-103
                               nuc_roll_rep103,  // Roll, REP-103
                           p);//输出四元数
    }
    memcpy(vision_send_data + 3, p, 16);

    {
        float yaw_reverse = gimbal_fetch_data.gimbal_imu_data->output.INS_angle[2];
        float_to_uint8_manual(yaw_reverse, vision_send_data + 19);
    }
    // memcpy(vision_send_data + 23, &gimbal_fetch_data.gimbal_imu_data->INS_data.INS_gyro[1], 8);
    memcpy(vision_send_data + 23, &gimbal_fetch_data.gimbal_imu_data->INS_data.INS_gyro[2], 8);//
    float_to_uint8_manual(gimbal_fetch_data.gimbal_imu_data->output.INS_angle[1], vision_send_data + 27);
    memcpy(vision_send_data + 31, &gimbal_pitch_vel_measure, 4);
    memcpy(&vision_send_data[35], &chassis_fetch_data_uart.initial_speed, 4);
    vision_send_data[39] = 0x0D;
    vision_send_data[40] = 0x00;
    Append_CRC16_Check_Sum(vision_send_data, NUC_SEND_SIZE);
    HostSend(host_instance, vision_send_data, NUC_SEND_SIZE);
}
#endif

#if defined(ONE_BOARD)
static void RobotCMDTaskOneBoard(void)
{
    if (FIFO_Read(&NUC_fifo, fifo_pack, NUC_RECV_SIZE, 0XFF, 0XFE))
        Version_devode();

    chassis_fetch_data = *(Chassis_Upload_Data_s *)CANCommGet(cmd_can_comm);
    SubGetMessage(shoot_feed_sub, &shoot_fetch_data);
    SubGetMessage(gimbal_feed_sub, &gimbal_fetch_data);
    SubGetMessage(chassis_feed_sub, (void *)&chassis_fetch_data);

    RobotCMDApplyControlInput();
    CalcOffsetAngle();
    if (RobotCMDInMouseKeyMode())
        ApplyKeyboardHeadTailSwitch();

    RobotCMDUpdateShootReferee(referee_data->GameRobotState.shooter_id1_17mm_cooling_rate,
                               referee_data->PowerHeatData.shooter_17mm_heat0,
                               referee_data->GameRobotState.shooter_id1_17mm_cooling_limit);
    PubPushMessage(shoot_cmd_pub, (void *)&shoot_cmd_send);

    memcpy(&chassis_cmd_send.chassis_power, &referee_data->PowerHeatData.chassis_power, sizeof(float));
    memcpy(&chassis_cmd_send.power_buffer, &referee_data->PowerHeatData.chassis_power_buffer, sizeof(uint16_t));
    memcpy(&chassis_cmd_send.level, &referee_data->GameRobotState.robot_level, sizeof(uint8_t));
    memcpy(&chassis_cmd_send.power_limit, &referee_data->GameRobotState.chassis_power_limit, sizeof(uint16_t));
    memcpy(&chassis_cmd_send.SuperCap_flag_from_user, &SuperCap_flag_from_user, sizeof(uint8_t));
    chassis_cmd_send.mecanum_force_enable = RobotCMDGetMecanumForceCtrl();
    RobotCMDApplyChassisSpeedRamp();
    PubPushMessage(chassis_cmd_pub, (void *)&chassis_cmd_send);
}
#endif

static void RobotCMDTaskDispatch(void)
{
#if defined(CHASSIS_BOARD)
    RobotCMDTaskChassisBoard();
#elif defined(GIMBAL_BOARD)
    RobotCMDTaskGimbalBoard();
#elif defined(ONE_BOARD)
    RobotCMDTaskOneBoard();
#endif
}

static void RobotCMDTaskPostProcess(void)
{
    HeatControl();

    if (shoot_cmd_send.load_mode == LOAD_STOP) {
        shoot_cmd_send.shoot_aim_angle = shoot_fetch_data.loader_angle - ONE_BULLET_DELTA_ANGLE;
        shoot_cmd_send.Shoot_Once_Flag = 1;
    } else if (shoot_cmd_send.load_mode == LOAD_1_BULLET || shoot_cmd_send.load_mode == LOAD_3_BULLET) {
        if (fabs(shoot_fetch_data.loader_angle - shoot_cmd_send.shoot_aim_angle) < 0.1)
            shoot_cmd_send.Shoot_Once_Flag = 0;
    }

    PubPushMessage(shoot_cmd_pub, (void *)&shoot_cmd_send);
    PubPushMessage(gimbal_cmd_pub, (void *)&gimbal_cmd_send);
}

void RobotCMDTask()
{
    RobotCMDTaskDispatch();
    RobotCMDTaskPostProcess();
}


