/*------------------------------------------------------------------------------*/
#include "stdio.h"//标准库
#include <math.h>
#include <string.h>
/*------------------------------------------------------------------------------*/
#include "gimbal.h"//拿到本模块对外接口 GimbalInit(),GimbalTask()      
#include "robot_board.h"//根据 CHASSIS_BOARD / ONE_BOARD 决定编译哪部分代码
#include "robot_params.h"//读云台几何参数、限位、控制宏定义等
#include "robot_types.h"//
#include "robot_def.h"//云台 SMC 开关与参数接口
#include "pitch_auto_lqr_eso_controller.h"
#include "yaw_lqr_eso_controller.h"
#include "yaw_identification_test.h"
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
static float filtered_yaw_acc = 0;                  //保存滤波后的视觉 yaw 加速度
static const float yaw_vel_filter_alpha = 0.12f;    //一阶低通滤波系数，数值越小滤波效果越明显，但响应越慢
static const float yaw_vel_reverse_alpha = 0.8f;   //视觉速度换向时加快滤波收敛，减少前馈拖尾
static float filtered_pitch_vel = 0.0f;
static float filtered_pitch_acc = 0.0f;
static float pitch_vision_torque_feedforward = 0.0f;
float pitch_vision_hold_feedforward = 0.0f;
float pitch_vision_hold_integral = 0.0f;
float pitch_gravity_feedforward_debug = 0.0f;
float pitch_motor_vel_damping_debug = 0.0f;
volatile uint8_t g_pitch_lqr_stage = GIMBAL_PITCH_LQR_DEFAULT_STAGE;
volatile float g_pitch_lqr_j_kg_m2 = GIMBAL_PITCH_AUTO_LQR_J;
volatile float g_pitch_lqr_k_theta_nm_rad = GIMBAL_PITCH_AUTO_LQR_K_THETA;
volatile float g_pitch_lqr_k_omega_nms_rad = GIMBAL_PITCH_AUTO_LQR_K_OMEGA;
volatile uint8_t g_pitch_lqr_integral_enable =
    GIMBAL_PITCH_AUTO_LQR_INTEGRAL_ENABLE_DEFAULT;
volatile float g_pitch_lqr_k_integral_nm_rad_s =
    GIMBAL_PITCH_AUTO_LQR_K_INTEGRAL;
volatile float g_pitch_lqr_integral_limit_nm =
    GIMBAL_PITCH_AUTO_LQR_INTEGRAL_LIMIT;
volatile float g_pitch_lqr_eso_alpha = GIMBAL_PITCH_AUTO_LQR_ESO_ALPHA;
volatile float g_pitch_lqr_eso_w0_rad_s = GIMBAL_PITCH_AUTO_LQR_ESO_W0;
volatile float g_pitch_lqr_eso_comp_gain = GIMBAL_PITCH_AUTO_LQR_ESO_COMP_GAIN;
volatile float pitch_motor_pos_debug = 0.0f;
volatile float pitch_motor_vel_debug = 0.0f;
volatile float pitch_motor_torque_feedback_debug = 0.0f;
volatile float pitch_motor_torque_command_debug = 0.0f;
volatile uint8_t pitch_motor_feedback_state_debug = 0u;
volatile uint8_t g_pitch_remote_ref_vel_enable =
    GIMBAL_PITCH_REMOTE_REF_VEL_ENABLE_DEFAULT;
volatile float pitch_remote_ref_vel_debug = 0.0f;
volatile float pitch_remote_ref_vel_raw_debug = 0.0f;
volatile float pitch_remote_ref_vel_accel_debug = 0.0f;
volatile float pitch_remote_theta_cmd_debug = 0.0f;
/* Raw calibrated INS gyro channels for pitch-axis mapping validation. */
volatile float pitch_gyro_raw_0_debug = 0.0f;
volatile float pitch_gyro_raw_1_debug = 0.0f;
volatile float pitch_gyro_raw_2_debug = 0.0f;
volatile float pitch_auto_lqr_tau_cmd_debug = 0.0f;
volatile float pitch_auto_lqr_tau_lqr_debug = 0.0f;
volatile float pitch_auto_lqr_tau_eso_debug = 0.0f;
volatile float pitch_auto_lqr_err_debug = 0.0f;
volatile float pitch_auto_lqr_omega_err_debug = 0.0f;
volatile float pitch_auto_lqr_ref_vel_debug = 0.0f;
volatile float pitch_auto_lqr_ref_acc_debug = 0.0f;
volatile float pitch_auto_lqr_theta_ref_debug = 0.0f;
volatile float pitch_auto_lqr_theta_meas_debug = 0.0f;
volatile float pitch_auto_lqr_omega_meas_debug = 0.0f;
volatile float pitch_auto_lqr_tau_applied_debug = 0.0f;
volatile float pitch_auto_lqr_tau_gravity_debug = 0.0f;
volatile float pitch_auto_lqr_tau_viscous_debug = 0.0f;
volatile float pitch_auto_lqr_tau_coulomb_debug = 0.0f;
volatile float pitch_auto_lqr_coulomb_model_debug = 0.0f;
volatile float pitch_auto_lqr_viscous_model_debug = 0.0f;
volatile float pitch_auto_lqr_tau_integral_debug = 0.0f;
volatile uint8_t pitch_auto_lqr_integral_active_debug = 0u;
volatile float pitch_auto_lqr_eso_z1_debug = 0.0f;
volatile float pitch_auto_lqr_eso_z2_debug = 0.0f;
volatile float pitch_auto_lqr_eso_z3_debug = 0.0f;
volatile float pitch_auto_lqr_eso_w0_debug = 0.0f;
volatile float pitch_auto_lqr_dt_debug = 0.0f;
volatile uint8_t pitch_auto_lqr_output_valid_debug = 0u;
volatile uint8_t pitch_auto_lqr_fallback_debug = 0u;
volatile uint8_t pitch_auto_lqr_timing_fault_debug = 0u;
volatile uint8_t pitch_auto_lqr_feedback_fault_debug = 0u;
volatile uint8_t pitch_auto_lqr_limit_debug = 0u;
static float pitch_vision_target_last = 0.0f;
static uint8_t pitch_vision_target_inited = 0u;
static uint16_t pitch_feedforward_clear_count = 0u;
static float pitch_remote_theta_cmd_last = 0.0f;
static float pitch_remote_ref_vel_filtered = 0.0f;
static uint8_t pitch_remote_ref_vel_inited = 0u;

static const float yaw_vel_deadzone = 0.05f;         //yaw 速度死区
extern float vision_yaw_vel;                        //视觉给出的 yaw 速度
extern float vision_yaw_acc;                        //视觉给出的 yaw 加速度
static float yaw_feedforward_vel_gain = -0.05f;      //定义 yaw 速度前馈增益
static const float yaw_feedforward_err_cutoff = 0.20f;
static const float yaw_feedforward_err_full = 3.0f;
static const float yaw_speed_feedforward_limit = 80.0f;
static const float yaw_target_jump_clear_threshold = 5.0f; //视觉目标跳变超过该角度时判定为换板
static const uint16_t yaw_feedforward_clear_cycles = 20u;  //换板后清前馈保持周期，1kHz下约20ms
static float yaw_vision_target_last = 0.0f;
static uint8_t yaw_vision_target_inited = 0u;
static uint16_t yaw_feedforward_clear_count = 0u;
const float pitch_offset = 7.7f;                    //作为 pitch 偏置量
float yaw_gyro_twoboard = 0, yaw_current_feedforward;  //yaw轴电流前馈
float pitch_current_feedforward, K_pitch_current_feedforward, B_pitch_current_feedforward;//俯仰轴电流前馈的 PID 参数
float pitch_tor_feedforward = 0;                    //保存 pitch 力矩前馈
float pitch_gyro_measure = 0;                       //保存 pitch 轴陀螺仪测量值
float gimbal_pitch_vel_measure = 0.0f;              // calibrated pitch gyro feedback (rad/s)
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

static float PitchRemoteRefVelDt(float dt_s)
{
    if (!isfinite(dt_s) || dt_s < 0.0001f || dt_s > 0.02f) {
        return 0.001f;
    }
    return dt_s;
}

static void PitchRemoteRefVelReset(float theta_cmd_rad)
{
    pitch_remote_theta_cmd_last = theta_cmd_rad;
    pitch_remote_ref_vel_filtered = 0.0f;
    pitch_remote_ref_vel_inited = 1u;
    pitch_remote_ref_vel_debug = 0.0f;
    pitch_remote_ref_vel_raw_debug = 0.0f;
    pitch_remote_ref_vel_accel_debug = 0.0f;
    pitch_remote_theta_cmd_debug = theta_cmd_rad;
}

static float PitchRemoteRefVelUpdate(float theta_cmd_rad, float dt_s)
{
    const float dt = PitchRemoteRefVelDt(dt_s);
    float raw_vel;
    float target_vel;
    float delta;
    float previous_vel;
    float slew_rate;

    if (!isfinite(theta_cmd_rad)) {
        PitchRemoteRefVelReset(0.0f);
        pitch_remote_ref_vel_inited = 0u;
        return 0.0f;
    }

    if (pitch_remote_ref_vel_inited == 0u) {
        PitchRemoteRefVelReset(theta_cmd_rad);
        return 0.0f;
    }

    raw_vel = (theta_cmd_rad - pitch_remote_theta_cmd_last) / dt;
    if (!isfinite(raw_vel)) {
        raw_vel = 0.0f;
    }
    raw_vel = clampf_local(raw_vel,
                           -GIMBAL_PITCH_REMOTE_REF_VEL_LIMIT_RAD_S,
                           GIMBAL_PITCH_REMOTE_REF_VEL_LIMIT_RAD_S);
    if (fabsf(raw_vel) <= GIMBAL_PITCH_REMOTE_REF_VEL_DEADBAND_RAD_S) {
        raw_vel = 0.0f;
    }

    target_vel = pitch_remote_ref_vel_filtered +
                 GIMBAL_PITCH_REMOTE_REF_VEL_LPF_ALPHA *
                 (raw_vel - pitch_remote_ref_vel_filtered);
    target_vel = clampf_local(target_vel,
                              -GIMBAL_PITCH_REMOTE_REF_VEL_LIMIT_RAD_S,
                              GIMBAL_PITCH_REMOTE_REF_VEL_LIMIT_RAD_S);

    previous_vel = pitch_remote_ref_vel_filtered;
    slew_rate = (target_vel * previous_vel < 0.0f) ?
                GIMBAL_PITCH_REMOTE_REF_VEL_REVERSE_SLEW_RAD_S2 :
                GIMBAL_PITCH_REMOTE_REF_VEL_SLEW_RAD_S2;
    delta = clampf_local(target_vel - previous_vel,
                         -slew_rate * dt,
                         slew_rate * dt);
    pitch_remote_ref_vel_filtered += delta;
    if (fabsf(pitch_remote_ref_vel_filtered) <=
        GIMBAL_PITCH_REMOTE_REF_VEL_DEADBAND_RAD_S) {
        pitch_remote_ref_vel_filtered = 0.0f;
    }

    pitch_remote_theta_cmd_last = theta_cmd_rad;
    pitch_remote_ref_vel_raw_debug = raw_vel;
    pitch_remote_ref_vel_debug = pitch_remote_ref_vel_filtered;
    pitch_remote_ref_vel_accel_debug =
        (pitch_remote_ref_vel_filtered - previous_vel) / dt;
    pitch_remote_theta_cmd_debug = theta_cmd_rad;
    return pitch_remote_ref_vel_filtered;
}

void GimbalSetPitchRemoteRefVelEnable(uint8_t enable)
{
    g_pitch_remote_ref_vel_enable = (enable != 0u) ? 1u : 0u;
    if (g_pitch_remote_ref_vel_enable == 0u) {
        PitchRemoteRefVelReset(pitch_remote_theta_cmd_debug);
    }
}

uint8_t GimbalGetPitchRemoteRefVelEnable(void)
{
    return g_pitch_remote_ref_vel_enable;
}

static float YawVisionFeedforwardScale(float yaw_error_deg)
{
    const float err_abs = fabsf(yaw_error_deg);

    if (err_abs <= yaw_feedforward_err_cutoff)
        return GIMBAL_YAW_VISION_FF_MIN_SCALE;
    if (err_abs >= yaw_feedforward_err_full)
        return 1.0f;

    return (err_abs - yaw_feedforward_err_cutoff) /
           (yaw_feedforward_err_full - yaw_feedforward_err_cutoff);
}

static float PitchGravityTorqueFeedforward(float pitch_angle_rad, float pitch_gyro_rads, float motor_pos_rad)
{
#if GIMBAL_PITCH_GRAVITY_USE_MOTOR_POS_FIT
    const float dx = motor_pos_rad - GIMBAL_PITCH_GRAVITY_POS_CENTER;
    float gravity_fit = GIMBAL_PITCH_GRAVITY_FIT_C0 +
                        GIMBAL_PITCH_GRAVITY_FIT_C1 * dx +
                        GIMBAL_PITCH_GRAVITY_FIT_C2 * dx * dx;
#if GIMBAL_PITCH_GRAVITY_LOCAL_CORR_ENABLE
    const float local_corr = clampf_local((motor_pos_rad - GIMBAL_PITCH_GRAVITY_LOCAL_CORR_CENTER) *
                                          GIMBAL_PITCH_GRAVITY_LOCAL_CORR_GAIN,
                                          GIMBAL_PITCH_GRAVITY_LOCAL_CORR_MIN,
                                          GIMBAL_PITCH_GRAVITY_LOCAL_CORR_MAX);
    gravity_fit += local_corr;
#endif
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

static float PitchMotorVelocityDamping(float motor_vel_rads)
{
    return clampf_local(-GIMBAL_PITCH_MOTOR_VEL_DAMPING_GAIN * motor_vel_rads,
                        -GIMBAL_PITCH_MOTOR_VEL_DAMPING_LIMIT,
                        GIMBAL_PITCH_MOTOR_VEL_DAMPING_LIMIT);
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
static PitchAutoLqrEso_t pitch_auto_lqr_eso;
static PitchAutoLqrEsoOutput_t pitch_auto_lqr_output;
static uint8_t pitch_auto_lqr_active = 0u;
static uint8_t pitch_lqr_direct_torque_active = 0u;
volatile uint8_t yaw_lqr_stage = GIMBAL_YAW_LQR_DEFAULT_STAGE;
volatile float yaw_lqr_inertia_kg_m2 = GIMBAL_YAW_LQR_INERTIA;
volatile float yaw_lqr_k_angle_nm_rad = GIMBAL_YAW_LQR_K_ANGLE;
volatile float yaw_lqr_k_rate_nms_rad = GIMBAL_YAW_LQR_K_RATE;
volatile float yaw_lqr_k_integral_nm_rad_s = GIMBAL_YAW_LQR_K_INTEGRAL;
volatile float yaw_lqr_integral_limit_nm = GIMBAL_YAW_LQR_INTEGRAL_LIMIT;
volatile float yaw_lqr_eso_w0_rad_s = GIMBAL_YAW_LQR_ESO_W0;
volatile float yaw_lqr_eso_comp_gain = GIMBAL_YAW_LQR_ESO_COMP_GAIN;
volatile float yaw_lqr_torque_to_current = GIMBAL_YAW_LQR_TORQUE_TO_CURRENT;
volatile float yaw_lqr_current_slew_rate_s = GIMBAL_YAW_LQR_CURRENT_SLEW_RATE;
volatile float yaw_lqr_current_command_debug = 0.0f;
volatile float yaw_lqr_current_pre_limit_debug = 0.0f;
volatile float yaw_lqr_current_applied_debug = 0.0f;
volatile float yaw_lqr_angle_ref_debug = 0.0f;
volatile float yaw_lqr_angle_measure_debug = 0.0f;
volatile float yaw_lqr_rate_ref_debug = 0.0f;
volatile float yaw_lqr_rate_measure_debug = 0.0f;
volatile float yaw_lqr_external_angle_debug = 0.0f;
volatile float yaw_lqr_motor_angle_debug = 0.0f;
volatile float yaw_lqr_motor_speed_debug = 0.0f;
volatile float yaw_lqr_motor_current_debug = 0.0f;
volatile float yaw_lqr_pid_output_debug = 0.0f;
volatile float yaw_lqr_angle_error_debug = 0.0f;
volatile float yaw_lqr_rate_error_debug = 0.0f;
volatile float yaw_lqr_torque_feedback_debug = 0.0f;
volatile float yaw_lqr_torque_command_debug = 0.0f;
volatile float yaw_lqr_torque_measure_debug = 0.0f;
volatile float yaw_lqr_torque_integral_debug = 0.0f;
volatile float yaw_lqr_torque_eso_debug = 0.0f;
volatile float yaw_lqr_eso_z3_debug = 0.0f;
volatile uint8_t yaw_lqr_output_valid_debug = 0u;
volatile uint8_t yaw_lqr_fallback_debug = 0u;
volatile uint8_t yaw_lqr_timing_fault_debug = 0u;
volatile uint8_t yaw_lqr_feedback_fault_debug = 0u;
volatile uint8_t yaw_lqr_current_saturation_debug = 0u;
volatile uint8_t yaw_lqr_current_slew_debug = 0u;
volatile uint8_t yaw_lqr_limit_debug = 0u;
volatile uint8_t yaw_lqr_active_debug = 0u;
static YawLqrEso_t yaw_lqr_eso;
static YawLqrEsoOutput_t yaw_lqr_output;
static uint8_t yaw_lqr_direct_current_active = 0u;
static uint8_t yaw_lqr_last_stage = YAW_LQR_STAGE_LEGACY;
static uint8_t pitch_zero_force_hold_active = 0u;
static float pitch_zero_force_hold_ref = 0.0f;

static const PitchAutoLqrEsoConfig_t pitch_auto_lqr_cfg = {
    .j_kg_m2 = GIMBAL_PITCH_AUTO_LQR_J,
    .k_theta_nm_rad = GIMBAL_PITCH_AUTO_LQR_K_THETA,
    .k_omega_nms_rad = GIMBAL_PITCH_AUTO_LQR_K_OMEGA,
    .torque_to_axis_gain = GIMBAL_PITCH_AUTO_LQR_TORQUE_AXIS_GAIN,
    .viscous_damping_nms_rad = GIMBAL_PITCH_AUTO_LQR_VISCOUS_B,
    .coulomb_torque_nm = GIMBAL_PITCH_AUTO_LQR_COULOMB_TORQUE,
    .coulomb_speed_smoothing_rad_s = GIMBAL_PITCH_AUTO_LQR_COULOMB_SPEED_SMOOTH,
    .coulomb_speed_deadband_rad_s = GIMBAL_PITCH_AUTO_LQR_COULOMB_SPEED_DEADBAND,
    .k_integral_nm_rad_s = GIMBAL_PITCH_AUTO_LQR_K_INTEGRAL,
    .integral_limit_nm = GIMBAL_PITCH_AUTO_LQR_INTEGRAL_LIMIT,
    .integral_ref_omega_gate_rad_s = GIMBAL_PITCH_AUTO_LQR_INTEGRAL_REF_OMEGA_GATE,
    .integral_meas_omega_gate_rad_s = GIMBAL_PITCH_AUTO_LQR_INTEGRAL_MEAS_OMEGA_GATE,
    .integral_error_gate_rad = GIMBAL_PITCH_AUTO_LQR_INTEGRAL_ERROR_GATE,
    .integral_leak_rate_s = GIMBAL_PITCH_AUTO_LQR_INTEGRAL_LEAK_RATE,
    .eso_bandwidth_rad_s = GIMBAL_PITCH_AUTO_LQR_ESO_W0,
    .eso_alpha = GIMBAL_PITCH_AUTO_LQR_ESO_ALPHA,
    .eso_comp_gain = GIMBAL_PITCH_AUTO_LQR_ESO_COMP_GAIN,
    .eso_comp_limit_nm = GIMBAL_PITCH_AUTO_LQR_ESO_COMP_LIMIT,
    .eso_omega_gate_rad_s = GIMBAL_PITCH_AUTO_LQR_ESO_OMEGA_GATE,
    .eso_alpha_gate_rad_s2 = GIMBAL_PITCH_AUTO_LQR_ESO_ALPHA_GATE,
    .theta_deadband_rad = GIMBAL_PITCH_AUTO_LQR_DEADBAND,
    .torque_soft_limit_nm = GIMBAL_PITCH_AUTO_LQR_SOFT_LIMIT,
    .torque_min_nm = GIMBAL_PITCH_AUTO_LQR_TORQUE_MIN,
    .torque_max_nm = GIMBAL_PITCH_AUTO_LQR_TORQUE_MAX,
    .torque_slew_rate_nm_s = GIMBAL_PITCH_AUTO_LQR_SLEW_RATE,
    .eso_enable = 1u,
    .eso_comp_enable = 0u,
    .torque_slew_enable = GIMBAL_PITCH_AUTO_LQR_SLEW_ENABLE,
    .integral_enable = GIMBAL_PITCH_AUTO_LQR_INTEGRAL_ENABLE_DEFAULT,
};

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

static void YawVisionFeedforwardReset(void)
{
    yaw_speed_feedforward = 0.0f;
    filtered_yaw_vel = 0.0f;
    filtered_yaw_acc = 0.0f;
    yaw_feedforward_clear_count = 0u;
    yaw_vision_target_last = 0.0f;
    yaw_vision_target_inited = 0u;
}

static float YawVisionCurrentFeedforward(float yaw_vel_deg_s, float yaw_acc_deg_s2)
{
#if GIMBAL_YAW_VISION_CTC_ENABLE
    float acc_limited = clampf_local(yaw_acc_deg_s2,
                                     -GIMBAL_YAW_VISION_ACC_LIMIT_DEG_S2,
                                     GIMBAL_YAW_VISION_ACC_LIMIT_DEG_S2);
    filtered_yaw_acc += GIMBAL_YAW_VISION_ACC_LPF_ALPHA * (acc_limited - filtered_yaw_acc);

    float current_target = GIMBAL_YAW_VISION_ACC_CURRENT_GAIN * filtered_yaw_acc +
                           GIMBAL_YAW_VISION_DAMP_CURRENT_GAIN * yaw_vel_deg_s;
    if (fabsf(yaw_vel_deg_s) > yaw_vel_deadzone) {
        current_target += (yaw_vel_deg_s > 0.0f ? -GIMBAL_YAW_VISION_COULOMB_CURRENT
                                                : GIMBAL_YAW_VISION_COULOMB_CURRENT);
    }
    current_target = clampf_local(current_target,
                                  -GIMBAL_YAW_VISION_CURRENT_LIMIT,
                                  GIMBAL_YAW_VISION_CURRENT_LIMIT);
    const float delta = clampf_local(current_target - yaw_current_feedforward,
                                     -GIMBAL_YAW_VISION_CURRENT_SLEW_STEP,
                                     GIMBAL_YAW_VISION_CURRENT_SLEW_STEP);
    return yaw_current_feedforward + delta;
#else
    (void)yaw_vel_deg_s;
    (void)yaw_acc_deg_s2;
    filtered_yaw_acc = 0.0f;
    return 0.0f;
#endif
}

static float PitchVisionFeedforwardScale(float pitch_error_rad)
{
    const float err_abs = fabsf(pitch_error_rad);

    if (err_abs <= GIMBAL_PITCH_VISION_ERR_CUTOFF_RAD)
        return GIMBAL_PITCH_VISION_FF_MIN_SCALE;
    if (err_abs >= GIMBAL_PITCH_VISION_ERR_FULL_RAD)
        return 1.0f;

    return GIMBAL_PITCH_VISION_FF_MIN_SCALE +
           (1.0f - GIMBAL_PITCH_VISION_FF_MIN_SCALE) *
           (err_abs - GIMBAL_PITCH_VISION_ERR_CUTOFF_RAD) /
           (GIMBAL_PITCH_VISION_ERR_FULL_RAD - GIMBAL_PITCH_VISION_ERR_CUTOFF_RAD);
}

static void PitchVisionFeedforwardReset(void)
{
    pitch_speed_feedforward = 0.0f;
    filtered_pitch_vel = 0.0f;
    filtered_pitch_acc = 0.0f;
    pitch_vision_torque_feedforward = 0.0f;
    pitch_vision_hold_feedforward = 0.0f;
    pitch_vision_hold_integral = 0.0f;
    pitch_feedforward_clear_count = 0u;
    pitch_vision_target_last = 0.0f;
    pitch_vision_target_inited = 0u;
}

static void PitchAutoLqrDebugClear(void)
{
    pitch_auto_lqr_tau_cmd_debug = 0.0f;
    pitch_auto_lqr_tau_lqr_debug = 0.0f;
    pitch_auto_lqr_tau_eso_debug = 0.0f;
    pitch_auto_lqr_err_debug = 0.0f;
    pitch_auto_lqr_omega_err_debug = 0.0f;
    pitch_auto_lqr_ref_vel_debug = 0.0f;
    pitch_auto_lqr_ref_acc_debug = 0.0f;
    pitch_auto_lqr_theta_ref_debug = 0.0f;
    pitch_auto_lqr_theta_meas_debug = 0.0f;
    pitch_auto_lqr_omega_meas_debug = 0.0f;
    pitch_auto_lqr_tau_applied_debug = 0.0f;
    pitch_auto_lqr_tau_gravity_debug = 0.0f;
    pitch_auto_lqr_tau_viscous_debug = 0.0f;
    pitch_auto_lqr_tau_coulomb_debug = 0.0f;
    pitch_auto_lqr_coulomb_model_debug = 0.0f;
    pitch_auto_lqr_viscous_model_debug = 0.0f;
    pitch_auto_lqr_tau_integral_debug = 0.0f;
    pitch_auto_lqr_integral_active_debug = 0u;
    pitch_auto_lqr_eso_z1_debug = 0.0f;
    pitch_auto_lqr_eso_z2_debug = 0.0f;
    pitch_auto_lqr_eso_z3_debug = 0.0f;
    pitch_auto_lqr_eso_w0_debug = 0.0f;
    pitch_auto_lqr_dt_debug = 0.0f;
    pitch_auto_lqr_output_valid_debug = 0u;
    pitch_auto_lqr_fallback_debug = 0u;
    pitch_auto_lqr_timing_fault_debug = 0u;
    pitch_auto_lqr_feedback_fault_debug = 0u;
    pitch_auto_lqr_limit_debug = 0u;
}

static float PitchAutoLqrMeasureOmega(float pitch_gyro_rad_s)
{
    return GIMBAL_PITCH_AUTO_LQR_MEAS_OMEGA_SIGN * pitch_gyro_rad_s;
}

static void PitchAutoLqrReset(float theta_rad, float omega_rad_s)
{
    PitchAutoLqrEso_Reset(&pitch_auto_lqr_eso,
                          theta_rad,
                          PitchAutoLqrMeasureOmega(omega_rad_s));
    memset(&pitch_auto_lqr_output, 0, sizeof(pitch_auto_lqr_output));
    pitch_auto_lqr_active = 0u;
    PitchAutoLqrDebugClear();
}

static void PitchAutoLqrNeutralizeCascadePid(void)
{
    pitch_motor->motor_controller.angle_PID.Iout = 0.0f;
    pitch_motor->motor_controller.angle_PID.Output = 0.0f;
    pitch_motor->motor_controller.angle_PID.Last_Output = 0.0f;
    pitch_motor->motor_controller.angle_PID.Last_Err = 0.0f;
    pitch_motor->motor_controller.speed_PID.Iout = 0.0f;
    pitch_motor->motor_controller.speed_PID.Output = 0.0f;
    pitch_motor->motor_controller.speed_PID.Last_Output = 0.0f;
    pitch_motor->motor_controller.speed_PID.Last_Err = 0.0f;
}

static void PitchLqrUseCascadeControl(void)
{
    if (pitch_lqr_direct_torque_active != 0u) {
        PitchAutoLqrNeutralizeCascadePid();
    }
    pitch_motor->motor_settings.angle_feedback_source = OTHER_FEED;
    pitch_motor->motor_settings.speed_feedback_source = OTHER_FEED;
    pitch_motor->motor_settings.outer_loop_type = ANGLE_LOOP;
    pitch_motor->motor_settings.close_loop_type = SPEED_LOOP | ANGLE_LOOP;
    pitch_motor->motor_settings.feedforward_flag = SPEED_FEEDFORWARD | CURRENT_FEEDFORWARD;
    pitch_lqr_direct_torque_active = 0u;
}

static void PitchLqrUseDirectTorque(float torque_nm)
{
    if (pitch_lqr_direct_torque_active == 0u) {
        PitchAutoLqrNeutralizeCascadePid();
    }
    pitch_motor->motor_settings.outer_loop_type = CURRENT_LOOP;
    pitch_motor->motor_settings.close_loop_type = CURRENT_LOOP;
    pitch_motor->motor_settings.feedforward_flag = FEEDFORWARD_NONE;
    pitch_motor->motor_controller.pid_ref = torque_nm;
    pitch_speed_feedforward = 0.0f;
    pitch_tor_feedforward = 0.0f;
    pitch_current_feedforward = 0.0f;
    pitch_lqr_direct_torque_active = 1u;
}

static float PitchAutoLqrCalcTorque(float pitch_ref_rad,
                                    float pitch_measure_rad,
                                    float pitch_gyro_rad_s,
                                    float pitch_ref_vel_rad_s,
                                    float pitch_ref_acc_rad_s2,
                                    float gravity_axis_nm,
                                    float dt_s,
                                    uint8_t stage)
{
    PitchAutoLqrEsoConfig_t cfg = pitch_auto_lqr_cfg;
    PitchAutoLqrEsoFeedback_t feedback;
    PitchAutoLqrEsoReference_t ref;
    const float pitch_omega_measure_rad_s = PitchAutoLqrMeasureOmega(pitch_gyro_rad_s);

    if (pitch_auto_lqr_active == 0u) {
        PitchAutoLqrEso_Reset(&pitch_auto_lqr_eso, pitch_measure_rad, pitch_omega_measure_rad_s);
        pitch_auto_lqr_active = 1u;
    }

    ref.theta_rad = pitch_ref_rad;
    ref.omega_rad_s = clampf_local(GIMBAL_PITCH_AUTO_LQR_REF_VEL_SIGN * pitch_ref_vel_rad_s,
                                   -GIMBAL_PITCH_AUTO_LQR_REF_VEL_LIMIT,
                                   GIMBAL_PITCH_AUTO_LQR_REF_VEL_LIMIT);
    ref.alpha_rad_s2 = clampf_local(GIMBAL_PITCH_AUTO_LQR_REF_ACC_SIGN * pitch_ref_acc_rad_s2,
                                    -GIMBAL_PITCH_AUTO_LQR_REF_ACC_LIMIT,
                                    GIMBAL_PITCH_AUTO_LQR_REF_ACC_LIMIT);
    ref.tau_gravity_nm = (stage >= PITCH_LQR_STAGE_GRAVITY) ? gravity_axis_nm : 0.0f;

    feedback.theta_rad = pitch_measure_rad;
    feedback.omega_rad_s = pitch_omega_measure_rad_s;
    feedback.tau_applied_nm = pitch_motor->ctrl.tor_set;
    feedback.feedback_ok = DMMotorIsOnline(pitch_motor);

    cfg.j_kg_m2 = g_pitch_lqr_j_kg_m2;
    cfg.k_theta_nm_rad = g_pitch_lqr_k_theta_nm_rad;
    cfg.k_omega_nms_rad = g_pitch_lqr_k_omega_nms_rad;
    cfg.k_integral_nm_rad_s = g_pitch_lqr_k_integral_nm_rad_s;
    cfg.integral_limit_nm = g_pitch_lqr_integral_limit_nm;
    cfg.integral_enable =
        (stage >= PITCH_LQR_STAGE_LOW_TORQUE &&
         g_pitch_lqr_integral_enable != 0u) ? 1u : 0u;
    cfg.eso_alpha = g_pitch_lqr_eso_alpha;
    cfg.eso_bandwidth_rad_s = g_pitch_lqr_eso_w0_rad_s;
    cfg.eso_comp_gain = g_pitch_lqr_eso_comp_gain;
    cfg.eso_enable = (stage >= PITCH_LQR_STAGE_SHADOW) ? 1u : 0u;
    cfg.eso_comp_enable = (stage >= PITCH_LQR_STAGE_ESO_COMP) ? 1u : 0u;
    /* Commissioning stages below FULL_MODEL deliberately retain the old
     * LQR behavior. Stage 4 is the first stage that adds the scan-derived
     * friction feedforward on top of the gravity model. */
    if (stage < PITCH_LQR_STAGE_FULL_MODEL) {
        cfg.viscous_damping_nms_rad = 0.0f;
        cfg.coulomb_torque_nm = 0.0f;
    }
    pitch_auto_lqr_coulomb_model_debug = cfg.coulomb_torque_nm;
    pitch_auto_lqr_viscous_model_debug = cfg.viscous_damping_nms_rad;
    cfg.torque_soft_limit_nm = (stage == PITCH_LQR_STAGE_LOW_TORQUE) ?
                               GIMBAL_PITCH_AUTO_LQR_LOW_TORQUE_LIMIT :
                               GIMBAL_PITCH_AUTO_LQR_SOFT_LIMIT;

    PitchAutoLqrEso_Calc(&pitch_auto_lqr_eso,
                         &cfg,
                         &feedback,
                         &ref,
                         dt_s,
                         &pitch_auto_lqr_output);

    pitch_auto_lqr_tau_cmd_debug = pitch_auto_lqr_output.tau_cmd_nm;
    pitch_auto_lqr_tau_lqr_debug = pitch_auto_lqr_output.tau_feedback_axis_nm;
    pitch_auto_lqr_tau_integral_debug = pitch_auto_lqr_output.tau_integral_axis_nm;
    pitch_auto_lqr_integral_active_debug = pitch_auto_lqr_output.integral_active;
    pitch_auto_lqr_tau_eso_debug = pitch_auto_lqr_output.tau_eso_active_axis_nm;
    pitch_auto_lqr_err_debug = pitch_auto_lqr_output.e_theta_rad;
    pitch_auto_lqr_omega_err_debug = pitch_auto_lqr_output.e_omega_rad_s;
    pitch_auto_lqr_ref_vel_debug = pitch_auto_lqr_output.omega_ref_rad_s;
    pitch_auto_lqr_ref_acc_debug = pitch_auto_lqr_output.alpha_ref_rad_s2;
    pitch_auto_lqr_theta_ref_debug = pitch_auto_lqr_output.theta_ref_rad;
    pitch_auto_lqr_theta_meas_debug = feedback.theta_rad;
    pitch_auto_lqr_omega_meas_debug = feedback.omega_rad_s;
    pitch_auto_lqr_tau_applied_debug = feedback.tau_applied_nm;
    pitch_auto_lqr_tau_gravity_debug = pitch_auto_lqr_output.tau_gravity_axis_nm;
    pitch_auto_lqr_tau_viscous_debug = pitch_auto_lqr_output.tau_viscous_axis_nm;
    pitch_auto_lqr_tau_coulomb_debug = pitch_auto_lqr_output.tau_coulomb_axis_nm;
    pitch_auto_lqr_eso_z1_debug = pitch_auto_lqr_eso.eso.z1;
    pitch_auto_lqr_eso_z2_debug = pitch_auto_lqr_eso.eso.z2;
    pitch_auto_lqr_eso_z3_debug = pitch_auto_lqr_eso.eso.z3;
    pitch_auto_lqr_eso_w0_debug = pitch_auto_lqr_eso.eso.w0;
    pitch_auto_lqr_dt_debug = dt_s;
    pitch_auto_lqr_output_valid_debug = pitch_auto_lqr_output.output_valid;
    pitch_auto_lqr_timing_fault_debug = pitch_auto_lqr_output.timing_fault;
    pitch_auto_lqr_feedback_fault_debug = pitch_auto_lqr_output.feedback_fault;
    pitch_auto_lqr_limit_debug = (uint8_t)(pitch_auto_lqr_output.soft_limit_active |
                                           pitch_auto_lqr_output.hard_limit_active |
                                           pitch_auto_lqr_output.slew_limit_active);

    return pitch_auto_lqr_output.tau_cmd_nm;
}

static void YawLqrNeutralizePid(void)
{
    yaw_motor->motor_controller.angle_PID.Iout = 0.0f;
    yaw_motor->motor_controller.angle_PID.Output = 0.0f;
    yaw_motor->motor_controller.angle_PID.Last_Output = 0.0f;
    yaw_motor->motor_controller.angle_PID.Last_Err = 0.0f;
    yaw_motor->motor_controller.speed_PID.Iout = 0.0f;
    yaw_motor->motor_controller.speed_PID.Output = 0.0f;
    yaw_motor->motor_controller.speed_PID.Last_Output = 0.0f;
    yaw_motor->motor_controller.speed_PID.Last_Err = 0.0f;
}

static float YawLqrMeasureRateRadS(float yaw_gyro_raw_rad_s)
{
    /* The gimbal board sends -INS_gyro over RS485. INS_gyro is already
     * rad/s, so only the established axis sign must be restored here. */
    return -yaw_gyro_raw_rad_s;
}

static void YawLqrUseLegacyPid(void)
{
    if (yaw_lqr_direct_current_active != 0u) {
        YawLqrNeutralizePid();
    }
    yaw_motor->motor_settings.angle_feedback_source = OTHER_FEED;
    yaw_motor->motor_settings.speed_feedback_source = OTHER_FEED;
    yaw_motor->motor_settings.outer_loop_type = ANGLE_LOOP;
    yaw_motor->motor_settings.close_loop_type = ANGLE_LOOP | SPEED_LOOP;
    yaw_motor->motor_settings.feedforward_flag = SPEED_FEEDFORWARD | CURRENT_FEEDFORWARD;
    yaw_motor->motor_settings.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    yaw_lqr_direct_current_active = 0u;
    yaw_lqr_active_debug = 0u;
}

static void YawLqrUseDirectCurrent(float current_command)
{
    if (yaw_lqr_direct_current_active == 0u) {
        YawLqrNeutralizePid();
    }
    yaw_motor->motor_settings.outer_loop_type = OPEN_LOOP;
    yaw_motor->motor_settings.close_loop_type = OPEN_LOOP;
    yaw_motor->motor_settings.feedforward_flag = FEEDFORWARD_NONE;
    yaw_motor->motor_settings.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    yaw_motor->motor_controller.pid_ref = current_command;
    yaw_speed_feedforward = 0.0f;
    yaw_current_feedforward = 0.0f;
    YawVisionFeedforwardReset();
    GimbalSMCReset(&yaw_smc_state);
    yaw_lqr_direct_current_active = 1u;
    yaw_lqr_active_debug = 1u;
}

static void YawLqrReset(float angle_deg, float yaw_gyro_raw_rad_s)
{
    const float rate_rad_s = YawLqrMeasureRateRadS(yaw_gyro_raw_rad_s);

    YawLqrEso_Reset(&yaw_lqr_eso,
                    angle_deg * DEGREE_2_RAD,
                    rate_rad_s);
    memset(&yaw_lqr_output, 0, sizeof(yaw_lqr_output));
    yaw_lqr_current_command_debug = 0.0f;
    yaw_lqr_current_pre_limit_debug = 0.0f;
    yaw_lqr_current_applied_debug = 0.0f;
    yaw_lqr_angle_measure_debug = angle_deg;
    yaw_lqr_rate_ref_debug = 0.0f;
    yaw_lqr_rate_measure_debug = rate_rad_s;
    yaw_lqr_angle_error_debug = 0.0f;
    yaw_lqr_rate_error_debug = 0.0f;
    yaw_lqr_torque_command_debug = 0.0f;
    yaw_lqr_eso_z3_debug = 0.0f;
    yaw_lqr_output_valid_debug = 0u;
    yaw_lqr_timing_fault_debug = 0u;
    yaw_lqr_feedback_fault_debug = 0u;
    yaw_lqr_current_saturation_debug = 0u;
    yaw_lqr_current_slew_debug = 0u;
    yaw_lqr_limit_debug = 0u;
    yaw_lqr_active_debug = 0u;
}

static float YawLqrCalculateCurrent(float angle_ref_deg,
                                    float angle_deg,
                                    float yaw_gyro_raw_rad_s,
                                    float dt_s,
                                    uint8_t stage)
{
    const float rate_measure_rad_s =
        YawLqrMeasureRateRadS(yaw_gyro_raw_rad_s);
    YawLqrEsoConfig_t cfg = {
        .inertia_kg_m2 = yaw_lqr_inertia_kg_m2,
        .k_angle_nm_rad = yaw_lqr_k_angle_nm_rad,
        .k_rate_nms_rad = yaw_lqr_k_rate_nms_rad,
        .k_integral_nm_rad_s = yaw_lqr_k_integral_nm_rad_s,
        .integral_limit_nm = yaw_lqr_integral_limit_nm,
        .eso_bandwidth_rad_s = yaw_lqr_eso_w0_rad_s,
        .eso_comp_gain = yaw_lqr_eso_comp_gain,
        .eso_comp_limit_nm = GIMBAL_YAW_LQR_ESO_COMP_LIMIT,
        .torque_to_current = yaw_lqr_torque_to_current,
        .current_soft_limit = (stage == YAW_LQR_STAGE_LOW_TORQUE) ?
                              GIMBAL_YAW_LQR_LOW_CURRENT_LIMIT :
                              GIMBAL_YAW_LQR_CURRENT_LIMIT,
        .current_min = -GIMBAL_YAW_LQR_CURRENT_LIMIT,
        .current_max = GIMBAL_YAW_LQR_CURRENT_LIMIT,
        .current_slew_rate_s = yaw_lqr_current_slew_rate_s,
        .integral_enable = (stage >= YAW_LQR_STAGE_INTEGRAL) ? 1u : 0u,
        .eso_enable = (stage >= YAW_LQR_STAGE_SHADOW) ? 1u : 0u,
        .eso_comp_enable = (stage >= YAW_LQR_STAGE_ESO_COMP) ? 1u : 0u,
        .slew_enable = 1u,
    };
    YawLqrEsoFeedback_t feedback = {
        .angle_rad = angle_deg * DEGREE_2_RAD,
        .rate_rad_s = rate_measure_rad_s,
        .applied_current = (yaw_lqr_direct_current_active != 0u) ?
                           yaw_lqr_current_command_debug :
                           (yaw_motor->motor_controller.speed_PID.Output + yaw_current_feedforward),
        .feedback_ok = DaemonIsOnline(yaw_motor->daemon),
    };
    const YawLqrEsoReference_t ref = {
        .angle_rad = angle_ref_deg * DEGREE_2_RAD,
        .rate_rad_s = 0.0f,
        .accel_ref_rad_s2 = 0.0f,
    };

    YawLqrEso_Calc(&yaw_lqr_eso, &cfg, &feedback, &ref, dt_s, &yaw_lqr_output);
    yaw_lqr_current_command_debug = yaw_lqr_output.current_cmd;
    yaw_lqr_current_pre_limit_debug = yaw_lqr_output.current_pre_limit;
    yaw_lqr_current_applied_debug = feedback.applied_current;
    yaw_lqr_angle_ref_debug = angle_ref_deg;
    yaw_lqr_angle_measure_debug = angle_deg;
    yaw_lqr_rate_ref_debug = yaw_lqr_output.rate_ref_rad_s;
    yaw_lqr_rate_measure_debug = rate_measure_rad_s;
    yaw_lqr_angle_error_debug = yaw_lqr_output.angle_error_rad;
    yaw_lqr_rate_error_debug = yaw_lqr_output.rate_error_rad_s;
    yaw_lqr_torque_feedback_debug = yaw_lqr_output.torque_feedback_nm;
    yaw_lqr_torque_command_debug =
        (fabsf(yaw_lqr_torque_to_current) > 1.0e-6f) ?
        yaw_lqr_output.current_cmd / yaw_lqr_torque_to_current : 0.0f;
    yaw_lqr_torque_integral_debug = yaw_lqr_output.torque_integral_nm;
    yaw_lqr_torque_eso_debug = yaw_lqr_output.torque_eso_nm;
    yaw_lqr_eso_z3_debug = yaw_lqr_eso.z3;
    yaw_lqr_output_valid_debug = yaw_lqr_output.output_valid;
    yaw_lqr_timing_fault_debug = yaw_lqr_output.timing_fault;
    yaw_lqr_feedback_fault_debug = yaw_lqr_output.feedback_fault;
    yaw_lqr_current_saturation_debug = yaw_lqr_output.limit_active;
    yaw_lqr_current_slew_debug = yaw_lqr_output.slew_active;
    yaw_lqr_limit_debug = (uint8_t)(yaw_lqr_output.limit_active |
                                    yaw_lqr_output.slew_active);
    return yaw_lqr_output.current_cmd;
}

static float PitchVisionHoldFeedforward(float pitch_error_rad,
                                        float pitch_gyro_rad_s,
                                        float pitch_vel_rad_s,
                                        float pitch_acc_rad_s2)
{
#if GIMBAL_PITCH_VISION_HOLD_ENABLE
    const uint8_t target_static = (fabsf(pitch_vel_rad_s) < GIMBAL_PITCH_VISION_HOLD_VEL_GATE &&
                                  fabsf(pitch_acc_rad_s2) < GIMBAL_PITCH_VISION_HOLD_ACC_GATE &&
                                  fabsf(pitch_gyro_rad_s) < GIMBAL_PITCH_VISION_HOLD_GYRO_GATE);
    const uint8_t pitch_stalled = (fabsf(pitch_error_rad) > GIMBAL_PITCH_VISION_HOLD_ERR_GATE &&
                                  fabsf(pitch_gyro_rad_s) < GIMBAL_PITCH_VISION_HOLD_GYRO_GATE);
    float hold_target = pitch_vision_hold_feedforward;

    if (target_static || pitch_stalled) {
        float err = pitch_error_rad;
        if (fabsf(err) < GIMBAL_PITCH_VISION_HOLD_ERR_DEADBAND) {
            err = 0.0f;
        }
        if (err * pitch_vision_hold_integral < 0.0f) {
            pitch_vision_hold_integral *= GIMBAL_PITCH_VISION_HOLD_REV_LEAK;
        }
        pitch_vision_hold_integral += GIMBAL_PITCH_VISION_HOLD_KI * err * GIMBAL_SMC_CTRL_DT;
        pitch_vision_hold_integral = clampf_local(pitch_vision_hold_integral,
                                                  -GIMBAL_PITCH_VISION_HOLD_I_LIMIT,
                                                  GIMBAL_PITCH_VISION_HOLD_I_LIMIT);
        hold_target = GIMBAL_PITCH_VISION_HOLD_KP * err + pitch_vision_hold_integral;
        hold_target = clampf_local(hold_target,
                                   -GIMBAL_PITCH_VISION_HOLD_LIMIT,
                                   GIMBAL_PITCH_VISION_HOLD_LIMIT);
    } else {
        pitch_vision_hold_integral *= GIMBAL_PITCH_VISION_HOLD_LEAK;
        if (fabsf(pitch_vision_hold_integral) < 0.001f) {
            pitch_vision_hold_integral = 0.0f;
        }
        hold_target *= GIMBAL_PITCH_VISION_HOLD_LEAK;
        if (fabsf(hold_target) < 0.001f) {
            hold_target = 0.0f;
        }
    }

    const float delta = clampf_local(hold_target - pitch_vision_hold_feedforward,
                                     -GIMBAL_PITCH_VISION_HOLD_SLEW_STEP,
                                     GIMBAL_PITCH_VISION_HOLD_SLEW_STEP);
    pitch_vision_hold_feedforward += delta;
    return pitch_vision_hold_feedforward;
#else
    (void)pitch_error_rad;
    (void)pitch_gyro_rad_s;
    (void)pitch_vel_rad_s;
    (void)pitch_acc_rad_s2;
    pitch_vision_hold_feedforward = 0.0f;
    pitch_vision_hold_integral = 0.0f;
    return 0.0f;
#endif
}

static float PitchVisionTorqueFeedforward(float pitch_vel_rad_s, float pitch_acc_rad_s2)
{
#if GIMBAL_PITCH_VISION_CTC_ENABLE
    float acc_limited = clampf_local(pitch_acc_rad_s2,
                                     -GIMBAL_PITCH_VISION_ACC_LIMIT_RAD_S2,
                                     GIMBAL_PITCH_VISION_ACC_LIMIT_RAD_S2);
    filtered_pitch_acc += GIMBAL_PITCH_VISION_ACC_LPF_ALPHA * (acc_limited - filtered_pitch_acc);

    float torque_target = GIMBAL_PITCH_VISION_ACC_TORQUE_GAIN * filtered_pitch_acc +
                          GIMBAL_PITCH_VISION_DAMP_TORQUE_GAIN * pitch_vel_rad_s;
    if (fabsf(pitch_vel_rad_s) > GIMBAL_PITCH_VISION_VEL_DEADBAND_RAD_S) {
        torque_target += (pitch_vel_rad_s > 0.0f ? GIMBAL_PITCH_VISION_COULOMB_TORQUE
                                                 : -GIMBAL_PITCH_VISION_COULOMB_TORQUE);
    }
    torque_target = clampf_local(torque_target,
                                 -GIMBAL_PITCH_VISION_TORQUE_LIMIT,
                                 GIMBAL_PITCH_VISION_TORQUE_LIMIT);
    const float delta = clampf_local(torque_target - pitch_vision_torque_feedforward,
                                     -GIMBAL_PITCH_VISION_TORQUE_SLEW_STEP,
                                     GIMBAL_PITCH_VISION_TORQUE_SLEW_STEP);
    return pitch_vision_torque_feedforward + delta;
#else
    (void)pitch_vel_rad_s;
    (void)pitch_acc_rad_s2;
    filtered_pitch_acc = 0.0f;
    return 0.0f;
#endif
}

static float PitchVisionSpeedFeedforward(float pitch_ref_rad,
                                         float pitch_measure_rad,
                                         float pitch_vel_rad_s,
                                         float pitch_acc_rad_s2)
{
    if (!pitch_vision_target_inited) {
        pitch_vision_target_last = pitch_ref_rad;
        pitch_vision_target_inited = 1u;
    } else {
        const float pitch_target_delta = pitch_ref_rad - pitch_vision_target_last;
        if (fabsf(pitch_target_delta) > GIMBAL_PITCH_VISION_TARGET_JUMP_RAD) {
            filtered_pitch_vel = 0.0f;
            filtered_pitch_acc = 0.0f;
            pitch_speed_feedforward = 0.0f;
            pitch_vision_torque_feedforward = 0.0f;
            pitch_vision_hold_feedforward = 0.0f;
            pitch_vision_hold_integral = 0.0f;
            pitch_feedforward_clear_count = GIMBAL_PITCH_VISION_CLEAR_CYCLES;
            GimbalSMCReset(&pitch_smc_state);
        }
        pitch_vision_target_last = pitch_ref_rad;
    }

    if (pitch_feedforward_clear_count > 0u) {
        pitch_feedforward_clear_count--;
        filtered_pitch_vel = 0.0f;
        filtered_pitch_acc = 0.0f;
        pitch_vision_torque_feedforward = 0.0f;
        pitch_vision_hold_feedforward = 0.0f;
        pitch_vision_hold_integral = 0.0f;
        return 0.0f;
    }

    const float vel_limited = clampf_local(GIMBAL_PITCH_VISION_DERIV_SIGN * pitch_vel_rad_s,
                                           -GIMBAL_PITCH_VISION_VEL_LIMIT_RAD_S,
                                           GIMBAL_PITCH_VISION_VEL_LIMIT_RAD_S);
    float vel_alpha = GIMBAL_PITCH_VISION_VEL_LPF_ALPHA;
    if (vel_limited * filtered_pitch_vel < 0.0f) {
        vel_alpha = GIMBAL_PITCH_VISION_VEL_REVERSE_ALPHA;
    }
    filtered_pitch_vel += vel_alpha * (vel_limited - filtered_pitch_vel);

    if (fabsf(filtered_pitch_vel) < GIMBAL_PITCH_VISION_VEL_DEADBAND_RAD_S) {
        filtered_pitch_vel = 0.0f;
    }

    const float scale = PitchVisionFeedforwardScale(pitch_ref_rad - pitch_measure_rad);
    const float speed_feedforward = clampf_local(filtered_pitch_vel *
                                                 GIMBAL_PITCH_VISION_VEL_FF_GAIN *
                                                 scale,
                                                 -GIMBAL_PITCH_VISION_VEL_LIMIT_RAD_S,
                                                 GIMBAL_PITCH_VISION_VEL_LIMIT_RAD_S);
    pitch_vision_torque_feedforward = PitchVisionTorqueFeedforward(speed_feedforward,
                                                                    GIMBAL_PITCH_VISION_DERIV_SIGN *
                                                                    pitch_acc_rad_s2 * scale);
    return speed_feedforward;
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
        YawLqrEso_Init(&yaw_lqr_eso);
        YawTest_Init();
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
                           PID_Derivative_On_Measurement | PID_DerivativeFilter |
                           PID_OutputFilter,
                .IntegralLimit = GIMBAL_PITCH_SPEED_INTEGRAL_LIMIT,
                .MaxOut = GIMBAL_PITCH_SPEED_MAXOUT,
                .Output_LPF_RC = GIMBAL_PITCH_SPEED_OUTPUT_FILTER,
                .Derivative_LPF_RC = GIMBAL_PITCH_SPEED_DERIVATIVE_FILTER,
            },
            //  .other_angle_feedback_ptr = &gimbal_IMU_data->output.INS_angle[INS_PITCH_ADDRESS_OFFSET], // pitch
            .other_angle_feedback_ptr = &gimbal_IMU_data->output.INS_angle[INS_PITCH_ADDRESS_OFFSET],     //pitch角度反馈:IMU 里的 pitch 弧度
            // ??????????????,????,ins_task.md??c??bodyframe?????
            .other_speed_feedback_ptr = &gimbal_pitch_vel_measure,
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
    PitchAutoLqrEso_Init(&pitch_auto_lqr_eso);
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
extern float vision_pitch_acc;
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
            YawLqrUseLegacyPid();
            DJIMotorStop(yaw_motor);          
            YawVisionFeedforwardReset();
            yaw_current_feedforward = 0.0f;
            GimbalSMCReset(&yaw_smc_state);
            YawLqrReset(*yaw_motor->motor_controller.other_angle_feedback_ptr,
                        *yaw_motor->motor_controller.other_speed_feedback_ptr);
            // DJIMotorStop(pitch_motor);
            break;
        // 视觉模式，yaw 轴使用视觉前馈，pitch 轴不使用前馈，完全由 PID 控制器根据 IMU 反馈来控制
        case GIMBAL_GYRO_MODE: { // 这个模式下 yaw 轴使用视觉前馈，pitch 轴不使用前馈，完全由 PID 控制器根据 IMU 反馈来控制
            uint8_t yaw_lqr_requested_stage = yaw_lqr_stage;
            DJIMotorEnable(yaw_motor);
            if (yaw_lqr_requested_stage > YAW_LQR_STAGE_ESO_COMP) {
                yaw_lqr_requested_stage = YAW_LQR_STAGE_LEGACY;
                yaw_lqr_stage = YAW_LQR_STAGE_LEGACY;
            }
            if (yaw_lqr_requested_stage != yaw_lqr_last_stage) {
                YawLqrNeutralizePid();
                YawLqrReset(*yaw_motor->motor_controller.other_angle_feedback_ptr,
                            *yaw_motor->motor_controller.other_speed_feedback_ptr);
                yaw_lqr_last_stage = yaw_lqr_requested_stage;
            }
            if (yaw_lqr_requested_stage < YAW_LQR_STAGE_LOW_TORQUE) {
                YawLqrUseLegacyPid();
            }
            float yaw_ref = gimbal_cmd_recv.yaw;
            if (gimbal_cmd_recv.nuc_mode == version_control)
            {
                float raw_yaw_vel = 0;
                float raw_yaw_acc = 0;
                float yaw_error = gimbal_cmd_recv.yaw_version - *yaw_motor->motor_controller.other_angle_feedback_ptr;
                float yaw_vel_alpha = yaw_vel_filter_alpha;
                // 根据编译选项选择 yaw 速度的来源，如果是 ONE_BOARD 或 GIMBAL_BOARD 就直接用视觉给出的 yaw 速度，如果是 CHASSIS_BOARD 就用串口收到的 yaw 速度
                #if defined(ONE_BOARD) || defined(GIMBAL_BOARD)
                raw_yaw_vel = vision_yaw_vel;//视觉给出的 yaw 速度，单位是度每秒
                raw_yaw_acc = vision_yaw_acc;//视觉给出的 yaw 加速度，单位是度每二次方秒
                #elif defined(CHASSIS_BOARD)//chassis_rs485_recv.yaw_gyro 就是串口收到的 yaw 速度，单位是度每秒
                    raw_yaw_vel = chassis_rs485_recv.yaw_vel;
                    raw_yaw_acc = chassis_rs485_recv.yaw_acc;
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

                if (!yaw_vision_target_inited) {
                    yaw_vision_target_last = gimbal_cmd_recv.yaw_version;
                    yaw_vision_target_inited = 1u;
                } else {
                    const float yaw_target_delta = GimbalWrapAngle180(gimbal_cmd_recv.yaw_version - yaw_vision_target_last);
                    if (fabsf(yaw_target_delta) > yaw_target_jump_clear_threshold) {
                        filtered_yaw_vel = 0.0f;
                        filtered_yaw_acc = 0.0f;
                        yaw_speed_feedforward = 0.0f;
                        yaw_current_feedforward = 0.0f;
                        yaw_feedforward_clear_count = yaw_feedforward_clear_cycles;
                        GimbalSMCReset(&yaw_smc_state);
                    }
                    yaw_vision_target_last = gimbal_cmd_recv.yaw_version;
                }

                if (yaw_feedforward_clear_count > 0u) {
                    yaw_feedforward_clear_count--;
                    filtered_yaw_vel = 0.0f;
                    filtered_yaw_acc = 0.0f;
                    yaw_speed_feedforward = 0.0f;
                    yaw_current_feedforward = 0.0f;
                } else {
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
                    yaw_current_feedforward = YawVisionCurrentFeedforward(filtered_yaw_vel, raw_yaw_acc);
                }

                yaw_ref = gimbal_cmd_recv.yaw_version;
                GimbalSMCReset(&yaw_smc_state);
                DJIMotorSetRef(yaw_motor, yaw_ref);
            }
            else
            {
                YawVisionFeedforwardReset();//如果不是自瞄模式，就不使用视觉前馈，yaw_speed_feedforward 置零
                //yaw_current_feedforward = 0;
                yaw_ref = gimbal_cmd_recv.yaw;
                DJIMotorSetRef(yaw_motor, yaw_ref);// yaw 角度参考直接来自命令，单位是度，云台会尽力把 yaw 轴转到这个角度
            }
            YawTest_Update(*yaw_motor->motor_controller.other_angle_feedback_ptr,
                           YawLqrMeasureRateRadS(
                               *yaw_motor->motor_controller.other_speed_feedback_ptr) *
                               RAD_2_DEGREE,
                           DaemonIsOnline(yaw_motor->daemon),
                           yaw_motor->dt,
                           (uint8_t)gimbal_cmd_recv.gimbal_mode);
            if (YawTest_IsActive() != 0u) {
                yaw_ref = YawTest_GetTarget();
                yaw_speed_feedforward = 0.0f;
                yaw_current_feedforward = 0.0f;
                YawVisionFeedforwardReset();
                GimbalSMCReset(&yaw_smc_state);
                DJIMotorSetRef(yaw_motor, yaw_ref);
            }
#if GIMBAL_YAW_SMC_ENABLE
            if (gimbal_cmd_recv.nuc_mode != version_control) {
                yaw_current_feedforward = GimbalSMCCalculate(&yaw_smc_state,
                                                             &yaw_smc_config,
                                                             yaw_ref,
                                                             *yaw_motor->motor_controller.other_angle_feedback_ptr,
                                                             (*yaw_motor->motor_controller.other_speed_feedback_ptr) * GIMBAL_YAW_SMC_GYRO_TO_DEG);
            }
#else
            if (gimbal_cmd_recv.nuc_mode != version_control) {
                yaw_current_feedforward = 0.0f;
            }
#endif
            if (YawTest_IsActive() != 0u) {
                /* The double-up test is intended to excite only the Yaw
                 * reference. Remove legacy visual/SMC feedforward after the
                 * normal PID preparation so the recorded model input is the
                 * selected PID or LQR path alone. */
                yaw_speed_feedforward = 0.0f;
                yaw_current_feedforward = 0.0f;
                YawVisionFeedforwardReset();
                GimbalSMCReset(&yaw_smc_state);
            }
            if (yaw_lqr_requested_stage >= YAW_LQR_STAGE_SHADOW) {
                const float yaw_lqr_current = YawLqrCalculateCurrent(
                    yaw_ref,
                    *yaw_motor->motor_controller.other_angle_feedback_ptr,
                    *yaw_motor->motor_controller.other_speed_feedback_ptr,
                    yaw_motor->dt,
                    yaw_lqr_requested_stage);
                if (yaw_lqr_requested_stage >= YAW_LQR_STAGE_LOW_TORQUE &&
                    yaw_lqr_output.output_valid != 0u) {
                    yaw_lqr_fallback_debug = 0u;
                    YawLqrUseDirectCurrent(yaw_lqr_current);
                } else {
                    yaw_lqr_fallback_debug =
                        (yaw_lqr_requested_stage >= YAW_LQR_STAGE_LOW_TORQUE) ? 1u : 0u;
                    YawLqrUseLegacyPid();
                    DJIMotorSetRef(yaw_motor, yaw_ref);
                }
            } else {
                yaw_lqr_fallback_debug = 0u;
                YawLqrReset(*yaw_motor->motor_controller.other_angle_feedback_ptr,
                            *yaw_motor->motor_controller.other_speed_feedback_ptr);
                /* In legacy mode the LQR reset must not make the diagnostic
                 * reference look identical to the measured angle. */
                yaw_lqr_angle_ref_debug = yaw_ref;
            }
            yaw_lqr_external_angle_debug =
                *yaw_motor->motor_controller.other_angle_feedback_ptr;
            yaw_lqr_motor_angle_debug =
                yaw_motor->measure.angle_single_round;
            yaw_lqr_motor_speed_debug = yaw_motor->measure.speed_aps;
            yaw_lqr_motor_current_debug = (float)yaw_motor->measure.real_current;
            yaw_lqr_torque_measure_debug =
                (fabsf(yaw_lqr_torque_to_current) > 1.0e-6f) ?
                yaw_lqr_motor_current_debug / yaw_lqr_torque_to_current : 0.0f;
            yaw_lqr_pid_output_debug = yaw_motor->motor_controller.speed_PID.Output;
            break;
        }
        case GIMBAL_MOTOR_MODE:
            YawLqrUseLegacyPid();
            DJIMotorEnable(yaw_motor);//电机使能
            DJIMotorChangeFeed(yaw_motor,ANGLE_LOOP,MOTOR_FEED);//把 yaw 轴的外环和内环的反馈源都切换成电机编码器，这样就不使用视觉前馈了，完全由电机自己根据编码器反馈来控制
            DJIMotorOuterLoop(yaw_motor, ANGLE_LOOP);//开启 yaw 轴的角度环控制
            DJIMotorSetRef(yaw_motor, gimbal_cmd_recv.yaw); // yaw 角度参考直接来自命令，单位是度，云台会尽力把 yaw 轴转到这个角度
            YawVisionFeedforwardReset();
            yaw_current_feedforward = 0.0f;
            GimbalSMCReset(&yaw_smc_state);
            YawLqrReset(*yaw_motor->motor_controller.other_angle_feedback_ptr,
                        *yaw_motor->motor_controller.other_speed_feedback_ptr);
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
    pitch_gyro_raw_0_debug = gimbal_IMU_data->INS_data.INS_gyro[0];
    pitch_gyro_raw_1_debug = gimbal_IMU_data->INS_data.INS_gyro[1];
    pitch_gyro_raw_2_debug = gimbal_IMU_data->INS_data.INS_gyro[2];
    pitch_gyro_measure =
        gimbal_IMU_data->INS_data.INS_gyro[INS_PITCH_GYRO_ADDRESS_OFFSET];
    gimbal_pitch_vel_measure = pitch_gyro_measure;
    pitch_motor_pos_debug = pitch_motor->measure.pos;
    pitch_motor_vel_debug = pitch_motor->measure.vel;
    pitch_motor_torque_feedback_debug = pitch_motor->measure.tor;
    pitch_motor_torque_command_debug = pitch_motor->ctrl.tor_set;
    pitch_motor_feedback_state_debug = (uint8_t)pitch_motor->measure.state;
    const float pitch_omega_measure =
        PitchAutoLqrMeasureOmega(pitch_gyro_measure);
    const float pitch_gravity_model = PitchGravityTorqueFeedforward(pitch_angle_measure,
                                                                    0.0f,
                                                                    pitch_motor->measure.pos);
    const float pitch_gravity_feedforward = PitchGravityTorqueFeedforward(pitch_angle_measure,
                                                                           pitch_gyro_measure,
                                                                           pitch_motor->measure.pos);
    const float pitch_motor_vel_damping = PitchMotorVelocityDamping(pitch_motor->measure.vel);
    pitch_gravity_feedforward_debug = pitch_gravity_feedforward;
    pitch_motor_vel_damping_debug = pitch_motor_vel_damping;
    pitch_tor_feedforward = clampf_local(pitch_gravity_feedforward + pitch_motor_vel_damping,
                                         GIMBAL_PITCH_FF_TOTAL_MIN,
                                         GIMBAL_PITCH_FF_TOTAL_MAX);
    pitch_current_feedforward = pitch_tor_feedforward;
    switch (gimbal_cmd_recv.gimbal_mode) {
        //停掉 DM pitch 电机，并把 angle/speed 两个 PID 的积分项清零，同时关掉速度前馈，防止重新使能时积分残留
        case GIMBAL_ZERO_FORCE:
            PitchLqrUseCascadeControl();
            PitchRemoteRefVelReset(pitch_angle_measure);
#if GIMBAL_PITCH_ZERO_FORCE_HOLD_ENABLE
            DMMotorEnable1(pitch_motor);
            if (!pitch_zero_force_hold_active) {
                pitch_zero_force_hold_ref = clampf_local(pitch_angle_measure,
                                                         PITCH_DOWN_LIMIT,
                                                         PITCH_UP_LIMIT);
                pitch_motor->motor_controller.speed_PID.Iout = 0;
                pitch_motor->motor_controller.angle_PID.Iout = 0;
                GimbalSMCReset(&pitch_smc_state);
                PitchAutoLqrReset(pitch_angle_measure, pitch_gyro_measure);
                pitch_zero_force_hold_active = 1u;
            }
            pitch_motor->motor_controller.pid_ref = pitch_zero_force_hold_ref;
            pitch_speed_feedforward = 0;
            PitchVisionFeedforwardReset();
            {
                const float hold_err = pitch_zero_force_hold_ref - pitch_angle_measure;
                float hold_smc_feedforward = 0.0f;

                if (fabsf(hold_err) < GIMBAL_PITCH_SMC_LINKAGE_ERR_GATE &&
                    fabsf(pitch_gyro_measure) < GIMBAL_PITCH_SMC_LINKAGE_GYRO_GATE) {
                    GimbalSMCReset(&pitch_smc_state);
                } else {
                    hold_smc_feedforward = GimbalSMCCalculate(&pitch_smc_state,
                                                              &pitch_smc_config,
                                                              pitch_zero_force_hold_ref,
                                                              pitch_angle_measure,
                                                              pitch_gyro_measure);
                    hold_smc_feedforward *= PitchLinkageSmcScale(pitch_motor->measure.pos);
                    hold_smc_feedforward *= GIMBAL_PITCH_ZERO_FORCE_SMC_GAIN;
                }

                pitch_tor_feedforward = clampf_local(pitch_gravity_feedforward +
                                                     pitch_motor_vel_damping +
                                                     hold_smc_feedforward,
                                                     GIMBAL_PITCH_FF_TOTAL_MIN,
                                                     GIMBAL_PITCH_FF_TOTAL_MAX);
            }
            pitch_current_feedforward = pitch_tor_feedforward;
#else
            DMMotorStop(pitch_motor);
            pitch_motor->motor_controller.speed_PID.Iout = 0;
            pitch_motor->motor_controller.angle_PID.Iout = 0;
            pitch_speed_feedforward = 0;
            pitch_tor_feedforward = 0;
            pitch_current_feedforward = 0;
            PitchVisionFeedforwardReset();
            PitchAutoLqrReset(pitch_angle_measure, pitch_gyro_measure);
            GimbalSMCReset(&pitch_smc_state);
#endif
            break;
        case GIMBAL_GYRO_MODE://使能 DM pitch 电机，切换到角度环和速度环都使用 IMU 反馈的模式，开启角度环控制
        {
#if GIMBAL_PITCH_AUTO_LQR_ESO_ENABLE
            uint8_t pitch_lqr_stage = g_pitch_lqr_stage;
            float pitch_lqr_ref_vel = 0.0f;
            float pitch_lqr_ref_acc = 0.0f;
            float pitch_lqr_torque = 0.0f;
#endif
            pitch_zero_force_hold_active = 0u;
            DMMotorEnable1(pitch_motor);

            if (gimbal_cmd_recv.nuc_mode == version_control)//自瞄模式，pitch 角度参考来自命令，并且使用一个单独的前馈值 pitch_version 来进行前馈补偿，这个前馈值可以是一个拟合函数的输出，也可以是一个滤波后的视觉速度乘以一个增益
            {
                pitch_limit(gimbal_cmd_recv.pitch_version);//对 pitch 角度参考进行限幅，确保它在安全范围内
#if GIMBAL_PITCH_AUTO_LQR_ESO_ENABLE
                PitchRemoteRefVelReset(pitch_motor->motor_controller.pid_ref);
                pitch_lqr_ref_vel = pitch_vel;
                pitch_lqr_ref_acc = vision_pitch_acc;
#endif
            }
            else
            {
                pitch_limit(gimbal_cmd_recv.pitch);//对 pitch 角度参考进行限幅，确保它在安全范围内
                pitch_speed_feedforward = 0;//如果不是自瞄模式，就不使用视觉前馈，pitch_speed_feedforward 置零
                PitchVisionFeedforwardReset();
#if GIMBAL_PITCH_AUTO_LQR_ESO_ENABLE
                if (g_pitch_remote_ref_vel_enable != 0u) {
                    pitch_lqr_ref_vel = PitchRemoteRefVelUpdate(
                        pitch_motor->motor_controller.pid_ref,
                        pitch_motor->dt);
                } else {
                    PitchRemoteRefVelReset(pitch_motor->motor_controller.pid_ref);
                    pitch_lqr_ref_vel = 0.0f;
                }
#endif
            }
#if GIMBAL_PITCH_AUTO_LQR_ESO_ENABLE
            if (pitch_lqr_stage > PITCH_LQR_STAGE_ESO_COMP) {
                pitch_lqr_stage = PITCH_LQR_STAGE_LEGACY;
            }
            if (pitch_lqr_stage >= PITCH_LQR_STAGE_SHADOW) {
                const float pitch_plan_ref = pitch_motor->motor_controller.pid_ref;

                pitch_lqr_torque = PitchAutoLqrCalcTorque(pitch_plan_ref,
                                                           pitch_angle_measure,
                                                           pitch_gyro_measure,
                                                           pitch_lqr_ref_vel,
                                                           pitch_lqr_ref_acc,
                                                           pitch_gravity_model,
                                                           pitch_motor->dt,
                                                           pitch_lqr_stage);
                if (pitch_lqr_stage >= PITCH_LQR_STAGE_LOW_TORQUE &&
                    pitch_auto_lqr_output.output_valid != 0u) {
                    PitchVisionFeedforwardReset();
                    GimbalSMCReset(&pitch_smc_state);
                    pitch_auto_lqr_fallback_debug = 0u;
                    PitchLqrUseDirectTorque(pitch_lqr_torque);
                    break;
                }
                pitch_auto_lqr_fallback_debug =
                    (pitch_lqr_stage >= PITCH_LQR_STAGE_LOW_TORQUE) ? 1u : 0u;
                PitchLqrUseCascadeControl();
                pitch_motor->motor_controller.pid_ref = pitch_plan_ref;
            } else {
                PitchLqrUseCascadeControl();
                PitchAutoLqrReset(pitch_angle_measure, pitch_gyro_measure);
            }
#else
            PitchLqrUseCascadeControl();
#endif
            if (gimbal_cmd_recv.nuc_mode == version_control) {
                pitch_speed_feedforward = PitchVisionSpeedFeedforward(
                    pitch_motor->motor_controller.pid_ref,
                    pitch_angle_measure,
                    pitch_vel,
                    vision_pitch_acc);
            }
#if GIMBAL_PITCH_SMC_ENABLE
            {
                float pitch_smc_feedforward = 0.0f;
                float pitch_vision_hold_ff = 0.0f;
                const float pitch_vision_ff = (gimbal_cmd_recv.nuc_mode == version_control) ?
                                              pitch_vision_torque_feedforward : 0.0f;
                float pitch_smc_err = pitch_motor->motor_controller.pid_ref - pitch_angle_measure;
                if (gimbal_cmd_recv.nuc_mode == version_control) {
                    pitch_vision_hold_ff = PitchVisionHoldFeedforward(pitch_smc_err,
                                                                      pitch_gyro_measure,
                                                                      pitch_vel,
                                                                      vision_pitch_acc);
                }
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
                pitch_tor_feedforward = clampf_local(pitch_gravity_feedforward +
                                                     pitch_motor_vel_damping +
                                                     pitch_smc_feedforward +
                                                     pitch_vision_ff +
                                                     pitch_vision_hold_ff,
                                                     GIMBAL_PITCH_FF_TOTAL_MIN,
                                                     GIMBAL_PITCH_FF_TOTAL_MAX);
                pitch_current_feedforward = pitch_tor_feedforward;
            }
#else
            {
                float pitch_vision_hold_ff = 0.0f;
                const float pitch_vision_ff = (gimbal_cmd_recv.nuc_mode == version_control) ?
                                              pitch_vision_torque_feedforward : 0.0f;
                if (gimbal_cmd_recv.nuc_mode == version_control) {
                    pitch_vision_hold_ff = PitchVisionHoldFeedforward(pitch_motor->motor_controller.pid_ref - pitch_angle_measure,
                                                                      pitch_gyro_measure,
                                                                      pitch_vel,
                                                                      vision_pitch_acc);
                }
                pitch_tor_feedforward = clampf_local(pitch_gravity_feedforward +
                                                     pitch_motor_vel_damping +
                                                     pitch_vision_ff +
                                                     pitch_vision_hold_ff,
                                                     GIMBAL_PITCH_FF_TOTAL_MIN,
                                                     GIMBAL_PITCH_FF_TOTAL_MAX);
                pitch_current_feedforward = pitch_tor_feedforward;
            }
#endif
            // DJIMotorSetRef(fpv_pitch_motor,fpv_pitch_test);
            // DJIMotorSetRef(telescope_motor,telescope_test);
           
        
            break;
        }
        case GIMBAL_MOTOR_MODE://使能 DM pitch 电机，切换到角度环和速度环都使用电机编码器反馈的模式，开启角度环控制，pitch 角度参考直接来自命令
            pitch_zero_force_hold_active = 0u;
            PitchLqrUseCascadeControl();
            PitchRemoteRefVelReset(pitch_angle_measure);
            DMMotorEnable1(pitch_motor);
            // DJIMotorOuterLoop(pitch_motor, ANGLE_LOOP);
            // DJIMotorChangeFeed(pitch_motor,ANGLE_LOOP,MOTOR_FEED);
            // DJIMotorChangeFeed(pitch_motor,SPEED_LOOP,MOTOR_FEED);
       
            DMMotorSetRef(pitch_motor, pitch_offset + gimbal_cmd_recv.pitch); //对 pitch 角度参考加上一个固定的偏置，确保它在一个合理的范围内，避免过度转动导致损坏，同时也可以根据实际情况调整这个偏置量
            // DJIMotorSetRef(pitch_motor, pitch_test); 
            pitch_speed_feedforward = 0;
            PitchVisionFeedforwardReset();
            pitch_tor_feedforward = clampf_local(pitch_gravity_feedforward + pitch_motor_vel_damping,
                                                 GIMBAL_PITCH_FF_TOTAL_MIN,
                                                 GIMBAL_PITCH_FF_TOTAL_MAX);
            pitch_current_feedforward = pitch_tor_feedforward;
            PitchAutoLqrReset(pitch_angle_measure, pitch_gyro_measure);
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





