#ifndef GIMBAL_H
#define GIMBAL_H

#include <stdint.h>

typedef enum {
    PITCH_LQR_STAGE_LEGACY = 0,
    PITCH_LQR_STAGE_SHADOW = 1,
    PITCH_LQR_STAGE_LOW_TORQUE = 2,
    PITCH_LQR_STAGE_GRAVITY = 3,
    /* Explicit friction feedforward is enabled at this stage; ESO still
     * estimates the remaining disturbance. */
    PITCH_LQR_STAGE_FULL_MODEL = 4,
    PITCH_LQR_STAGE_ESO_COMP = 5,
} PitchLqrStage_e;

typedef enum {
    YAW_LQR_STAGE_LEGACY = 0,
    YAW_LQR_STAGE_SHADOW = 1,
    YAW_LQR_STAGE_LOW_TORQUE = 2,
    YAW_LQR_STAGE_FULL_LQR = 3,
    YAW_LQR_STAGE_INTEGRAL = 4,
    YAW_LQR_STAGE_ESO_COMP = 5,
} YawLqrStage_e;

extern volatile uint8_t yaw_lqr_stage;
extern volatile float yaw_lqr_inertia_kg_m2;
extern volatile float yaw_lqr_k_angle_nm_rad;
extern volatile float yaw_lqr_k_rate_nms_rad;
extern volatile float yaw_lqr_k_integral_nm_rad_s;
extern volatile float yaw_lqr_integral_limit_nm;
extern volatile float yaw_lqr_eso_w0_rad_s;
extern volatile float yaw_lqr_eso_comp_gain;
extern volatile float yaw_lqr_torque_to_current;
extern volatile float yaw_lqr_current_slew_rate_s;
extern volatile float yaw_lqr_current_command_debug;
extern volatile float yaw_lqr_current_pre_limit_debug;
extern volatile float yaw_lqr_current_applied_debug;
/* Paired commissioning traces: angle is deg; rate is rad/s; torque is Nm.
 * torque_measure is inferred from GM6020 current using the provisional
 * yaw_lqr_torque_to_current conversion, not measured by a torque sensor. */
extern volatile float yaw_lqr_angle_ref_debug;
extern volatile float yaw_lqr_angle_measure_debug;
extern volatile float yaw_lqr_rate_ref_debug;
extern volatile float yaw_lqr_rate_measure_debug;
extern volatile float yaw_lqr_external_angle_debug;
extern volatile float yaw_lqr_motor_angle_debug;
extern volatile float yaw_lqr_motor_speed_debug;
extern volatile float yaw_lqr_motor_current_debug;
extern volatile float yaw_lqr_pid_output_debug;
extern volatile float yaw_lqr_angle_error_debug;
extern volatile float yaw_lqr_rate_error_debug;
extern volatile float yaw_lqr_torque_feedback_debug;
extern volatile float yaw_lqr_torque_command_debug;
extern volatile float yaw_lqr_torque_measure_debug;
extern volatile float yaw_lqr_torque_integral_debug;
extern volatile float yaw_lqr_torque_eso_debug;
extern volatile float yaw_lqr_eso_z3_debug;
extern volatile uint8_t yaw_lqr_output_valid_debug;
extern volatile uint8_t yaw_lqr_fallback_debug;
extern volatile uint8_t yaw_lqr_timing_fault_debug;
extern volatile uint8_t yaw_lqr_feedback_fault_debug;
extern volatile uint8_t yaw_lqr_current_saturation_debug;
extern volatile uint8_t yaw_lqr_current_slew_debug;
extern volatile uint8_t yaw_lqr_limit_debug;
extern volatile uint8_t yaw_lqr_active_debug;

/* Runtime commissioning knobs exposed to Ozone. */
extern volatile uint8_t g_pitch_lqr_stage;
extern volatile float g_pitch_lqr_j_kg_m2;
extern volatile float g_pitch_lqr_k_theta_nm_rad;
extern volatile float g_pitch_lqr_k_omega_nms_rad;
extern volatile uint8_t g_pitch_lqr_integral_enable;
extern volatile float g_pitch_lqr_k_integral_nm_rad_s;
extern volatile float g_pitch_lqr_integral_limit_nm;
extern volatile float g_pitch_lqr_eso_alpha;
extern volatile float g_pitch_lqr_eso_w0_rad_s;
extern volatile float g_pitch_lqr_eso_comp_gain;

/* Scalar motor telemetry for Ozone Data Sampling (avoid pointer expansion). */
extern volatile float pitch_motor_pos_debug;
extern volatile float pitch_motor_vel_debug;
extern volatile float pitch_motor_torque_feedback_debug;
extern volatile float pitch_motor_torque_command_debug;
extern volatile uint8_t pitch_motor_feedback_state_debug;

/* Manual remote Pitch target-velocity reference. */
extern volatile uint8_t g_pitch_remote_ref_vel_enable;
extern volatile float pitch_remote_ref_vel_debug;
extern volatile float pitch_remote_ref_vel_raw_debug;
extern volatile float pitch_remote_ref_vel_accel_debug;
extern volatile float pitch_remote_theta_cmd_debug;
extern volatile float pitch_auto_lqr_coulomb_model_debug;
extern volatile float pitch_auto_lqr_viscous_model_debug;
extern volatile float pitch_auto_lqr_tau_integral_debug;
extern volatile uint8_t pitch_auto_lqr_integral_active_debug;

void GimbalSetPitchRemoteRefVelEnable(uint8_t enable);
uint8_t GimbalGetPitchRemoteRefVelEnable(void);
extern volatile float pitch_gyro_raw_0_debug;
extern volatile float pitch_gyro_raw_1_debug;
extern volatile float pitch_gyro_raw_2_debug;

/**
 * @brief 初始化云台,会被RobotInit()调用
 * 
 */
void GimbalInit();

/**
 * @brief 云台任务
 * 
 */
void GimbalTask();



#endif // GIMBAL_H
