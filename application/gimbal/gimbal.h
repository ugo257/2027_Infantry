#ifndef GIMBAL_H
#define GIMBAL_H

#include <stdint.h>

typedef enum {
    PITCH_LQR_STAGE_LEGACY = 0,
    PITCH_LQR_STAGE_SHADOW = 1,
    PITCH_LQR_STAGE_LOW_TORQUE = 2,
    PITCH_LQR_STAGE_GRAVITY = 3,
    /* Kept for numeric compatibility; friction remains in ESO disturbance d. */
    PITCH_LQR_STAGE_FULL_MODEL = 4,
    PITCH_LQR_STAGE_ESO_COMP = 5,
} PitchLqrStage_e;

/* Runtime commissioning knobs exposed to Ozone. */
extern volatile uint8_t g_pitch_lqr_stage;
extern volatile float g_pitch_lqr_j_kg_m2;
extern volatile float g_pitch_lqr_k_theta_nm_rad;
extern volatile float g_pitch_lqr_k_omega_nms_rad;
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
