#ifndef GIMBAL_H
#define GIMBAL_H

#include <stdint.h>

typedef enum {
    PITCH_LQR_STAGE_LEGACY = 0,
    PITCH_LQR_STAGE_SHADOW = 1,
    PITCH_LQR_STAGE_LOW_TORQUE = 2,
    PITCH_LQR_STAGE_GRAVITY = 3,
    PITCH_LQR_STAGE_FULL_MODEL = 4,
    PITCH_LQR_STAGE_ESO_COMP = 5,
} PitchLqrStage_e;

/* Runtime commissioning knobs exposed to Ozone. */
extern volatile uint8_t g_pitch_lqr_stage;
extern volatile float g_pitch_lqr_j_kg_m2;
extern volatile float g_pitch_lqr_b_nms_rad;
extern volatile float g_pitch_lqr_k_theta_nm_rad;
extern volatile float g_pitch_lqr_k_omega_nms_rad;
extern volatile float g_pitch_lqr_coulomb_nm;
extern volatile float g_pitch_lqr_eso_alpha;
extern volatile float g_pitch_lqr_eso_w0_rad_s;
extern volatile float g_pitch_lqr_eso_comp_gain;

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
