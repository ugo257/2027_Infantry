#ifndef PITCH_AUTO_LQR_ESO_CONTROLLER_H
#define PITCH_AUTO_LQR_ESO_CONTROLLER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float j_kg_m2;
    float b_nms_rad;
    float k_theta;
    float k_omega;
    float k_i;
    float theta_integral_limit_rad_s;
    float tau_coulomb_nm;
    float coulomb_smooth_rad_s;
    float eso_bandwidth_rad_s;
    float eso_comp_gain;
    float eso_comp_limit_nm;
    float eso_omega_gate_rad_s;
    float eso_alpha_gate_rad_s2;
    float tau_bias_ki;
    float tau_bias_limit_nm;
    float tau_meas_lpf_alpha;
    float theta_deadband_rad;
    float torque_soft_limit_nm;
    float torque_min_nm;
    float torque_max_nm;
    float torque_slew_rate_nm_s;
    uint8_t eso_enable;
    uint8_t eso_comp_enable;
    uint8_t torque_slew_enable;
} PitchAutoLqrEsoConfig_t;

typedef struct {
    float theta_rad;
    float omega_rad_s;
    float alpha_rad_s2;
} PitchAutoLqrEsoReference_t;

typedef struct {
    float theta_rad;
    float omega_rad_s;
    float tau_meas_nm;
    uint8_t feedback_ok;
} PitchAutoLqrEsoFeedback_t;

typedef struct {
    float z1;
    float z2;
    float z3;
    float b0;
    float w0;
    float beta1;
    float beta2;
    float beta3;
} PitchAutoLqrEsoObserver_t;

typedef struct {
    PitchAutoLqrEsoObserver_t eso;
    float theta_integral_rad_s;
    float tau_bias_nm;
    float tau_meas_filt_nm;
    float tau_cmd_last_nm;
    uint8_t feedback_ready;
} PitchAutoLqrEso_t;

typedef struct {
    float a[2][2];
    float b[2][2];
    float c[2];
} PitchAutoLqrEsoStateSpace_t;

typedef struct {
    float theta_ref_rad;
    float omega_ref_rad_s;
    float alpha_ref_rad_s2;
    float e_theta_rad;
    float e_omega_rad_s;
    float tau_ff_alpha_nm;
    float tau_ff_viscous_nm;
    float tau_ff_coulomb_nm;
    float tau_ff_nm;
    float tau_i_nm;
    float tau_lqr_nm;
    float tau_eso_raw_nm;
    float tau_eso_active_nm;
    float tau_bias_nm;
    float tau_pre_limit_nm;
    float tau_cmd_before_slew_nm;
    float tau_cmd_nm;
    uint8_t eso_comp_active;
    uint8_t soft_limit_active;
    uint8_t hard_limit_active;
    uint8_t slew_limit_active;
} PitchAutoLqrEsoOutput_t;

void PitchAutoLqrEso_Init(PitchAutoLqrEso_t *ctrl);
void PitchAutoLqrEso_Reset(PitchAutoLqrEso_t *ctrl, float theta_rad, float omega_rad_s);
uint8_t PitchAutoLqrEso_GetStateSpace(const PitchAutoLqrEsoConfig_t *cfg,
                                      PitchAutoLqrEsoStateSpace_t *model);
void PitchAutoLqrEso_Calc(PitchAutoLqrEso_t *ctrl,
                          const PitchAutoLqrEsoConfig_t *cfg,
                          const PitchAutoLqrEsoFeedback_t *feedback,
                          const PitchAutoLqrEsoReference_t *ref,
                          float dt_s,
                          PitchAutoLqrEsoOutput_t *output);

#ifdef __cplusplus
}
#endif

#endif
