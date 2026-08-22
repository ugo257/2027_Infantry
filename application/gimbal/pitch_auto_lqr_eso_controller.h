#ifndef PITCH_AUTO_LQR_ESO_CONTROLLER_H
#define PITCH_AUTO_LQR_ESO_CONTROLLER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Axis model:
 *   J * theta_ddot + B * theta_dot + Fc * tanh(theta_dot / omega_eps)
 *       + G(theta) = torque_to_axis_gain * tau_motor + disturbance
 *
 * k_theta and k_omega are torque-domain LQR gains. The controller API keeps
 * the historical Auto name, but it is shared by manual and auto pitch modes.
 */
typedef struct {
    float j_kg_m2;
    float b_nms_rad;
    float k_theta_nm_rad;
    float k_omega_nms_rad;
    float torque_to_axis_gain;
    float tau_coulomb_nm;
    float coulomb_smooth_rad_s;
    float eso_bandwidth_rad_s;
    float eso_alpha;
    float eso_comp_gain;
    float eso_comp_limit_nm;
    float eso_omega_gate_rad_s;
    float eso_alpha_gate_rad_s2;
    float theta_deadband_rad;
    float torque_soft_limit_nm;
    float torque_min_nm;
    float torque_max_nm;
    float torque_slew_rate_nm_s;
    uint8_t viscous_feedforward_enable;
    uint8_t coulomb_feedforward_enable;
    uint8_t eso_enable;
    uint8_t eso_comp_enable;
    uint8_t torque_slew_enable;
} PitchAutoLqrEsoConfig_t;

typedef struct {
    float theta_rad;
    float omega_rad_s;
    float alpha_rad_s2;
    float tau_gravity_nm;
} PitchAutoLqrEsoReference_t;

typedef struct {
    float theta_rad;
    float omega_rad_s;
    /* Previous torque actually sent to the motor, not the candidate output. */
    float tau_applied_nm;
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
    float tau_cmd_last_nm;
    uint8_t feedback_ready;
} PitchAutoLqrEso_t;

typedef struct {
    float a[2][2];
    float b[2];
    float c[2];
} PitchAutoLqrEsoStateSpace_t;

typedef struct {
    float theta_ref_rad;
    float omega_ref_rad_s;
    float alpha_ref_rad_s2;
    float e_theta_rad;
    float e_omega_rad_s;
    float tau_feedback_axis_nm;
    float tau_inertia_axis_nm;
    float tau_viscous_axis_nm;
    float tau_coulomb_axis_nm;
    float tau_gravity_axis_nm;
    float tau_model_axis_nm;
    float tau_eso_raw_axis_nm;
    float tau_eso_active_axis_nm;
    float tau_pre_limit_motor_nm;
    float tau_cmd_before_slew_nm;
    float tau_cmd_nm;
    uint8_t output_valid;
    uint8_t timing_fault;
    uint8_t feedback_fault;
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
