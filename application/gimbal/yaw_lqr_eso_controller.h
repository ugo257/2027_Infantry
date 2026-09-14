#ifndef YAW_LQR_ESO_CONTROLLER_H
#define YAW_LQR_ESO_CONTROLLER_H

#include <stdint.h>

typedef struct {
    float inertia_kg_m2;
    float k_angle_nm_rad;
    float k_rate_nms_rad;
    float k_integral_nm_rad_s;
    float integral_limit_nm;
    float eso_bandwidth_rad_s;
    float eso_comp_gain;
    float eso_comp_limit_nm;
    float torque_to_current;
    float current_soft_limit;
    float current_min;
    float current_max;
    float current_slew_rate_s;
    uint8_t integral_enable;
    uint8_t eso_enable;
    uint8_t eso_comp_enable;
    uint8_t slew_enable;
} YawLqrEsoConfig_t;

typedef struct {
    float angle_rad;
    float rate_rad_s;
    float applied_current;
    uint8_t feedback_ok;
} YawLqrEsoFeedback_t;

typedef struct {
    float angle_rad;
    float rate_rad_s;
    float rate_ref_rad_s;
    float accel_ref_rad_s2;
    float current_injection;
} YawLqrEsoReference_t;

typedef struct {
    float z1;
    float z2;
    float z3;
    float w0;
    uint8_t feedback_ready;
    float integral_torque_nm;
} YawLqrEso_t;

typedef struct {
    float angle_ref_rad;
    float rate_ref_rad_s;
    float angle_error_rad;
    float rate_error_rad_s;
    float torque_feedback_nm;
    float torque_integral_nm;
    float torque_inertia_nm;
    float torque_eso_nm;
    float current_pre_limit;
    float current_cmd;
    uint8_t output_valid;
    uint8_t timing_fault;
    uint8_t feedback_fault;
    uint8_t eso_active;
    uint8_t integral_active;
    uint8_t limit_active;
    uint8_t slew_active;
} YawLqrEsoOutput_t;

void YawLqrEso_Init(YawLqrEso_t *ctrl);
void YawLqrEso_Reset(YawLqrEso_t *ctrl, float angle_rad, float rate_rad_s);
void YawLqrEso_Calc(YawLqrEso_t *ctrl,
                    const YawLqrEsoConfig_t *cfg,
                    const YawLqrEsoFeedback_t *feedback,
                    const YawLqrEsoReference_t *ref,
                    float dt_s,
                    YawLqrEsoOutput_t *output);

#endif
