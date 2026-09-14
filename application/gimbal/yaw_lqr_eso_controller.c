#include "yaw_lqr_eso_controller.h"
#include <math.h>
#include <string.h>

#define YAW_LQR_MIN_DT_S 1.0e-4f
#define YAW_LQR_MAX_DT_S 0.02f
#define YAW_LQR_PI       3.14159265358979323846f
#define YAW_LQR_TWO_PI   (2.0f * YAW_LQR_PI)

static float yaw_lqr_abs(float x) { return x < 0.0f ? -x : x; }
static float yaw_lqr_clamp(float x, float lo, float hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}
static uint8_t yaw_lqr_finite(float x) { return isfinite(x) ? 1u : 0u; }

static float yaw_lqr_wrap_pi(float angle_rad)
{
    while (angle_rad > YAW_LQR_PI) angle_rad -= YAW_LQR_TWO_PI;
    while (angle_rad < -YAW_LQR_PI) angle_rad += YAW_LQR_TWO_PI;
    return angle_rad;
}

void YawLqrEso_Init(YawLqrEso_t *ctrl)
{
    if (ctrl != NULL) memset(ctrl, 0, sizeof(*ctrl));
}

void YawLqrEso_Reset(YawLqrEso_t *ctrl, float angle_rad, float rate_rad_s)
{
    if (ctrl == NULL) return;
    memset(ctrl, 0, sizeof(*ctrl));
    ctrl->z1 = angle_rad;
    ctrl->z2 = rate_rad_s;
}

void YawLqrEso_Calc(YawLqrEso_t *ctrl,
                    const YawLqrEsoConfig_t *cfg,
                    const YawLqrEsoFeedback_t *feedback,
                    const YawLqrEsoReference_t *ref,
                    float dt_s,
                    YawLqrEsoOutput_t *output)
{
    YawLqrEsoOutput_t out;
    float torque_nm;
    float current;
    memset(&out, 0, sizeof(out));
    if (ctrl == NULL || cfg == NULL || feedback == NULL || ref == NULL) {
        if (output != NULL) *output = out;
        return;
    }
    out.angle_ref_rad = ref->angle_rad;
    out.rate_ref_rad_s = ref->rate_rad_s;
    if (!yaw_lqr_finite(cfg->inertia_kg_m2) ||
        !yaw_lqr_finite(cfg->k_angle_nm_rad) ||
        !yaw_lqr_finite(cfg->k_rate_nms_rad) ||
        !yaw_lqr_finite(cfg->torque_to_current) ||
        cfg->inertia_kg_m2 <= 1.0e-6f ||
        yaw_lqr_abs(cfg->torque_to_current) <= 1.0e-6f ||
        !yaw_lqr_finite(feedback->angle_rad) || !yaw_lqr_finite(feedback->rate_rad_s) ||
        !yaw_lqr_finite(feedback->applied_current) ||
        !yaw_lqr_finite(ref->angle_rad) || !yaw_lqr_finite(ref->rate_rad_s) ||
        !yaw_lqr_finite(ref->accel_ref_rad_s2) ||
        !yaw_lqr_finite(ref->current_injection) || !feedback->feedback_ok) {
        out.feedback_fault = 1u;
        YawLqrEso_Reset(ctrl, feedback->angle_rad, feedback->rate_rad_s);
        if (output != NULL) *output = out;
        return;
    }
    if (!yaw_lqr_finite(dt_s) || dt_s < YAW_LQR_MIN_DT_S || dt_s > YAW_LQR_MAX_DT_S) {
        out.timing_fault = 1u;
        ctrl->feedback_ready = 0u;
        if (output != NULL) *output = out;
        return;
    }
    if (ctrl->feedback_ready == 0u) {
        ctrl->z1 = feedback->angle_rad;
        ctrl->z2 = feedback->rate_rad_s;
        ctrl->z3 = 0.0f;
        ctrl->feedback_ready = 1u;
    } else if (cfg->eso_enable != 0u && cfg->eso_bandwidth_rad_s > 0.0f) {
        const float w0 = yaw_lqr_clamp(cfg->eso_bandwidth_rad_s, 1.0f, 500.0f);
        const float e = ctrl->z1 - feedback->angle_rad;
        const float input_accel = feedback->applied_current /
                                  (cfg->torque_to_current * cfg->inertia_kg_m2);
        ctrl->z1 += dt_s * (ctrl->z2 - 3.0f * w0 * e);
        ctrl->z2 += dt_s * (ctrl->z3 + input_accel - 3.0f * w0 * w0 * e);
        ctrl->z3 += dt_s * (-w0 * w0 * w0 * e);
        ctrl->w0 = w0;
    }
    out.angle_error_rad = yaw_lqr_wrap_pi(feedback->angle_rad - ref->angle_rad);
    out.rate_error_rad_s = feedback->rate_rad_s - ref->rate_ref_rad_s;
    out.torque_feedback_nm = -cfg->k_angle_nm_rad * out.angle_error_rad -
                             cfg->k_rate_nms_rad * out.rate_error_rad_s;
    if (cfg->integral_enable != 0u && cfg->k_integral_nm_rad_s > 0.0f &&
        cfg->integral_limit_nm > 0.0f && yaw_lqr_abs(out.angle_error_rad) < 0.20f &&
        yaw_lqr_abs(feedback->rate_rad_s) < 0.20f) {
        ctrl->integral_torque_nm -= cfg->k_integral_nm_rad_s * out.angle_error_rad * dt_s;
        ctrl->integral_torque_nm = yaw_lqr_clamp(ctrl->integral_torque_nm,
                                                 -cfg->integral_limit_nm,
                                                 cfg->integral_limit_nm);
        out.integral_active = 1u;
    } else if (cfg->integral_enable == 0u || cfg->k_integral_nm_rad_s <= 0.0f ||
               cfg->integral_limit_nm <= 0.0f) {
        ctrl->integral_torque_nm = 0.0f;
    }
    out.torque_integral_nm = ctrl->integral_torque_nm;
    out.torque_inertia_nm = cfg->inertia_kg_m2 * ref->accel_ref_rad_s2;
    if (cfg->eso_enable != 0u && cfg->eso_comp_enable != 0u && cfg->eso_comp_gain != 0.0f) {
        out.torque_eso_nm = yaw_lqr_clamp(-cfg->eso_comp_gain * cfg->inertia_kg_m2 * ctrl->z3,
                                          -cfg->eso_comp_limit_nm, cfg->eso_comp_limit_nm);
        out.eso_active = 1u;
    }
    torque_nm = out.torque_feedback_nm + out.torque_integral_nm +
                out.torque_inertia_nm + out.torque_eso_nm;
    /* Identification excitation is expressed in native GM6020 current counts
     * and passes through the same final current and slew protection. */
    current = torque_nm * cfg->torque_to_current + ref->current_injection;
    out.current_pre_limit = current;
    if (cfg->current_soft_limit > 0.0f) {
        const float limited = yaw_lqr_clamp(current, -cfg->current_soft_limit,
                                            cfg->current_soft_limit);
        out.limit_active = (limited != current) ? 1u : 0u;
        current = limited;
    }
    if (cfg->current_min < cfg->current_max) {
        const float limited = yaw_lqr_clamp(current, cfg->current_min, cfg->current_max);
        out.limit_active = (limited != current) ? 1u : out.limit_active;
        current = limited;
    }
    if (cfg->slew_enable != 0u && cfg->current_slew_rate_s > 0.0f) {
        const float max_delta = cfg->current_slew_rate_s * dt_s;
        float slew_center = feedback->applied_current;
        if (cfg->current_soft_limit > 0.0f) {
            slew_center = yaw_lqr_clamp(slew_center, -cfg->current_soft_limit,
                                        cfg->current_soft_limit);
        }
        if (cfg->current_min < cfg->current_max) {
            slew_center = yaw_lqr_clamp(slew_center, cfg->current_min, cfg->current_max);
        }
        const float limited = yaw_lqr_clamp(current,
                                            slew_center - max_delta,
                                            slew_center + max_delta);
        out.slew_active = (limited != current) ? 1u : 0u;
        current = limited;
    }
    if (!yaw_lqr_finite(current)) {
        out.feedback_fault = 1u;
        ctrl->feedback_ready = 0u;
    } else {
        out.output_valid = 1u;
        out.current_cmd = current;
    }
    if (output != NULL) *output = out;
}
