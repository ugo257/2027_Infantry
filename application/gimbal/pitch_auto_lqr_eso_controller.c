#include "pitch_auto_lqr_eso_controller.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#define PITCH_LQR_MIN_J    (1.0e-6f)
#define PITCH_LQR_MIN_GAIN (1.0e-6f)
#define PITCH_LQR_MIN_DT_S (1.0e-4f)
#define PITCH_LQR_MAX_DT_S (0.01f)
#define PITCH_LQR_MAX_ESO_W0_RAD_S (500.0f)

static float PitchLqrAbs(float value)
{
    return (value >= 0.0f) ? value : -value;
}

static float PitchLqrClamp(float value, float min_value, float max_value)
{
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

static float PitchLqrClampAbs(float value, float limit)
{
    if (limit <= 0.0f) {
        return value;
    }
    return PitchLqrClamp(value, -limit, limit);
}

static float PitchLqrDeadband(float value, float deadband)
{
    const float abs_value = PitchLqrAbs(value);

    if (deadband <= 0.0f) {
        return value;
    }
    if (abs_value <= deadband) {
        return 0.0f;
    }
    return (value > 0.0f) ? (value - deadband) : (value + deadband);
}

static uint8_t PitchLqrFinite(float value)
{
    return isfinite(value) ? 1U : 0U;
}

static uint8_t PitchLqrFeedbackValid(const PitchAutoLqrEsoConfig_t *cfg,
                                     const PitchAutoLqrEsoFeedback_t *feedback,
                                     const PitchAutoLqrEsoReference_t *ref)
{
    if (cfg == NULL || feedback == NULL || ref == NULL ||
        feedback->feedback_ok == 0U || cfg->j_kg_m2 <= PITCH_LQR_MIN_J ||
        PitchLqrAbs(cfg->torque_to_axis_gain) <= PITCH_LQR_MIN_GAIN) {
        return 0U;
    }

    return (uint8_t)(PitchLqrFinite(feedback->theta_rad) != 0U &&
                     PitchLqrFinite(feedback->omega_rad_s) != 0U &&
                     PitchLqrFinite(feedback->tau_applied_nm) != 0U &&
                     PitchLqrFinite(ref->theta_rad) != 0U &&
                     PitchLqrFinite(ref->omega_rad_s) != 0U &&
                     PitchLqrFinite(ref->alpha_rad_s2) != 0U &&
                     PitchLqrFinite(ref->tau_gravity_nm) != 0U);
}

static uint8_t PitchLqrGatePass(float value, float limit)
{
    if (limit <= 0.0f) {
        return 1U;
    }
    return (PitchLqrAbs(value) <= limit) ? 1U : 0U;
}

static void PitchLqrUpdateEsoGains(PitchAutoLqrEso_t *ctrl,
                                   const PitchAutoLqrEsoConfig_t *cfg,
                                   float dt_s)
{
    float w0;

    if (ctrl == NULL || cfg == NULL || cfg->eso_enable == 0U ||
        cfg->j_kg_m2 <= PITCH_LQR_MIN_J) {
        if (ctrl != NULL) {
            ctrl->eso.b0 = 0.0f;
            ctrl->eso.w0 = 0.0f;
            ctrl->eso.beta1 = 0.0f;
            ctrl->eso.beta2 = 0.0f;
            ctrl->eso.beta3 = 0.0f;
        }
        return;
    }

    w0 = cfg->eso_bandwidth_rad_s;
    if (w0 <= 0.0f && cfg->eso_alpha > 0.0f) {
        const float wn_term = cfg->k_theta_nm_rad / cfg->j_kg_m2;
        const float wn = (wn_term > 0.0f) ? sqrtf(wn_term) : 0.0f;
        const float sigma = cfg->k_omega_nms_rad /
                            (2.0f * cfg->j_kg_m2);
        float closed_loop_bandwidth;

        closed_loop_bandwidth = (wn > sigma) ? wn : sigma;
        w0 = cfg->eso_alpha * closed_loop_bandwidth;
    }

    w0 = PitchLqrClamp(w0,
                       0.0f,
                       PitchLqrClamp(1.0f / dt_s,
                                     0.0f,
                                     PITCH_LQR_MAX_ESO_W0_RAD_S));
    ctrl->eso.b0 = cfg->torque_to_axis_gain / cfg->j_kg_m2;
    ctrl->eso.w0 = w0;
    ctrl->eso.beta1 = 3.0f * w0;
    ctrl->eso.beta2 = 3.0f * w0 * w0;
    ctrl->eso.beta3 = w0 * w0 * w0;
}

static void PitchLqrUpdateEso(PitchAutoLqrEso_t *ctrl,
                              const PitchAutoLqrEsoConfig_t *cfg,
                              const PitchAutoLqrEsoFeedback_t *feedback,
                              float tau_gravity_axis_nm,
                              float dt_s)
{
    float e;
    float z1_dot;
    float z2_dot;
    float z3_dot;

    if (ctrl == NULL || cfg == NULL || feedback == NULL ||
        cfg->eso_enable == 0U || ctrl->eso.w0 <= 0.0f ||
        cfg->j_kg_m2 <= PITCH_LQR_MIN_J) {
        return;
    }

    e = feedback->theta_rad - ctrl->eso.z1;
    z1_dot = ctrl->eso.z2 + ctrl->eso.beta1 * e;
    /* Gravity is a known model term. Only the residual (friction, load and
     * unmodelled effects) is left for the extended disturbance state z3. */
    z2_dot = ctrl->eso.b0 * feedback->tau_applied_nm -
             tau_gravity_axis_nm / cfg->j_kg_m2 + ctrl->eso.z3 +
             ctrl->eso.beta2 * e;
    z3_dot = ctrl->eso.beta3 * e;

    ctrl->eso.z1 += dt_s * z1_dot;
    ctrl->eso.z2 += dt_s * z2_dot;
    ctrl->eso.z3 += dt_s * z3_dot;
}

void PitchAutoLqrEso_Init(PitchAutoLqrEso_t *ctrl)
{
    if (ctrl != NULL) {
        memset(ctrl, 0, sizeof(*ctrl));
    }
}

void PitchAutoLqrEso_Reset(PitchAutoLqrEso_t *ctrl, float theta_rad, float omega_rad_s)
{
    if (ctrl == NULL) {
        return;
    }

    memset(ctrl, 0, sizeof(*ctrl));
    ctrl->eso.z1 = theta_rad;
    ctrl->eso.z2 = omega_rad_s;
}

uint8_t PitchAutoLqrEso_GetStateSpace(const PitchAutoLqrEsoConfig_t *cfg,
                                      PitchAutoLqrEsoStateSpace_t *model)
{
    if (cfg == NULL || model == NULL || cfg->j_kg_m2 <= PITCH_LQR_MIN_J ||
        PitchLqrAbs(cfg->torque_to_axis_gain) <= PITCH_LQR_MIN_GAIN) {
        return 0U;
    }

    memset(model, 0, sizeof(*model));
    model->a[0][1] = 1.0f;
    model->a[1][1] = 0.0f;
    model->b[1] = cfg->torque_to_axis_gain / cfg->j_kg_m2;
    model->c[0] = 1.0f;
    return 1U;
}

void PitchAutoLqrEso_Calc(PitchAutoLqrEso_t *ctrl,
                          const PitchAutoLqrEsoConfig_t *cfg,
                          const PitchAutoLqrEsoFeedback_t *feedback,
                          const PitchAutoLqrEsoReference_t *ref,
                          float dt_s,
                          PitchAutoLqrEsoOutput_t *output)
{
    PitchAutoLqrEsoOutput_t out;
    float tau_axis_nm;
    float tau_motor_nm;

    memset(&out, 0, sizeof(out));
    if (ctrl == NULL || cfg == NULL || feedback == NULL || ref == NULL) {
        if (output != NULL) {
            *output = out;
        }
        return;
    }

    out.theta_ref_rad = ref->theta_rad;
    out.omega_ref_rad_s = ref->omega_rad_s;
    out.alpha_ref_rad_s2 = ref->alpha_rad_s2;

    if (PitchLqrFeedbackValid(cfg, feedback, ref) == 0U) {
        out.feedback_fault = 1U;
        PitchAutoLqrEso_Reset(ctrl, feedback->theta_rad, feedback->omega_rad_s);
        if (output != NULL) {
            *output = out;
        }
        return;
    }

    if (PitchLqrFinite(dt_s) == 0U || dt_s < PITCH_LQR_MIN_DT_S ||
        dt_s > PITCH_LQR_MAX_DT_S) {
        out.timing_fault = 1U;
        ctrl->feedback_ready = 0U;
        if (output != NULL) {
            *output = out;
        }
        return;
    }

    PitchLqrUpdateEsoGains(ctrl, cfg, dt_s);
    if (ctrl->feedback_ready == 0U) {
        ctrl->eso.z1 = feedback->theta_rad;
        ctrl->eso.z2 = feedback->omega_rad_s;
        ctrl->eso.z3 = 0.0f;
        ctrl->feedback_ready = 1U;
    } else {
        PitchLqrUpdateEso(ctrl, cfg, feedback, ref->tau_gravity_nm, dt_s);
    }

    out.e_theta_rad = PitchLqrDeadband(feedback->theta_rad - ref->theta_rad,
                                       cfg->theta_deadband_rad);
    out.e_omega_rad_s = feedback->omega_rad_s - ref->omega_rad_s;
    out.tau_feedback_axis_nm = -cfg->k_theta_nm_rad * out.e_theta_rad -
                               cfg->k_omega_nms_rad * out.e_omega_rad_s;
    out.tau_inertia_axis_nm = cfg->j_kg_m2 * ref->alpha_rad_s2;
    /* Friction is part of d and is intentionally not feedforwarded. */
    out.tau_viscous_axis_nm = 0.0f;
    out.tau_coulomb_axis_nm = 0.0f;
    out.tau_gravity_axis_nm = ref->tau_gravity_nm;
    out.tau_model_axis_nm = out.tau_inertia_axis_nm +
                            out.tau_viscous_axis_nm +
                            out.tau_coulomb_axis_nm +
                            out.tau_gravity_axis_nm;

    if (cfg->eso_enable != 0U && ctrl->eso.w0 > 0.0f) {
        out.tau_eso_raw_axis_nm = PitchLqrClampAbs(
            -cfg->eso_comp_gain * cfg->j_kg_m2 * ctrl->eso.z3,
            cfg->eso_comp_limit_nm);
        if (cfg->eso_comp_enable != 0U &&
            PitchLqrGatePass(feedback->omega_rad_s,
                             cfg->eso_omega_gate_rad_s) != 0U &&
            PitchLqrGatePass(ref->alpha_rad_s2,
                             cfg->eso_alpha_gate_rad_s2) != 0U) {
            out.eso_comp_active = 1U;
            out.tau_eso_active_axis_nm = out.tau_eso_raw_axis_nm;
        }
    }

    tau_axis_nm = out.tau_feedback_axis_nm + out.tau_model_axis_nm +
                  out.tau_eso_active_axis_nm;
    tau_motor_nm = tau_axis_nm / cfg->torque_to_axis_gain;
    out.tau_pre_limit_motor_nm = tau_motor_nm;

    if (cfg->torque_soft_limit_nm > 0.0f) {
        const float limited = PitchLqrClampAbs(tau_motor_nm,
                                               cfg->torque_soft_limit_nm);
        out.soft_limit_active = (limited != tau_motor_nm) ? 1U : 0U;
        tau_motor_nm = limited;
    }
    if (cfg->torque_min_nm < cfg->torque_max_nm) {
        const float limited = PitchLqrClamp(tau_motor_nm,
                                            cfg->torque_min_nm,
                                            cfg->torque_max_nm);
        out.hard_limit_active = (limited != tau_motor_nm) ? 1U : 0U;
        tau_motor_nm = limited;
    }

    out.tau_cmd_before_slew_nm = tau_motor_nm;
    if (cfg->torque_slew_enable != 0U && cfg->torque_slew_rate_nm_s > 0.0f) {
        const float max_delta = cfg->torque_slew_rate_nm_s * dt_s;
        float slew_center = feedback->tau_applied_nm;
        if (cfg->torque_soft_limit_nm > 0.0f) {
            slew_center = PitchLqrClampAbs(slew_center,
                                           cfg->torque_soft_limit_nm);
        }
        if (cfg->torque_min_nm < cfg->torque_max_nm) {
            slew_center = PitchLqrClamp(slew_center,
                                        cfg->torque_min_nm,
                                        cfg->torque_max_nm);
        }
        const float limited = PitchLqrClamp(tau_motor_nm,
                                            slew_center - max_delta,
                                            slew_center + max_delta);
        out.slew_limit_active = (limited != tau_motor_nm) ? 1U : 0U;
        tau_motor_nm = limited;
    }

    if (PitchLqrFinite(tau_motor_nm) == 0U) {
        out.feedback_fault = 1U;
        ctrl->feedback_ready = 0U;
    } else {
        out.output_valid = 1U;
        out.tau_cmd_nm = tau_motor_nm;
        ctrl->tau_cmd_last_nm = tau_motor_nm;
    }

    if (output != NULL) {
        *output = out;
    }
}
