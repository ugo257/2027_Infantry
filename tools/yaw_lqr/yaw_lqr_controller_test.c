#include "yaw_lqr_eso_controller.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

static YawLqrEsoConfig_t test_config(void)
{
    YawLqrEsoConfig_t cfg = {0};
    cfg.inertia_kg_m2 = 0.02f;
    cfg.k_angle_nm_rad = 10.0f;
    cfg.k_rate_nms_rad = 1.0f;
    cfg.torque_to_current = 100.0f;
    cfg.current_soft_limit = 5000.0f;
    cfg.current_min = -5000.0f;
    cfg.current_max = 5000.0f;
    return cfg;
}

static void expect_near(float actual, float expected, float tolerance)
{
    assert(fabsf(actual - expected) <= tolerance);
}

int main(void)
{
    YawLqrEso_t ctrl;
    YawLqrEsoOutput_t out;
    YawLqrEsoConfig_t cfg = test_config();
    YawLqrEsoFeedback_t feedback = {0};
    YawLqrEsoReference_t ref = {0};

    feedback.feedback_ok = 1u;
    ref.angle_rad = 0.1f;
    YawLqrEso_Init(&ctrl);
    YawLqrEso_Calc(&ctrl, &cfg, &feedback, &ref, 0.001f, &out);
    assert(out.output_valid != 0u);
    expect_near(out.current_cmd, 100.0f, 0.001f);

    ref.angle_rad = 0.0f;
    YawLqrEso_Reset(&ctrl, 0.0f, 0.0f);
    YawLqrEso_Calc(&ctrl, &cfg, &feedback, &ref, 0.001f, &out);
    expect_near(out.current_cmd, 0.0f, 0.001f);

    feedback.rate_rad_s = 1.0f;
    YawLqrEso_Reset(&ctrl, 0.0f, feedback.rate_rad_s);
    YawLqrEso_Calc(&ctrl, &cfg, &feedback, &ref, 0.001f, &out);
    expect_near(out.current_cmd, -100.0f, 0.001f);
    feedback.rate_rad_s = 0.0f;

    /* The installed GM6020/Yaw coordinate uses negative current for positive
     * yaw torque. A positive angle target must therefore request negative
     * motor current, while positive measured rate requests positive damping
     * current. */
    cfg = test_config();
    cfg.torque_to_current = -100.0f;
    ref.angle_rad = 0.1f;
    YawLqrEso_Reset(&ctrl, 0.0f, 0.0f);
    YawLqrEso_Calc(&ctrl, &cfg, &feedback, &ref, 0.001f, &out);
    expect_near(out.current_cmd, -100.0f, 0.001f);
    ref.angle_rad = 0.0f;
    feedback.rate_rad_s = 1.0f;
    YawLqrEso_Reset(&ctrl, 0.0f, feedback.rate_rad_s);
    YawLqrEso_Calc(&ctrl, &cfg, &feedback, &ref, 0.001f, &out);
    expect_near(out.current_cmd, 100.0f, 0.001f);
    feedback.rate_rad_s = 0.0f;

    YawLqrEso_Calc(&ctrl, &cfg, &feedback, &ref, 0.1f, &out);
    assert(out.output_valid == 0u && out.timing_fault != 0u);

    cfg.torque_to_current = 0.0f;
    YawLqrEso_Calc(&ctrl, &cfg, &feedback, &ref, 0.001f, &out);
    assert(out.output_valid == 0u && out.feedback_fault != 0u);

    cfg = test_config();
    cfg.current_soft_limit = 200.0f;
    ref.angle_rad = 1.0f;
    YawLqrEso_Reset(&ctrl, 0.0f, 0.0f);
    YawLqrEso_Calc(&ctrl, &cfg, &feedback, &ref, 0.001f, &out);
    assert(out.limit_active != 0u);
    expect_near(out.current_cmd, 200.0f, 0.001f);

    cfg = test_config();
    cfg.slew_enable = 1u;
    cfg.current_slew_rate_s = 100.0f;
    feedback.applied_current = 50.0f;
    ref.angle_rad = 0.1f;
    YawLqrEso_Reset(&ctrl, 0.0f, 0.0f);
    YawLqrEso_Calc(&ctrl, &cfg, &feedback, &ref, 0.01f, &out);
    assert(out.slew_active != 0u);
    expect_near(out.current_cmd, 51.0f, 0.001f);

    cfg = test_config();
    cfg.integral_enable = 1u;
    cfg.k_integral_nm_rad_s = 2.0f;
    cfg.integral_limit_nm = 0.5f;
    feedback.angle_rad = -0.01f;
    feedback.applied_current = 0.0f;
    ref.angle_rad = 0.0f;
    YawLqrEso_Reset(&ctrl, feedback.angle_rad, 0.0f);
    YawLqrEso_Calc(&ctrl, &cfg, &feedback, &ref, 0.01f, &out);
    assert(out.integral_active != 0u && out.torque_integral_nm > 0.0f);

    puts("yaw_lqr_controller_test: PASS");
    return 0;
}
