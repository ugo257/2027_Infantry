#include "yaw_identification_test.h"
#include "robot_def.h"
#include "yaw_qp_reference_tables.h"

#include <math.h>

#define YAW_TEST_MIN_DT_S                    (0.0001f)
#define YAW_TEST_MAX_DT_S                    (0.02f)
#define YAW_TEST_DEFAULT_DT_S                (0.001f)
#define YAW_TEST_ARM_TIME_S                  (0.50f)
#define YAW_TEST_TRAJECTORY_TIME_S           (10.0f)
#define YAW_TEST_MIN_FREQUENCY_HZ            (0.10f)
#define YAW_TEST_MAX_FREQUENCY_HZ            (10.0f)
#define YAW_TEST_MIN_PEAK_TO_PEAK_DEG        (0.10f)
#define YAW_TEST_MAX_PEAK_TO_PEAK_DEG        (120.0f)
#define YAW_TEST_MIN_ACCEL_RAD_S2            (1.0f)
#define YAW_TEST_MAX_ACCEL_RAD_S2            (2000.0f)
#define YAW_TEST_RAD_TO_DEG                  (57.295779513082320876f)

volatile uint8_t g_yaw_test_state = YAW_TEST_IDLE;
volatile uint8_t g_yaw_test_finished = 0u;
volatile uint8_t g_yaw_test_sample_valid = 0u;
volatile uint32_t g_yaw_test_sample_count = 0u;
volatile float g_yaw_test_target_debug = 0.0f;
volatile float g_yaw_test_elapsed_debug = 0.0f;
volatile float g_yaw_test_ref_rate_debug = 0.0f;
volatile float g_yaw_test_ref_accel_debug = 0.0f;
volatile float g_yaw_test_wave_frequency_debug = 0.0f;
volatile float g_yaw_test_wave_accel_limit_debug = 0.0f;
volatile float g_yaw_test_planned_peak_to_peak_debug = 0.0f;
volatile float g_yaw_test_planner_transition_debug = 0.0f;
volatile float yaw_id_current_injection_debug = 0.0f;

volatile float g_yaw_test_wave_frequency_hz =
    GIMBAL_YAW_TEST_WAVE_FREQUENCY_HZ;
volatile float g_yaw_test_wave_peak_to_peak_deg =
    GIMBAL_YAW_TEST_WAVE_PEAK_TO_PEAK_DEG;
volatile float g_yaw_test_max_accel_rad_s2 =
    GIMBAL_YAW_TEST_MAX_ACCEL_RAD_S2;
volatile uint8_t g_yaw_test_accel_ff_enable =
    GIMBAL_YAW_TEST_ACCEL_FF_ENABLE_DEFAULT;

static float yaw_test_target = 0.0f;
static float yaw_test_center_deg = 0.0f;
static float yaw_test_state_elapsed = 0.0f;
static float yaw_test_ref_rate_rad_s = 0.0f;
static float yaw_test_ref_accel_rad_s2 = 0.0f;
static float yaw_test_frequency_hz = GIMBAL_YAW_TEST_WAVE_FREQUENCY_HZ;
static float yaw_test_accel_limit_rad_s2 = GIMBAL_YAW_TEST_MAX_ACCEL_RAD_S2;
static float yaw_test_amplitude_scale = 1.0f;
static float yaw_test_accel_blend = 0.0f;
static uint8_t yaw_test_accel_lower_index = 0u;
static uint8_t yaw_test_accel_upper_index = 0u;
static const float (*yaw_test_angle_table)[YAW_TEST_QP_REFERENCE_POINTS] =
    yaw_test_qp_3hz_angle_rad;
static const float (*yaw_test_rate_table)[YAW_TEST_QP_REFERENCE_POINTS] =
    yaw_test_qp_3hz_rate_rad_s;
static const float (*yaw_test_accel_table)[YAW_TEST_QP_REFERENCE_POINTS] =
    yaw_test_qp_3hz_accel_rad_s2;
static float yaw_test_entry_c3_rad = 0.0f;
static float yaw_test_entry_c4_rad = 0.0f;
static float yaw_test_entry_c5_rad = 0.0f;
static uint8_t yaw_test_accel_ff_latched = 0u;
static uint8_t yaw_test_active = 0u;

static float YawTestClamp(float value, float lower, float upper)
{
    if (value < lower) return lower;
    if (value > upper) return upper;
    return value;
}

static float YawTestDt(float dt_s)
{
    if (!isfinite(dt_s) || dt_s < YAW_TEST_MIN_DT_S ||
        dt_s > YAW_TEST_MAX_DT_S) {
        return YAW_TEST_DEFAULT_DT_S;
    }
    return dt_s;
}

static void YawTestResetReference(void)
{
    g_yaw_test_target_debug = yaw_test_center_deg;
    g_yaw_test_ref_rate_debug = 0.0f;
    g_yaw_test_ref_accel_debug = 0.0f;
    yaw_test_target = yaw_test_center_deg;
    yaw_test_ref_rate_rad_s = 0.0f;
    yaw_test_ref_accel_rad_s2 = 0.0f;
}

static void YawTestSetState(YawTestState_e state)
{
    g_yaw_test_state = (uint8_t)state;
    g_yaw_test_sample_valid = 0u;
    yaw_test_state_elapsed = 0.0f;
    g_yaw_test_elapsed_debug = 0.0f;
    yaw_id_current_injection_debug = 0.0f;
    YawTestResetReference();
}

static void YawTestStop(float yaw_measure_deg)
{
    yaw_test_active = 0u;
    g_yaw_test_state = YAW_TEST_IDLE;
    g_yaw_test_finished = 0u;
    g_yaw_test_sample_valid = 0u;
    yaw_test_center_deg = isfinite(yaw_measure_deg) ? yaw_measure_deg : 0.0f;
    yaw_test_frequency_hz = GIMBAL_YAW_TEST_WAVE_FREQUENCY_HZ;
    yaw_test_accel_limit_rad_s2 = GIMBAL_YAW_TEST_MAX_ACCEL_RAD_S2;
    yaw_test_amplitude_scale = 1.0f;
    yaw_test_accel_blend = 0.0f;
    yaw_test_accel_lower_index = 0u;
    yaw_test_accel_upper_index = 0u;
    yaw_test_angle_table = yaw_test_qp_3hz_angle_rad;
    yaw_test_rate_table = yaw_test_qp_3hz_rate_rad_s;
    yaw_test_accel_table = yaw_test_qp_3hz_accel_rad_s2;
    yaw_test_entry_c3_rad = 0.0f;
    yaw_test_entry_c4_rad = 0.0f;
    yaw_test_entry_c5_rad = 0.0f;
    yaw_test_ref_rate_rad_s = 0.0f;
    yaw_test_ref_accel_rad_s2 = 0.0f;
    g_yaw_test_target_debug = yaw_test_center_deg;
    g_yaw_test_elapsed_debug = 0.0f;
    g_yaw_test_ref_rate_debug = 0.0f;
    g_yaw_test_ref_accel_debug = 0.0f;
    g_yaw_test_wave_frequency_debug = 0.0f;
    g_yaw_test_wave_accel_limit_debug = 0.0f;
    g_yaw_test_planned_peak_to_peak_debug = 0.0f;
    g_yaw_test_planner_transition_debug = 0.0f;
    yaw_id_current_injection_debug = 0.0f;
    yaw_test_state_elapsed = 0.0f;
}

static void YawTestPrepareTrajectory(float yaw_measure_deg)
{
    const float requested_frequency =
        YawTestClamp(g_yaw_test_wave_frequency_hz,
                     YAW_TEST_MIN_FREQUENCY_HZ,
                     YAW_TEST_MAX_FREQUENCY_HZ);
    const float peak_to_peak_deg =
        YawTestClamp(g_yaw_test_wave_peak_to_peak_deg,
                     YAW_TEST_MIN_PEAK_TO_PEAK_DEG,
                     YAW_TEST_MAX_PEAK_TO_PEAK_DEG);
    const float requested_accel =
        YawTestClamp(g_yaw_test_max_accel_rad_s2,
                     YAW_TEST_MIN_ACCEL_RAD_S2,
                     YAW_TEST_MAX_ACCEL_RAD_S2);
    const float frequency =
        (fabsf(requested_frequency - 3.0f) <=
         fabsf(requested_frequency - 5.0f)) ? 3.0f : 5.0f;
    float normalized_accel;
    float lower_accel;
    float upper_accel;
    float planned_min = 1.0e9f;
    float planned_max = -1.0e9f;
    uint32_t negative_rate_count = 0u;

    yaw_test_center_deg = isfinite(yaw_measure_deg) ? yaw_measure_deg : 0.0f;
    yaw_test_frequency_hz = frequency;
    yaw_test_amplitude_scale =
        peak_to_peak_deg / YAW_TEST_QP_BASE_PEAK_TO_PEAK_DEG;

    if (frequency < 4.0f) {
        yaw_test_angle_table = yaw_test_qp_3hz_angle_rad;
        yaw_test_rate_table = yaw_test_qp_3hz_rate_rad_s;
        yaw_test_accel_table = yaw_test_qp_3hz_accel_rad_s2;
    } else {
        yaw_test_angle_table = yaw_test_qp_5hz_angle_rad;
        yaw_test_rate_table = yaw_test_qp_5hz_rate_rad_s;
        yaw_test_accel_table = yaw_test_qp_5hz_accel_rad_s2;
    }

    normalized_accel = requested_accel / yaw_test_amplitude_scale;
    if (normalized_accel < yaw_test_qp_accel_levels_rad_s2[0]) {
        /* Preserve the requested value as a hard bound below the smallest
         * generated table by reducing planned excursion proportionally. */
        yaw_test_amplitude_scale *=
            normalized_accel / yaw_test_qp_accel_levels_rad_s2[0];
        normalized_accel = yaw_test_qp_accel_levels_rad_s2[0];
    }
    yaw_test_accel_lower_index = 0u;
    yaw_test_accel_upper_index = 0u;
    yaw_test_accel_blend = 0.0f;
    if (normalized_accel >=
        yaw_test_qp_accel_levels_rad_s2[YAW_TEST_QP_ACCEL_LEVEL_COUNT - 1u]) {
        yaw_test_accel_lower_index = YAW_TEST_QP_ACCEL_LEVEL_COUNT - 1u;
        yaw_test_accel_upper_index = yaw_test_accel_lower_index;
    } else if (normalized_accel > yaw_test_qp_accel_levels_rad_s2[0]) {
        for (uint8_t index = 0u;
             index + 1u < YAW_TEST_QP_ACCEL_LEVEL_COUNT; ++index) {
            lower_accel = yaw_test_qp_accel_levels_rad_s2[index];
            upper_accel = yaw_test_qp_accel_levels_rad_s2[index + 1u];
            if (normalized_accel <= upper_accel) {
                yaw_test_accel_lower_index = index;
                yaw_test_accel_upper_index = index + 1u;
                yaw_test_accel_blend =
                    (normalized_accel - lower_accel) /
                    (upper_accel - lower_accel);
                break;
            }
        }
    }

    lower_accel = yaw_test_qp_accel_levels_rad_s2[yaw_test_accel_lower_index];
    upper_accel = yaw_test_qp_accel_levels_rad_s2[yaw_test_accel_upper_index];
    yaw_test_accel_limit_rad_s2 = yaw_test_amplitude_scale *
        (lower_accel + yaw_test_accel_blend * (upper_accel - lower_accel));

    for (uint32_t index = 0u; index < YAW_TEST_QP_REFERENCE_POINTS; ++index) {
        const float lower_angle =
            yaw_test_angle_table[yaw_test_accel_lower_index][index];
        const float upper_angle =
            yaw_test_angle_table[yaw_test_accel_upper_index][index];
        const float lower_rate =
            yaw_test_rate_table[yaw_test_accel_lower_index][index];
        const float upper_rate =
            yaw_test_rate_table[yaw_test_accel_upper_index][index];
        const float angle = yaw_test_amplitude_scale *
            (lower_angle + yaw_test_accel_blend * (upper_angle - lower_angle));
        const float rate = yaw_test_amplitude_scale *
            (lower_rate + yaw_test_accel_blend * (upper_rate - lower_rate));
        if (angle < planned_min) planned_min = angle;
        if (angle > planned_max) planned_max = angle;
        if (rate < 0.0f) negative_rate_count++;
    }

    {
        const float duration = YAW_TEST_ARM_TIME_S;
        const float lower_angle =
            yaw_test_angle_table[yaw_test_accel_lower_index][0];
        const float upper_angle =
            yaw_test_angle_table[yaw_test_accel_upper_index][0];
        const float lower_rate =
            yaw_test_rate_table[yaw_test_accel_lower_index][0];
        const float upper_rate =
            yaw_test_rate_table[yaw_test_accel_upper_index][0];
        const float lower_accel_ref =
            yaw_test_accel_table[yaw_test_accel_lower_index][0];
        const float upper_accel_ref =
            yaw_test_accel_table[yaw_test_accel_upper_index][0];
        const float end_angle = yaw_test_amplitude_scale *
            (lower_angle + yaw_test_accel_blend * (upper_angle - lower_angle));
        const float end_rate = yaw_test_amplitude_scale *
            (lower_rate + yaw_test_accel_blend * (upper_rate - lower_rate));
        const float end_accel = yaw_test_amplitude_scale *
            (lower_accel_ref +
             yaw_test_accel_blend * (upper_accel_ref - lower_accel_ref));
        const float velocity_distance = end_rate * duration;
        const float accel_distance = end_accel * duration * duration;
        yaw_test_entry_c3_rad =
            10.0f * end_angle - 4.0f * velocity_distance +
            0.5f * accel_distance;
        yaw_test_entry_c4_rad =
            -15.0f * end_angle + 7.0f * velocity_distance - accel_distance;
        yaw_test_entry_c5_rad =
            6.0f * end_angle - 3.0f * velocity_distance +
            0.5f * accel_distance;
    }

    yaw_test_accel_ff_latched =
        (g_yaw_test_accel_ff_enable != 0u) ? 1u : 0u;
    g_yaw_test_wave_frequency_debug = yaw_test_frequency_hz;
    g_yaw_test_wave_accel_limit_debug = yaw_test_accel_limit_rad_s2;
    g_yaw_test_planned_peak_to_peak_debug =
        (planned_max - planned_min) * YAW_TEST_RAD_TO_DEG;
    g_yaw_test_planner_transition_debug =
        ((float)negative_rate_count /
         (float)YAW_TEST_QP_REFERENCE_POINTS) / yaw_test_frequency_hz;
    YawTestResetReference();
}

static float YawTestBlendedTableValue(
    const float table[][YAW_TEST_QP_REFERENCE_POINTS], uint32_t index)
{
    const float lower = table[yaw_test_accel_lower_index][index];
    const float upper = table[yaw_test_accel_upper_index][index];
    return yaw_test_amplitude_scale *
           (lower + yaw_test_accel_blend * (upper - lower));
}

static void YawTestPublishReference(float angle_rad,
                                    float rate_rad_s,
                                    float accel_rad_s2)
{
    yaw_test_target = yaw_test_center_deg + angle_rad * YAW_TEST_RAD_TO_DEG;
    yaw_test_ref_rate_rad_s = rate_rad_s;
    yaw_test_ref_accel_rad_s2 = accel_rad_s2;
    g_yaw_test_target_debug = yaw_test_target;
    g_yaw_test_ref_rate_debug = yaw_test_ref_rate_rad_s;
    g_yaw_test_ref_accel_debug = yaw_test_ref_accel_rad_s2;
}

static void YawTestGenerateEntry(void)
{
    const float duration = YAW_TEST_ARM_TIME_S;
    const float normalized_time =
        YawTestClamp(yaw_test_state_elapsed / duration, 0.0f, 1.0f);
    const float s2 = normalized_time * normalized_time;
    const float s3 = s2 * normalized_time;
    const float s4 = s3 * normalized_time;
    const float s5 = s4 * normalized_time;
    const float angle_rad = yaw_test_entry_c3_rad * s3 +
                            yaw_test_entry_c4_rad * s4 +
                            yaw_test_entry_c5_rad * s5;
    const float rate_rad_s =
        (3.0f * yaw_test_entry_c3_rad * s2 +
         4.0f * yaw_test_entry_c4_rad * s3 +
         5.0f * yaw_test_entry_c5_rad * s4) / duration;
    const float accel_rad_s2 =
        (6.0f * yaw_test_entry_c3_rad * normalized_time +
         12.0f * yaw_test_entry_c4_rad * s2 +
         20.0f * yaw_test_entry_c5_rad * s3) /
        (duration * duration);
    YawTestPublishReference(angle_rad, rate_rad_s, accel_rad_s2);
}

static void YawTestGenerateTrajectory(void)
{
    const float phase_points =
        fmodf(yaw_test_state_elapsed * yaw_test_frequency_hz, 1.0f) *
        (float)YAW_TEST_QP_REFERENCE_POINTS;
    const uint32_t lower_index = (uint32_t)phase_points;
    const uint32_t upper_index =
        (lower_index + 1u) % YAW_TEST_QP_REFERENCE_POINTS;
    const float fraction = phase_points - (float)lower_index;
    const float lower_angle =
        YawTestBlendedTableValue(yaw_test_angle_table, lower_index);
    const float upper_angle =
        YawTestBlendedTableValue(yaw_test_angle_table, upper_index);
    const float lower_rate =
        YawTestBlendedTableValue(yaw_test_rate_table, lower_index);
    const float upper_rate =
        YawTestBlendedTableValue(yaw_test_rate_table, upper_index);
    const float lower_accel =
        YawTestBlendedTableValue(yaw_test_accel_table, lower_index);
    const float upper_accel =
        YawTestBlendedTableValue(yaw_test_accel_table, upper_index);
    YawTestPublishReference(
        lower_angle + fraction * (upper_angle - lower_angle),
        lower_rate + fraction * (upper_rate - lower_rate),
        lower_accel + fraction * (upper_accel - lower_accel));
}

static void YawTestStart(float yaw_measure_deg)
{
    yaw_test_active = 1u;
    g_yaw_test_finished = 0u;
    g_yaw_test_sample_valid = 0u;
    g_yaw_test_sample_count = 0u;
    YawTestPrepareTrajectory(yaw_measure_deg);
    YawTestSetState(YAW_TEST_ARM);
}

void YawTest_Init(void)
{
    YawTestStop(0.0f);
    g_yaw_test_sample_count = 0u;
}

void YawTest_Update(float yaw_measure_deg,
                    float yaw_rate_rad_s,
                    uint8_t feedback_ok,
                    float dt_s,
                    uint8_t yaw_lqr_identification_ready)
{
    const float dt = YawTestDt(dt_s);

    (void)yaw_rate_rad_s;
    (void)feedback_ok;
    (void)yaw_lqr_identification_ready;

    g_yaw_test_sample_valid = 0u;
    if (g_yaw_test_double_up == 0u) {
        YawTestStop(yaw_measure_deg);
        return;
    }
    if (yaw_test_active == 0u) {
        YawTestStart(yaw_measure_deg);
    }

    yaw_test_state_elapsed += dt;
    g_yaw_test_elapsed_debug = yaw_test_state_elapsed;
    g_yaw_test_wave_frequency_debug = yaw_test_frequency_hz;
    g_yaw_test_wave_accel_limit_debug = yaw_test_accel_limit_rad_s2;

    switch ((YawTestState_e)g_yaw_test_state) {
    case YAW_TEST_ARM:
        YawTestGenerateEntry();
        if (yaw_test_state_elapsed >= YAW_TEST_ARM_TIME_S) {
            /* The quintic entry terminates at table phase zero. Preserve its
             * reference while starting the periodic, valid-data interval. */
            g_yaw_test_state = (uint8_t)YAW_TEST_QP_TRACKING;
            yaw_test_state_elapsed = 0.0f;
            g_yaw_test_elapsed_debug = 0.0f;
        }
        break;

    case YAW_TEST_QP_TRACKING:
        YawTestGenerateTrajectory();
        g_yaw_test_sample_valid = 1u;
        g_yaw_test_sample_count++;
        if (yaw_test_state_elapsed >= YAW_TEST_TRAJECTORY_TIME_S) {
            g_yaw_test_finished = 1u;
            YawTestSetState(YAW_TEST_DONE);
        }
        break;

    case YAW_TEST_DONE:
        g_yaw_test_finished = 1u;
        YawTestResetReference();
        break;

    case YAW_TEST_ABORT:
        YawTestResetReference();
        break;

    case YAW_TEST_IDLE:
    default:
        YawTestStop(yaw_measure_deg);
        break;
    }
}

uint8_t YawTest_IsActive(void)
{
    return yaw_test_active;
}

float YawTest_GetTarget(void)
{
    return yaw_test_target;
}

float YawTest_GetReferenceRate(void)
{
    return yaw_test_ref_rate_rad_s;
}

float YawTest_GetReferenceAcceleration(void)
{
    return yaw_test_ref_accel_rad_s2;
}

uint8_t YawTest_GetAccelerationFeedforwardEnable(void)
{
    return yaw_test_accel_ff_latched;
}

float YawTest_GetCurrentInjection(void)
{
    return yaw_id_current_injection_debug;
}
