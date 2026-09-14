#include "yaw_identification_test.h"

#include <math.h>

#define YAW_TEST_MIN_DT_S                    (0.0001f)
#define YAW_TEST_MAX_DT_S                    (0.02f)
#define YAW_TEST_DEFAULT_DT_S                (0.001f)
#define YAW_TEST_ARM_TIME_S                  (0.50f)
#define YAW_TEST_PRBS_LOW_AMPLITUDE_CURRENT  (1800.0f)
#define YAW_TEST_PRBS_LOW_INTERVAL_S         (0.040f)
#define YAW_TEST_PRBS_LOW_DURATION_S         (10.0f)
#define YAW_TEST_PRBS_HIGH_AMPLITUDE_CURRENT (3000.0f)
#define YAW_TEST_PRBS_HIGH_INTERVAL_S        (0.020f)
#define YAW_TEST_PRBS_HIGH_DURATION_S        (15.0f)
#define YAW_TEST_CHIRP_AMPLITUDE_CURRENT     (2400.0f)
#define YAW_TEST_CHIRP_FREQUENCY_START_HZ    (0.30f)
#define YAW_TEST_CHIRP_FREQUENCY_END_HZ      (6.00f)
#define YAW_TEST_CHIRP_DURATION_S            (15.0f)
#define YAW_TEST_CHIRP_RAMP_S                (0.50f)
#define YAW_TEST_RECOVER_TIME_S              (2.0f)
#define YAW_TEST_PI                          (3.14159265358979323846f)
#define YAW_TEST_TWO_PI                      (2.0f * YAW_TEST_PI)
#define YAW_TEST_LFSR_SEED                    (0xACE1u)

volatile uint8_t g_yaw_test_state = YAW_TEST_IDLE;
volatile uint8_t g_yaw_test_finished = 0u;
volatile uint8_t g_yaw_test_sample_valid = 0u;
volatile uint32_t g_yaw_test_sample_count = 0u;
volatile float g_yaw_test_target_debug = 0.0f;
volatile float g_yaw_test_elapsed_debug = 0.0f;
volatile float g_yaw_test_chirp_frequency_debug = 0.0f;
volatile float yaw_id_current_injection_debug = 0.0f;

static float yaw_test_target = 0.0f;
static float yaw_test_state_elapsed = 0.0f;
static float yaw_test_prbs_elapsed = 0.0f;
static float yaw_test_chirp_phase = 0.0f;
static uint16_t yaw_test_lfsr = YAW_TEST_LFSR_SEED;
static uint8_t yaw_test_active = 0u;

static float YawTestDt(float dt_s)
{
    if (!isfinite(dt_s) || dt_s < YAW_TEST_MIN_DT_S ||
        dt_s > YAW_TEST_MAX_DT_S) {
        return YAW_TEST_DEFAULT_DT_S;
    }
    return dt_s;
}

static void YawTestSetState(YawTestState_e state)
{
    g_yaw_test_state = (uint8_t)state;
    g_yaw_test_sample_valid = 0u;
    yaw_test_state_elapsed = 0.0f;
    yaw_test_prbs_elapsed = 0.0f;
    g_yaw_test_elapsed_debug = 0.0f;
    g_yaw_test_chirp_frequency_debug = 0.0f;
    yaw_id_current_injection_debug = 0.0f;
}

static void YawTestStop(float yaw_measure_deg)
{
    yaw_test_active = 0u;
    g_yaw_test_state = YAW_TEST_IDLE;
    g_yaw_test_finished = 0u;
    g_yaw_test_sample_valid = 0u;
    g_yaw_test_target_debug = yaw_measure_deg;
    g_yaw_test_elapsed_debug = 0.0f;
    g_yaw_test_chirp_frequency_debug = 0.0f;
    yaw_id_current_injection_debug = 0.0f;
    yaw_test_target = yaw_measure_deg;
    yaw_test_state_elapsed = 0.0f;
    yaw_test_prbs_elapsed = 0.0f;
    yaw_test_chirp_phase = 0.0f;
    yaw_test_lfsr = YAW_TEST_LFSR_SEED;
}

static void YawTestStart(float yaw_measure_deg)
{
    yaw_test_active = 1u;
    g_yaw_test_finished = 0u;
    g_yaw_test_sample_valid = 0u;
    g_yaw_test_sample_count = 0u;
    yaw_test_target = yaw_measure_deg;
    yaw_test_chirp_phase = 0.0f;
    yaw_test_lfsr = YAW_TEST_LFSR_SEED;
    g_yaw_test_target_debug = yaw_test_target;
    YawTestSetState(YAW_TEST_ARM);
}

static float YawTestNextPrbs(float amplitude_current)
{
    const uint16_t feedback_bit = (uint16_t)(
        ((yaw_test_lfsr >> 0u) ^ (yaw_test_lfsr >> 2u) ^
         (yaw_test_lfsr >> 3u) ^ (yaw_test_lfsr >> 5u)) & 1u);
    yaw_test_lfsr = (uint16_t)((yaw_test_lfsr >> 1u) |
                               (feedback_bit << 15u));
    return ((yaw_test_lfsr & 1u) != 0u) ? amplitude_current :
                                          -amplitude_current;
}

static void YawTestUpdatePrbs(float dt_s,
                              float interval_s,
                              float amplitude_current)
{
    yaw_test_prbs_elapsed += dt_s;
    if (yaw_test_prbs_elapsed >= interval_s ||
        yaw_id_current_injection_debug == 0.0f) {
        while (yaw_test_prbs_elapsed >= interval_s) {
            yaw_test_prbs_elapsed -= interval_s;
        }
        yaw_id_current_injection_debug = YawTestNextPrbs(amplitude_current);
    }
    g_yaw_test_sample_valid = 1u;
    g_yaw_test_sample_count++;
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

    /* Identification intentionally does not abort on tracking excursions.
     * The input is only used while double-up is held; invalid rows are
     * rejected by the capture flag at the LQR boundary. */
    (void)yaw_rate_rad_s;
    (void)feedback_ok;
    (void)yaw_lqr_identification_ready;

    g_yaw_test_sample_valid = 0u;
    yaw_test_state_elapsed += dt;
    g_yaw_test_elapsed_debug = yaw_test_state_elapsed;

    if (g_yaw_test_double_up == 0u) {
        YawTestStop(yaw_measure_deg);
        return;
    }
    if (yaw_test_active == 0u) {
        YawTestStart(yaw_measure_deg);
    }

    g_yaw_test_target_debug = yaw_test_target;
    switch ((YawTestState_e)g_yaw_test_state) {
    case YAW_TEST_ARM:
        if (yaw_test_state_elapsed >= YAW_TEST_ARM_TIME_S) {
            YawTestSetState(YAW_TEST_PRBS_LOW);
        }
        break;

    case YAW_TEST_PRBS_LOW:
        YawTestUpdatePrbs(dt,
                          YAW_TEST_PRBS_LOW_INTERVAL_S,
                          YAW_TEST_PRBS_LOW_AMPLITUDE_CURRENT);
        if (yaw_test_state_elapsed >= YAW_TEST_PRBS_LOW_DURATION_S) {
            YawTestSetState(YAW_TEST_PRBS_HIGH);
        }
        break;

    case YAW_TEST_PRBS_HIGH:
        YawTestUpdatePrbs(dt,
                          YAW_TEST_PRBS_HIGH_INTERVAL_S,
                          YAW_TEST_PRBS_HIGH_AMPLITUDE_CURRENT);
        if (yaw_test_state_elapsed >= YAW_TEST_PRBS_HIGH_DURATION_S) {
            yaw_test_chirp_phase = 0.0f;
            YawTestSetState(YAW_TEST_CURRENT_CHIRP);
        }
        break;

    case YAW_TEST_CURRENT_CHIRP: {
        const float frequency_slope =
            (YAW_TEST_CHIRP_FREQUENCY_END_HZ -
             YAW_TEST_CHIRP_FREQUENCY_START_HZ) /
            YAW_TEST_CHIRP_DURATION_S;
        const float frequency = YAW_TEST_CHIRP_FREQUENCY_START_HZ +
                                frequency_slope * yaw_test_state_elapsed;
        float amplitude_scale = 1.0f;

        if (yaw_test_state_elapsed < YAW_TEST_CHIRP_RAMP_S) {
            const float ramp = yaw_test_state_elapsed / YAW_TEST_CHIRP_RAMP_S;
            amplitude_scale = 3.0f * ramp * ramp -
                              2.0f * ramp * ramp * ramp;
        }
        yaw_id_current_injection_debug =
            YAW_TEST_CHIRP_AMPLITUDE_CURRENT * amplitude_scale *
            sinf(yaw_test_chirp_phase);
        yaw_test_chirp_phase += 2.0f * YAW_TEST_PI * frequency * dt;
        if (yaw_test_chirp_phase >= YAW_TEST_TWO_PI) {
            yaw_test_chirp_phase -= YAW_TEST_TWO_PI;
        }
        g_yaw_test_chirp_frequency_debug = frequency;
        g_yaw_test_sample_valid = 1u;
        g_yaw_test_sample_count++;
        if (yaw_test_state_elapsed >= YAW_TEST_CHIRP_DURATION_S) {
            YawTestSetState(YAW_TEST_RECOVER);
        }
        break;
    }

    case YAW_TEST_RECOVER:
        if (yaw_test_state_elapsed >= YAW_TEST_RECOVER_TIME_S) {
            g_yaw_test_finished = 1u;
            YawTestSetState(YAW_TEST_DONE);
        }
        break;

    case YAW_TEST_DONE:
        g_yaw_test_finished = 1u;
        break;

    case YAW_TEST_ABORT:
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

float YawTest_GetCurrentInjection(void)
{
    return yaw_id_current_injection_debug;
}
