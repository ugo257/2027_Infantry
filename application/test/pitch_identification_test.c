#include "pitch_identification_test.h"

#include <math.h>

#include "robot_cmd.h"
#include "robot_types.h"

#define PITCH_TEST_MIN_DT_S                  (0.0001f)
#define PITCH_TEST_MAX_DT_S                  (0.01f)
#define PITCH_TEST_DEFAULT_DT_S              (0.001f)
#define PITCH_TEST_ARM_TIME_S                (2.0f)
#define PITCH_TEST_HOLD_TIME_S               (0.60f)
#define PITCH_TEST_SETTLE_TIME_S             (0.35f)
#define PITCH_TEST_MOVE_TO_START_SPEED_RAD_S (0.25f)
#define PITCH_TEST_TARGET_MIN_RAD            (-0.261799f) /* -15 deg */
#define PITCH_TEST_TARGET_MAX_RAD            (0.261799f)  /* +15 deg */
#define PITCH_TEST_ENDPOINT_TOL_RAD          (0.002f)
#define PITCH_TEST_ENDPOINT_EXCLUSION_RAD    (0.035f)
#define PITCH_TEST_MIN_MEASURED_SPEED_RAD_S  (0.10f)
#define PITCH_TEST_MAX_MEASURED_SPEED_RAD_S  (1.60f)
#define PITCH_TEST_SPEED_ERROR_BASE_RAD_S    (0.12f)
#define PITCH_TEST_SPEED_ERROR_RATIO         (0.25f)
#define PITCH_TEST_ACCEL_LPF_ALPHA           (0.08f)
#define PITCH_TEST_MAX_ACCEL_RAD_S2          (2.0f)
#define PITCH_TEST_SINE_AMPLITUDE_RAD       (0.349066f) /* +/-20 deg */
#define PITCH_TEST_SINE_FREQUENCY_HZ        (2.0f) /* two full cycles per second */
#define PITCH_TEST_SINE_CYCLES              (24.0f)
#define PITCH_TEST_SINE_MAX_SPEED_RAD_S     (8.0f)
#define PITCH_TEST_SINE_MAX_ACCEL_RAD_S2    (160.0f)
#define PITCH_TEST_SINE_ALIGN_SPEED_RAD_S   (0.40f)
#define PITCH_TEST_SINE_ALIGN_TARGET_TOL_RAD (0.002f)
#define PITCH_TEST_SINE_ALIGN_MEAS_TOL_RAD  (0.050f)
#define PITCH_TEST_SINE_ALIGN_SPEED_TOL_RAD_S (0.20f)
#define PITCH_TEST_SINE_RAMP_TIME_S         (0.50f)
#define PITCH_TEST_PI                       (3.14159265358979323846f)

/* Speeds are deliberately high enough to expose viscous drag, while the
 * 1.6 rad/s ceiling leaves margin for reversal transients. */
static const float pitch_test_speeds_rad_s[] = {
    0.30f,
    0.50f,
    0.70f,
};

#define PITCH_TEST_SPEED_COUNT \
    ((uint8_t)(sizeof(pitch_test_speeds_rad_s) / \
               sizeof(pitch_test_speeds_rad_s[0])))

volatile uint8_t g_pitch_test_state = PITCH_TEST_IDLE;
volatile uint8_t g_pitch_test_mode_select = 0u;
volatile uint8_t g_pitch_test_finished = 0u;
volatile uint8_t g_pitch_test_sample_valid = 0u;
volatile uint8_t g_pitch_test_speed_index = 0u;
volatile int8_t g_pitch_test_direction = 1;
volatile uint8_t g_pitch_test_abort_reason = 0u;
volatile uint32_t g_pitch_test_sample_count = 0u;
volatile float g_pitch_test_sweep_speed_debug = 0.0f;
volatile float g_pitch_test_target_debug = 0.0f;
volatile float g_pitch_test_target_velocity_debug = 0.0f;
volatile float g_pitch_test_target_acceleration_debug = 0.0f;
volatile float g_pitch_test_gyro_acceleration_debug = 0.0f;
volatile float g_pitch_test_elapsed_debug = 0.0f;

static float pitch_test_target = 0.0f;
static float pitch_test_state_elapsed = 0.0f;
static float pitch_test_gyro_last = 0.0f;
static uint8_t pitch_test_gyro_inited = 0u;
static uint8_t pitch_test_active = 0u;
static float pitch_test_sine_center = 0.0f;
static float pitch_test_sine_phase = 0.0f;
static float pitch_test_sine_elapsed = 0.0f;
static float pitch_test_sine_amplitude_scale = 0.0f;

static float PitchTestAbs(float value)
{
    return (value >= 0.0f) ? value : -value;
}

static float PitchTestClamp(float value, float min_value, float max_value)
{
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

static float PitchTestDt(float dt_s)
{
    if (!isfinite(dt_s) || dt_s < PITCH_TEST_MIN_DT_S ||
        dt_s > PITCH_TEST_MAX_DT_S) {
        return PITCH_TEST_DEFAULT_DT_S;
    }
    return dt_s;
}

static void PitchTestSetState(PitchTestState_e state)
{
    g_pitch_test_state = (uint8_t)state;
    pitch_test_state_elapsed = 0.0f;
    g_pitch_test_elapsed_debug = 0.0f;
}

static void PitchTestStop(float theta_meas_rad)
{
    pitch_test_active = 0u;
    g_pitch_test_state = PITCH_TEST_IDLE;
    g_pitch_test_finished = 0u;
    g_pitch_test_sample_valid = 0u;
    g_pitch_test_sweep_speed_debug = 0.0f;
    pitch_test_target = theta_meas_rad;
    g_pitch_test_target_debug = theta_meas_rad;
    g_pitch_test_target_velocity_debug = 0.0f;
    g_pitch_test_target_acceleration_debug = 0.0f;
    g_pitch_test_elapsed_debug = 0.0f;
    g_pitch_test_gyro_acceleration_debug = 0.0f;
    pitch_test_sine_phase = 0.0f;
    pitch_test_sine_elapsed = 0.0f;
    pitch_test_sine_amplitude_scale = 0.0f;
}

static void PitchTestAbort(uint8_t reason)
{
    g_pitch_test_abort_reason = reason;
    g_pitch_test_sample_valid = 0u;
    PitchTestSetState(PITCH_TEST_ABORT);
}

static void PitchTestStart(float theta_meas_rad, float gyro_rad_s)
{
    pitch_test_active = 1u;
    g_pitch_test_finished = 0u;
    g_pitch_test_sample_valid = 0u;
    g_pitch_test_speed_index = 0u;
    g_pitch_test_direction = 1;
    g_pitch_test_abort_reason = 0u;
    g_pitch_test_sample_count = 0u;
    g_pitch_test_sweep_speed_debug = pitch_test_speeds_rad_s[0];
    pitch_test_target = theta_meas_rad;
    pitch_test_sine_center = 0.0f;
    pitch_test_sine_phase = 0.0f;
    pitch_test_sine_elapsed = 0.0f;
    pitch_test_sine_amplitude_scale = 0.0f;
    g_pitch_test_target_debug = theta_meas_rad;
    pitch_test_gyro_last = gyro_rad_s;
    pitch_test_gyro_inited = isfinite(gyro_rad_s) ? 1u : 0u;
    PitchTestSetState(PITCH_TEST_ARM);
}

static void PitchTestUpdateAcceleration(float gyro_rad_s, float dt)
{
    if (!isfinite(gyro_rad_s)) {
        pitch_test_gyro_inited = 0u;
        g_pitch_test_gyro_acceleration_debug = 0.0f;
        return;
    }

    if (pitch_test_gyro_inited == 0u) {
        pitch_test_gyro_last = gyro_rad_s;
        pitch_test_gyro_inited = 1u;
        return;
    }

    const float accel_raw = (gyro_rad_s - pitch_test_gyro_last) / dt;
    g_pitch_test_gyro_acceleration_debug +=
        PITCH_TEST_ACCEL_LPF_ALPHA *
        (accel_raw - g_pitch_test_gyro_acceleration_debug);
    pitch_test_gyro_last = gyro_rad_s;
}

void PitchTest_Init(void)
{
    g_pitch_test_state = PITCH_TEST_IDLE;
    g_pitch_test_finished = 0u;
    g_pitch_test_sample_valid = 0u;
    g_pitch_test_speed_index = 0u;
    g_pitch_test_direction = 1;
    g_pitch_test_abort_reason = 0u;
    g_pitch_test_sample_count = 0u;
    g_pitch_test_sweep_speed_debug = 0.0f;
    g_pitch_test_target_debug = 0.0f;
    g_pitch_test_target_velocity_debug = 0.0f;
    g_pitch_test_target_acceleration_debug = 0.0f;
    g_pitch_test_gyro_acceleration_debug = 0.0f;
    g_pitch_test_elapsed_debug = 0.0f;
    pitch_test_target = 0.0f;
    pitch_test_state_elapsed = 0.0f;
    pitch_test_gyro_last = 0.0f;
    pitch_test_gyro_inited = 0u;
    pitch_test_active = 0u;
    pitch_test_sine_center = 0.0f;
    pitch_test_sine_phase = 0.0f;
    pitch_test_sine_elapsed = 0.0f;
}

void PitchTest_Update(float theta_meas_rad,
                      float gyro_rad_s,
                      float motor_vel_rad_s,
                      uint8_t feedback_ok,
                      float dt_s,
                      uint8_t gimbal_mode)
{
    const float dt = PitchTestDt(dt_s);

    g_pitch_test_target_velocity_debug = 0.0f;
    g_pitch_test_target_acceleration_debug = 0.0f;
    g_pitch_test_sample_valid = 0u;
    pitch_test_state_elapsed += dt;
    g_pitch_test_elapsed_debug = pitch_test_state_elapsed;
    PitchTestUpdateAcceleration(gyro_rad_s, dt);

    if (g_pitch_test_double_up == 0u) {
        PitchTestStop(theta_meas_rad);
        return;
    }
    if (gimbal_mode != GIMBAL_GYRO_MODE || feedback_ok == 0u ||
        !isfinite(theta_meas_rad) || !isfinite(gyro_rad_s)) {
        PitchTestAbort(3u);
        return;
    }
    if (pitch_test_active == 0u) {
        PitchTestStart(theta_meas_rad, gyro_rad_s);
    }

    if (g_pitch_test_state != PITCH_TEST_ABORT) {
        const float max_speed =
            (g_pitch_test_state == PITCH_TEST_SINE) ?
            PITCH_TEST_SINE_MAX_SPEED_RAD_S :
            PITCH_TEST_MAX_MEASURED_SPEED_RAD_S;
        if (PitchTestAbs(gyro_rad_s) > max_speed) {
            PitchTestAbort(2u);
        } else if (g_pitch_test_state == PITCH_TEST_SINE &&
                   PitchTestAbs(g_pitch_test_gyro_acceleration_debug) >
                   PITCH_TEST_SINE_MAX_ACCEL_RAD_S2) {
            PitchTestAbort(5u);
        }
    }

    switch ((PitchTestState_e)g_pitch_test_state) {
    case PITCH_TEST_ARM:
        pitch_test_target = theta_meas_rad;
        g_pitch_test_target_debug = pitch_test_target;
        if (pitch_test_state_elapsed >= PITCH_TEST_ARM_TIME_S) {
            if (g_pitch_test_mode_select == 1u) {
                /* First return to the pitch zero without a target step. */
                pitch_test_sine_center = 0.0f;
                pitch_test_sine_phase = 0.0f;
                pitch_test_sine_elapsed = 0.0f;
                pitch_test_sine_amplitude_scale = 0.0f;
                g_pitch_test_gyro_acceleration_debug = 0.0f;
                PitchTestSetState(PITCH_TEST_SINE_ALIGN);
            } else {
                PitchTestSetState(PITCH_TEST_MOVE_TO_START);
            }
        }
        break;

    case PITCH_TEST_SINE_ALIGN: {
        const float delta = -pitch_test_target;
        const float step = PitchTestClamp(delta,
                                          -PITCH_TEST_SINE_ALIGN_SPEED_RAD_S * dt,
                                          PITCH_TEST_SINE_ALIGN_SPEED_RAD_S * dt);

        pitch_test_target += step;
        g_pitch_test_target_velocity_debug = step / dt;
        g_pitch_test_target_debug = pitch_test_target;

        if (PitchTestAbs(pitch_test_target) <=
                PITCH_TEST_SINE_ALIGN_TARGET_TOL_RAD) {
            pitch_test_target = 0.0f;
            g_pitch_test_target_velocity_debug = 0.0f;
            g_pitch_test_target_debug = 0.0f;
            if (PitchTestAbs(theta_meas_rad) <=
                    PITCH_TEST_SINE_ALIGN_MEAS_TOL_RAD &&
                PitchTestAbs(gyro_rad_s) <=
                    PITCH_TEST_SINE_ALIGN_SPEED_TOL_RAD_S) {
                pitch_test_sine_phase = 0.0f;
                pitch_test_sine_elapsed = 0.0f;
                pitch_test_sine_amplitude_scale = 0.0f;
                PitchTestSetState(PITCH_TEST_SINE);
            }
        }
        break;
    }

    case PITCH_TEST_SINE: {
        const float w = 2.0f * PITCH_TEST_PI * PITCH_TEST_SINE_FREQUENCY_HZ;
        float scale = 1.0f;
        float scale_dot = 0.0f;
        float scale_ddot = 0.0f;

        /* Smoothly ramp the sine amplitude from zero. This keeps the target
         * velocity continuous when leaving the zero-alignment state. */
        if (pitch_test_sine_elapsed < PITCH_TEST_SINE_RAMP_TIME_S) {
            const float r = pitch_test_sine_elapsed /
                            PITCH_TEST_SINE_RAMP_TIME_S;
            scale = 3.0f * r * r - 2.0f * r * r * r;
            scale_dot = (6.0f * r - 6.0f * r * r) /
                        PITCH_TEST_SINE_RAMP_TIME_S;
            scale_ddot = (6.0f - 12.0f * r) /
                         (PITCH_TEST_SINE_RAMP_TIME_S *
                          PITCH_TEST_SINE_RAMP_TIME_S);
        }
        pitch_test_sine_amplitude_scale = scale;

        pitch_test_target = pitch_test_sine_center +
                            PITCH_TEST_SINE_AMPLITUDE_RAD *
                            scale * sinf(pitch_test_sine_phase);
        g_pitch_test_target_velocity_debug =
            PITCH_TEST_SINE_AMPLITUDE_RAD *
            (scale_dot * sinf(pitch_test_sine_phase) +
             scale * w * cosf(pitch_test_sine_phase));
        g_pitch_test_target_acceleration_debug =
            PITCH_TEST_SINE_AMPLITUDE_RAD *
            (scale_ddot * sinf(pitch_test_sine_phase) +
             2.0f * scale_dot * w * cosf(pitch_test_sine_phase) -
             scale * w * w * sinf(pitch_test_sine_phase));
        g_pitch_test_target_debug = pitch_test_target;
        g_pitch_test_sample_valid = (feedback_ok != 0u) ? 1u : 0u;
        if (g_pitch_test_sample_valid != 0u) {
            g_pitch_test_sample_count++;
        }

        pitch_test_sine_phase += w * dt;
        pitch_test_sine_elapsed += dt;
        g_pitch_test_elapsed_debug = pitch_test_sine_elapsed;
        if (pitch_test_sine_phase >=
            2.0f * PITCH_TEST_PI * PITCH_TEST_SINE_CYCLES) {
            pitch_test_target = pitch_test_sine_center;
            g_pitch_test_target_velocity_debug = 0.0f;
            g_pitch_test_target_acceleration_debug = 0.0f;
            g_pitch_test_finished = 1u;
            PitchTestSetState(PITCH_TEST_DONE);
        }
        break;
    }

    case PITCH_TEST_MOVE_TO_START: {
        const float delta = PITCH_TEST_TARGET_MIN_RAD - pitch_test_target;
        const float step = PitchTestClamp(delta,
                                          -PITCH_TEST_MOVE_TO_START_SPEED_RAD_S * dt,
                                          PITCH_TEST_MOVE_TO_START_SPEED_RAD_S * dt);
        pitch_test_target += step;
        g_pitch_test_target_velocity_debug = step / dt;
        g_pitch_test_target_debug = pitch_test_target;
        if (PitchTestAbs(pitch_test_target - PITCH_TEST_TARGET_MIN_RAD) <=
            PITCH_TEST_ENDPOINT_TOL_RAD) {
            pitch_test_target = PITCH_TEST_TARGET_MIN_RAD;
            PitchTestSetState(PITCH_TEST_START_HOLD);
        }
        break;
    }

    case PITCH_TEST_START_HOLD:
        pitch_test_target = PITCH_TEST_TARGET_MIN_RAD;
        g_pitch_test_target_debug = pitch_test_target;
        if (pitch_test_state_elapsed >= PITCH_TEST_HOLD_TIME_S) {
            g_pitch_test_direction = 1;
            g_pitch_test_sweep_speed_debug =
                pitch_test_speeds_rad_s[g_pitch_test_speed_index];
            PitchTestSetState(PITCH_TEST_SWEEP);
        }
        break;

    case PITCH_TEST_SWEEP: {
        const float speed = pitch_test_speeds_rad_s[g_pitch_test_speed_index];
        const float desired = (g_pitch_test_direction > 0) ?
                              PITCH_TEST_TARGET_MAX_RAD :
                              PITCH_TEST_TARGET_MIN_RAD;
        const float speed_tolerance = PITCH_TEST_SPEED_ERROR_BASE_RAD_S +
                                      PITCH_TEST_SPEED_ERROR_RATIO * speed;
        const float delta = desired - pitch_test_target;
        const float step = PitchTestClamp(delta,
                                          -speed * dt,
                                          speed * dt);

        g_pitch_test_sweep_speed_debug = speed;
        pitch_test_target += step;
        g_pitch_test_target_velocity_debug = step / dt;
        g_pitch_test_target_debug = pitch_test_target;

        if (feedback_ok != 0u && isfinite(theta_meas_rad) &&
            isfinite(gyro_rad_s) && isfinite(motor_vel_rad_s) &&
            pitch_test_state_elapsed >= PITCH_TEST_SETTLE_TIME_S &&
            theta_meas_rad >= PITCH_TEST_TARGET_MIN_RAD +
                              PITCH_TEST_ENDPOINT_EXCLUSION_RAD &&
            theta_meas_rad <= PITCH_TEST_TARGET_MAX_RAD -
                              PITCH_TEST_ENDPOINT_EXCLUSION_RAD &&
            PitchTestAbs(gyro_rad_s) >= PITCH_TEST_MIN_MEASURED_SPEED_RAD_S &&
            gyro_rad_s * (float)g_pitch_test_direction > 0.0f &&
            PitchTestAbs(PitchTestAbs(gyro_rad_s) - speed) <= speed_tolerance &&
            PitchTestAbs(g_pitch_test_gyro_acceleration_debug) <=
                PITCH_TEST_MAX_ACCEL_RAD_S2) {
            g_pitch_test_sample_valid = 1u;
            g_pitch_test_sample_count++;
        }

        if (PitchTestAbs(pitch_test_target - desired) <=
            PITCH_TEST_ENDPOINT_TOL_RAD) {
            pitch_test_target = desired;
            PitchTestSetState(PITCH_TEST_END_HOLD);
        }
        break;
    }

    case PITCH_TEST_END_HOLD:
        pitch_test_target = (g_pitch_test_direction > 0) ?
                            PITCH_TEST_TARGET_MAX_RAD :
                            PITCH_TEST_TARGET_MIN_RAD;
        g_pitch_test_target_debug = pitch_test_target;
        if (pitch_test_state_elapsed >= PITCH_TEST_HOLD_TIME_S) {
            if (g_pitch_test_direction > 0) {
                g_pitch_test_direction = -1;
                PitchTestSetState(PITCH_TEST_SWEEP);
            } else if (g_pitch_test_speed_index + 1u <
                       PITCH_TEST_SPEED_COUNT) {
                g_pitch_test_speed_index++;
                g_pitch_test_direction = 1;
                g_pitch_test_sweep_speed_debug =
                    pitch_test_speeds_rad_s[g_pitch_test_speed_index];
                PitchTestSetState(PITCH_TEST_SWEEP);
            } else {
                g_pitch_test_finished = 1u;
                PitchTestSetState(PITCH_TEST_DONE);
            }
        }
        break;

    case PITCH_TEST_DONE:
        pitch_test_target = PITCH_TEST_TARGET_MIN_RAD;
        g_pitch_test_target_debug = pitch_test_target;
        g_pitch_test_finished = 1u;
        break;

    case PITCH_TEST_ABORT:
        break;

    case PITCH_TEST_IDLE:
    default:
        PitchTestStop(theta_meas_rad);
        return;
    }
}

uint8_t PitchTest_IsActive(void)
{
    return pitch_test_active;
}

float PitchTest_GetTarget(void)
{
    return pitch_test_target;
}

float PitchTest_GetTargetVelocity(void)
{
    return g_pitch_test_target_velocity_debug;
}
