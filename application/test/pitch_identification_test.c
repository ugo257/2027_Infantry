#include "pitch_identification_test.h"

#include <math.h>

#include "robot_types.h"
#include "robot_cmd.h"

#define PITCH_TEST_MIN_DT_S          (0.0001f)
#define PITCH_TEST_MAX_DT_S          (0.01f)
#define PITCH_TEST_DEFAULT_DT_S      (0.001f)
#define PITCH_TEST_ARM_TIME_S        (2.0f)
#define PITCH_TEST_HOLD_TIME_S       (0.50f)
#define PITCH_TEST_TARGET_SPEED_RAD_S (0.10f)
#define PITCH_TEST_TARGET_MIN_RAD    (-0.349066f) /* -20 deg */
#define PITCH_TEST_TARGET_MAX_RAD    (0.349066f)  /* +20 deg */
#define PITCH_TEST_TARGET_DEADBAND_RAD (0.002f)
#define PITCH_TEST_STABLE_ANGLE_RAD  (0.005f)
#define PITCH_TEST_STABLE_GYRO_RAD_S (0.05f)
#define PITCH_TEST_STABLE_MOTOR_VEL_RAD_S (0.10f)
#define PITCH_TEST_STABLE_TIME_S     (0.30f)
#define PITCH_TEST_MOTION_MIN_GYRO_RAD_S (0.03f)
#define PITCH_TEST_MOTION_MAX_ACCEL_RAD_S2 (1.00f)
#define PITCH_TEST_MOTION_MIN_MOTOR_VEL_RAD_S (0.05f)
#define PITCH_TEST_MOTION_SPEED_TOL_RAD_S (0.05f)
#define PITCH_TEST_GYRO_ACCEL_LPF_ALPHA (0.20f)

volatile uint8_t g_pitch_test_enable = 0u;
volatile uint8_t g_pitch_test_abort = 0u;
volatile uint8_t g_pitch_test_state = PITCH_TEST_IDLE;
volatile uint8_t g_pitch_test_target_index = 0u;
volatile int8_t g_pitch_test_direction = 1;
volatile uint8_t g_pitch_test_finished = 0u;
volatile uint8_t g_pitch_test_sample_valid = 0u;
volatile uint32_t g_pitch_test_sample_count = 0u;
volatile float g_pitch_test_target_debug = 0.0f;
volatile float g_pitch_test_target_velocity_debug = 0.0f;
volatile float g_pitch_test_target_acceleration_debug = 0.0f;
volatile float g_pitch_test_gyro_acceleration_debug = 0.0f;
volatile float g_pitch_test_elapsed_debug = 0.0f;
volatile float g_pitch_test_stable_elapsed_debug = 0.0f;
volatile uint8_t g_pitch_test_abort_reason = 0u;

/* Scan endpoints over the requested +/-20-degree working range. The target
 * is ramped continuously; static hold samples are not used for the gravity
 * fit because the mechanism has substantial stiction. */
static const float pitch_test_targets[] = {
    -0.349066f, /* -20 deg */
    0.349066f, /* +20 deg */
};
#define PITCH_TEST_TARGET_COUNT \
    ((uint8_t)(sizeof(pitch_test_targets) / sizeof(pitch_test_targets[0])))

static float pitch_test_target = 0.0f;
static float pitch_test_hold_elapsed = 0.0f;
static float pitch_test_stable_elapsed = 0.0f;
static float pitch_test_arm_elapsed = 0.0f;
static float pitch_test_start_angle = 0.0f;
static float pitch_test_gyro_last = 0.0f;
static uint8_t pitch_test_gyro_inited = 0u;
static uint8_t pitch_test_active = 0u;

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

static float PitchTestAbs(float value)
{
    return (value >= 0.0f) ? value : -value;
}

static float PitchTestDt(float dt_s)
{
    if (!isfinite(dt_s) || dt_s < PITCH_TEST_MIN_DT_S ||
        dt_s > PITCH_TEST_MAX_DT_S) {
        return PITCH_TEST_DEFAULT_DT_S;
    }
    return dt_s;
}

static void PitchTestPublish(void)
{
    g_pitch_test_target_debug = pitch_test_target;
    g_pitch_test_elapsed_debug =
        (g_pitch_test_state == PITCH_TEST_ARM) ? pitch_test_arm_elapsed :
        pitch_test_hold_elapsed;
    g_pitch_test_stable_elapsed_debug = pitch_test_stable_elapsed;
}

void PitchTest_Init(void)
{
    g_pitch_test_state = PITCH_TEST_IDLE;
    g_pitch_test_target_index = 0u;
    g_pitch_test_direction = 1;
    g_pitch_test_finished = 0u;
    g_pitch_test_sample_valid = 0u;
    g_pitch_test_sample_count = 0u;
    g_pitch_test_abort_reason = 0u;
    pitch_test_target = 0.0f;
    pitch_test_hold_elapsed = 0.0f;
    pitch_test_stable_elapsed = 0.0f;
    pitch_test_arm_elapsed = 0.0f;
    pitch_test_gyro_last = 0.0f;
    pitch_test_gyro_inited = 0u;
    pitch_test_active = 0u;
    g_pitch_test_gyro_acceleration_debug = 0.0f;
    PitchTestPublish();
}

void PitchTest_Update(float theta_meas_rad,
                      float gyro_rad_s,
                      float motor_vel_rad_s,
                      uint8_t feedback_ok,
                      float dt_s,
                      uint8_t gimbal_mode)
{
    const float dt = PitchTestDt(dt_s);
    const uint8_t run_request =
        (g_pitch_test_enable != 0u || g_pitch_test_double_up != 0u) ? 1u : 0u;

    g_pitch_test_target_velocity_debug = 0.0f;
    g_pitch_test_target_acceleration_debug = 0.0f;
    g_pitch_test_sample_valid = 0u;

    if (isfinite(gyro_rad_s) != 0) {
        if (pitch_test_gyro_inited == 0u) {
            pitch_test_gyro_last = gyro_rad_s;
            pitch_test_gyro_inited = 1u;
        } else {
            const float accel_raw = (gyro_rad_s - pitch_test_gyro_last) / dt;
            g_pitch_test_gyro_acceleration_debug +=
                PITCH_TEST_GYRO_ACCEL_LPF_ALPHA *
                (accel_raw - g_pitch_test_gyro_acceleration_debug);
            pitch_test_gyro_last = gyro_rad_s;
        }
    } else {
        pitch_test_gyro_inited = 0u;
        g_pitch_test_gyro_acceleration_debug = 0.0f;
    }

    if (g_pitch_test_abort != 0u) {
        pitch_test_active = 0u;
        g_pitch_test_state = PITCH_TEST_ABORT;
        g_pitch_test_abort_reason = 1u;
        g_pitch_test_sample_valid = 0u;
        PitchTestPublish();
        return;
    }

    if (pitch_test_active == 0u) {
        if (run_request == 0u || gimbal_mode != GIMBAL_GYRO_MODE) {
            g_pitch_test_state = PITCH_TEST_IDLE;
            g_pitch_test_finished = 0u;
            PitchTestPublish();
            return;
        }
        pitch_test_active = 1u;
        g_pitch_test_state = PITCH_TEST_ARM;
        g_pitch_test_target_index = 0u;
        g_pitch_test_direction = 1;
        g_pitch_test_finished = 0u;
        g_pitch_test_sample_count = 0u;
        g_pitch_test_abort_reason = 0u;
        pitch_test_start_angle = theta_meas_rad;
        pitch_test_target = theta_meas_rad;
        pitch_test_arm_elapsed = 0.0f;
        pitch_test_hold_elapsed = 0.0f;
        pitch_test_stable_elapsed = 0.0f;
        pitch_test_gyro_last = gyro_rad_s;
        pitch_test_gyro_inited = (isfinite(gyro_rad_s) != 0) ? 1u : 0u;
    }

    if (run_request == 0u || gimbal_mode != GIMBAL_GYRO_MODE) {
        pitch_test_active = 0u;
        g_pitch_test_state = PITCH_TEST_ABORT;
        g_pitch_test_abort_reason = 2u;
        g_pitch_test_sample_valid = 0u;
        PitchTestPublish();
        return;
    }

    switch ((PitchTestState_e)g_pitch_test_state) {
    case PITCH_TEST_ARM:
        pitch_test_target = pitch_test_start_angle;
        pitch_test_arm_elapsed += dt;
        if (pitch_test_arm_elapsed >= PITCH_TEST_ARM_TIME_S) {
            g_pitch_test_state = PITCH_TEST_MOVE;
            /* Keep the current command for this transition; MOVE will ramp
             * from the armed position to the first grid point on the next
             * tick instead of creating a target step. */
            pitch_test_target = pitch_test_start_angle;
        }
        break;

    case PITCH_TEST_MOVE: {
        const float desired = PitchTestClamp(
            pitch_test_targets[g_pitch_test_target_index],
            PITCH_TEST_TARGET_MIN_RAD,
            PITCH_TEST_TARGET_MAX_RAD);
        const float delta = desired - pitch_test_target;
        const float max_delta = PITCH_TEST_TARGET_SPEED_RAD_S * dt;

        pitch_test_target += PitchTestClamp(delta, -max_delta, max_delta);
        g_pitch_test_target_velocity_debug =
            PitchTestClamp(delta / dt,
                           -PITCH_TEST_TARGET_SPEED_RAD_S,
                           PITCH_TEST_TARGET_SPEED_RAD_S);

        /* Use only moving, low-acceleration samples for gravity fitting. The
         * positive/negative sweeps are averaged offline to reject friction. */
        if (feedback_ok != 0u &&
            isfinite(theta_meas_rad) != 0 &&
            isfinite(gyro_rad_s) != 0 &&
            isfinite(motor_vel_rad_s) != 0 &&
            PitchTestAbs(g_pitch_test_target_velocity_debug) >=
                PITCH_TEST_MOTION_MIN_GYRO_RAD_S &&
            PitchTestAbs(gyro_rad_s) >= PITCH_TEST_MOTION_MIN_GYRO_RAD_S &&
            gyro_rad_s * g_pitch_test_target_velocity_debug > 0.0f &&
            PitchTestAbs(PitchTestAbs(gyro_rad_s) -
                         PitchTestAbs(g_pitch_test_target_velocity_debug)) <=
                PITCH_TEST_MOTION_SPEED_TOL_RAD_S &&
            PitchTestAbs(motor_vel_rad_s) >=
                PITCH_TEST_MOTION_MIN_MOTOR_VEL_RAD_S &&
            PitchTestAbs(g_pitch_test_gyro_acceleration_debug) <=
                PITCH_TEST_MOTION_MAX_ACCEL_RAD_S2) {
            g_pitch_test_sample_valid = 1u;
            g_pitch_test_sample_count++;
        }

        if (PitchTestAbs(desired - pitch_test_target) <=
            PITCH_TEST_TARGET_DEADBAND_RAD) {
            pitch_test_target = desired;
            pitch_test_hold_elapsed = 0.0f;
            g_pitch_test_state = PITCH_TEST_HOLD;
        }
        break;
    }

    case PITCH_TEST_HOLD:
        pitch_test_target = PitchTestClamp(
            pitch_test_targets[g_pitch_test_target_index],
            PITCH_TEST_TARGET_MIN_RAD,
            PITCH_TEST_TARGET_MAX_RAD);
        pitch_test_hold_elapsed += dt;
        /* Endpoint dwell lets the mechanism settle before reversing; these
         * rows are intentionally invalid for the dynamic gravity fit. */
        if (pitch_test_hold_elapsed >= PITCH_TEST_HOLD_TIME_S) {
            /* Reverse at each endpoint. This gives both approach directions
             * for the same angle, which is useful for rejecting friction and
             * backlash during offline fitting. */
            if (g_pitch_test_direction > 0 &&
                g_pitch_test_target_index + 1u >= PITCH_TEST_TARGET_COUNT) {
                g_pitch_test_direction = -1;
                g_pitch_test_target_index--;
            } else if (g_pitch_test_direction < 0 &&
                       g_pitch_test_target_index == 0u) {
                g_pitch_test_direction = 1;
                g_pitch_test_target_index++;
            } else {
                g_pitch_test_target_index =
                    (uint8_t)((int16_t)g_pitch_test_target_index +
                              (int16_t)g_pitch_test_direction);
            }
            g_pitch_test_state = PITCH_TEST_MOVE;
            pitch_test_hold_elapsed = 0.0f;
            pitch_test_stable_elapsed = 0.0f;
            g_pitch_test_sample_valid = 0u;
        }
        break;

    case PITCH_TEST_DONE:
        /* Hold the final point. There is intentionally no automatic return. */
        pitch_test_target = PitchTestClamp(
            pitch_test_targets[g_pitch_test_target_index],
            PITCH_TEST_TARGET_MIN_RAD,
            PITCH_TEST_TARGET_MAX_RAD);
        break;

    case PITCH_TEST_ABORT:
    case PITCH_TEST_IDLE:
    default:
        pitch_test_active = 0u;
        g_pitch_test_sample_valid = 0u;
        break;
    }

    PitchTestPublish();
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
