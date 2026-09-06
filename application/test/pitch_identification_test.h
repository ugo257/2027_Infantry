#ifndef PITCH_IDENTIFICATION_TEST_H
#define PITCH_IDENTIFICATION_TEST_H

#include <stdint.h>

typedef enum {
    PITCH_TEST_IDLE = 0,
    PITCH_TEST_ARM = 1,
    PITCH_TEST_MOVE = 2,
    PITCH_TEST_HOLD = 3,
    PITCH_TEST_DONE = 4,
    PITCH_TEST_ABORT = 5,
    PITCH_TEST_DYNAMIC = 6,
} PitchTestState_e;

/* Ozone controls. Physical double-up is an alternate start request.
 * Normal mode performs the +/-20 degree identification scan. Set
 * g_pitch_test_signal_check before starting to use the smaller +/-5 degree
 * signal/feedback direction check instead. Set g_pitch_test_dynamic_enable
 * for the smooth dynamic excitation used for J/B identification; it takes
 * precedence over signal-check mode. */
extern volatile uint8_t g_pitch_test_enable;
extern volatile uint8_t g_pitch_test_abort;
extern volatile uint8_t g_pitch_test_state;
extern volatile uint8_t g_pitch_test_target_index;
extern volatile int8_t g_pitch_test_direction;
extern volatile uint8_t g_pitch_test_finished;
extern volatile uint8_t g_pitch_test_sample_valid;
extern volatile uint32_t g_pitch_test_sample_count;
extern volatile float g_pitch_test_target_debug;
extern volatile float g_pitch_test_target_velocity_debug;
extern volatile float g_pitch_test_target_acceleration_debug;
extern volatile float g_pitch_test_gyro_acceleration_debug;
extern volatile float g_pitch_test_elapsed_debug;
extern volatile float g_pitch_test_stable_elapsed_debug;
extern volatile uint8_t g_pitch_test_abort_reason;
extern volatile uint8_t g_pitch_test_signal_check;
extern volatile uint8_t g_pitch_test_sign_fault;
extern volatile uint32_t g_pitch_test_sign_mismatch_count;
extern volatile uint8_t g_pitch_test_dynamic_enable;
/* 0: normal scan, 1: signal check, 2: dynamic J/B excitation. */
extern volatile uint8_t g_pitch_test_mode_debug;

void PitchTest_Init(void);
void PitchTest_Update(float theta_meas_rad,
                      float gyro_rad_s,
                      float motor_vel_rad_s,
                      uint8_t feedback_ok,
                      float dt_s,
                      uint8_t gimbal_mode);
uint8_t PitchTest_IsActive(void);
float PitchTest_GetTarget(void);
float PitchTest_GetTargetVelocity(void);

#endif
