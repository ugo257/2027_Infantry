#ifndef PITCH_IDENTIFICATION_TEST_H
#define PITCH_IDENTIFICATION_TEST_H

#include <stdint.h>

typedef enum {
    PITCH_TEST_IDLE = 0,
    PITCH_TEST_ARM = 1,
    PITCH_TEST_MOVE_TO_START = 2,
    PITCH_TEST_START_HOLD = 3,
    PITCH_TEST_SWEEP = 4,
    PITCH_TEST_END_HOLD = 5,
    PITCH_TEST_DONE = 6,
    PITCH_TEST_ABORT = 7,
    PITCH_TEST_SINE = 8,
    PITCH_TEST_SINE_ALIGN = 9,
} PitchTestState_e;

/* Both RC switches up is the only start request. Release either to stop. */
/* Mode select: 0 = normal constant-speed identification, 1 = 3 Hz sine. */
extern volatile uint8_t g_pitch_test_mode_select;
extern volatile uint8_t g_pitch_test_state;
extern volatile uint8_t g_pitch_test_finished;
extern volatile uint8_t g_pitch_test_sample_valid;
extern volatile uint8_t g_pitch_test_speed_index;
extern volatile int8_t g_pitch_test_direction;
extern volatile uint8_t g_pitch_test_abort_reason;
extern volatile uint32_t g_pitch_test_sample_count;
extern volatile float g_pitch_test_sweep_speed_debug;
extern volatile float g_pitch_test_target_debug;
extern volatile float g_pitch_test_target_velocity_debug;
extern volatile float g_pitch_test_target_acceleration_debug;
extern volatile float g_pitch_test_gyro_acceleration_debug;
extern volatile float g_pitch_test_elapsed_debug;

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
