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
} PitchTestState_e;

/* Ozone controls. Physical double-up is an alternate start request. */
extern volatile uint8_t g_pitch_test_enable;
extern volatile uint8_t g_pitch_test_abort;
extern volatile uint8_t g_pitch_test_state;
extern volatile uint8_t g_pitch_test_target_index;
extern volatile uint8_t g_pitch_test_finished;
extern volatile float g_pitch_test_target_debug;
extern volatile float g_pitch_test_target_velocity_debug;
extern volatile float g_pitch_test_target_acceleration_debug;
extern volatile float g_pitch_test_elapsed_debug;
extern volatile uint8_t g_pitch_test_abort_reason;

void PitchTest_Init(void);
void PitchTest_Update(float theta_meas_rad, float dt_s, uint8_t gimbal_mode);
uint8_t PitchTest_IsActive(void);
float PitchTest_GetTarget(void);

#endif
