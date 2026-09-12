#ifndef YAW_IDENTIFICATION_TEST_H
#define YAW_IDENTIFICATION_TEST_H

#include <stdint.h>

typedef enum {
    YAW_TEST_IDLE = 0,
    YAW_TEST_ARM = 1,
    YAW_TEST_CENTER = 2,
    YAW_TEST_STEP = 3,
    YAW_TEST_CHIRP = 4,
    YAW_TEST_DONE = 5,
} YawTestState_e;

/* Both RC switches up starts the deterministic Yaw reference trajectory. */
extern volatile uint8_t g_yaw_test_double_up;
extern volatile uint8_t g_yaw_test_state;
extern volatile uint8_t g_yaw_test_finished;
extern volatile uint8_t g_yaw_test_sample_valid;
extern volatile uint8_t g_yaw_test_step_index;
extern volatile uint32_t g_yaw_test_sample_count;
extern volatile float g_yaw_test_target_debug;
extern volatile float g_yaw_test_target_offset_debug;
extern volatile float g_yaw_test_target_velocity_debug;
extern volatile float g_yaw_test_target_acceleration_debug;
extern volatile float g_yaw_test_elapsed_debug;
extern volatile float g_yaw_test_chirp_frequency_debug;

void YawTest_Init(void);
void YawTest_Update(float yaw_measure_deg,
                    float yaw_rate_deg_s,
                    uint8_t feedback_ok,
                    float dt_s,
                    uint8_t gimbal_mode);
uint8_t YawTest_IsActive(void);
float YawTest_GetTarget(void);

#endif
