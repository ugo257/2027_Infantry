#ifndef YAW_IDENTIFICATION_TEST_H
#define YAW_IDENTIFICATION_TEST_H

#include <stdint.h>

typedef enum {
    YAW_TEST_IDLE = 0,
    YAW_TEST_ARM = 1,
    YAW_TEST_PRBS_LOW = 2,
    YAW_TEST_PRBS_HIGH = 3,
    YAW_TEST_CURRENT_CHIRP = 4,
    YAW_TEST_RECOVER = 5,
    YAW_TEST_DONE = 6,
    YAW_TEST_ABORT = 7,
} YawTestState_e;

/* Both RC switches up starts the closed-loop Yaw current-identification test. */
extern volatile uint8_t g_yaw_test_double_up;
extern volatile uint8_t g_yaw_test_state;
extern volatile uint8_t g_yaw_test_finished;
extern volatile uint8_t g_yaw_test_sample_valid;
extern volatile uint32_t g_yaw_test_sample_count;
extern volatile float g_yaw_test_target_debug;
extern volatile float g_yaw_test_elapsed_debug;
extern volatile float g_yaw_test_chirp_frequency_debug;
extern volatile float yaw_id_current_injection_debug;

void YawTest_Init(void);
void YawTest_Update(float yaw_measure_deg,
                    float yaw_rate_rad_s,
                    uint8_t feedback_ok,
                    float dt_s,
                    uint8_t yaw_lqr_identification_ready);
uint8_t YawTest_IsActive(void);
float YawTest_GetTarget(void);
float YawTest_GetCurrentInjection(void);

#endif
