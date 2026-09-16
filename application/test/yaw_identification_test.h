#ifndef YAW_IDENTIFICATION_TEST_H
#define YAW_IDENTIFICATION_TEST_H

#include <stdint.h>

typedef enum {
    YAW_TEST_IDLE = 0,
    YAW_TEST_ARM = 1,
    YAW_TEST_QP_TRACKING = 2,
    YAW_TEST_DONE = 3,
    YAW_TEST_ABORT = 7,
} YawTestState_e;

/* Both RC switches up starts the closed-loop Yaw trajectory tracking test. */
extern volatile uint8_t g_yaw_test_double_up;
extern volatile uint8_t g_yaw_test_state;
extern volatile uint8_t g_yaw_test_finished;
extern volatile uint8_t g_yaw_test_sample_valid;
extern volatile uint32_t g_yaw_test_sample_count;
extern volatile float g_yaw_test_target_debug; /* deg */
extern volatile float g_yaw_test_elapsed_debug;
extern volatile float g_yaw_test_ref_rate_debug; /* rad/s */
extern volatile float g_yaw_test_ref_accel_debug; /* rad/s^2 */
extern volatile float g_yaw_test_wave_frequency_debug;
extern volatile float g_yaw_test_wave_accel_limit_debug;
extern volatile float g_yaw_test_planned_peak_to_peak_debug; /* deg */
extern volatile float g_yaw_test_planner_transition_debug; /* s */
extern volatile float yaw_id_current_injection_debug;

/* Ozone Watch parameters. They are latched when double-up starts. */
extern volatile float g_yaw_test_wave_frequency_hz;
extern volatile float g_yaw_test_wave_peak_to_peak_deg;
extern volatile float g_yaw_test_max_accel_rad_s2;
extern volatile uint8_t g_yaw_test_accel_ff_enable;

void YawTest_Init(void);
void YawTest_Update(float yaw_measure_deg,
                    float yaw_rate_rad_s,
                    uint8_t feedback_ok,
                    float dt_s,
                    uint8_t yaw_lqr_identification_ready);
uint8_t YawTest_IsActive(void);
float YawTest_GetTarget(void);
float YawTest_GetReferenceRate(void);
float YawTest_GetReferenceAcceleration(void);
uint8_t YawTest_GetAccelerationFeedforwardEnable(void);
float YawTest_GetCurrentInjection(void);

#endif
