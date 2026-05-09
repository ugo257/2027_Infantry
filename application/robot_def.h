#pragma once
#ifndef ROBOT_DEF_H
#define ROBOT_DEF_H

/* compatibility entry: keep old include path alive */
#include "robot_board.h"
#include "robot_params.h"
#include "robot_types.h"

/*
 * Gimbal sliding mode controller switch.
 * Set the corresponding macro to 0 to return to the original cascaded PID chain only.
 * When enabled, SMC is added as current/torque feedforward.
 */
#ifndef GIMBAL_YAW_SMC_ENABLE
#define GIMBAL_YAW_SMC_ENABLE 1
#endif

#ifndef GIMBAL_PITCH_SMC_ENABLE
#define GIMBAL_PITCH_SMC_ENABLE 1
#endif

#define GIMBAL_SMC_CTRL_DT 0.001f

/* Yaw: degree / degree per second domain, output is GM6020 current feedforward. */
#define GIMBAL_YAW_SMC_OUTPUT_SIGN  -1.0f
#define GIMBAL_YAW_SMC_LAMBDA       24.0f
#define GIMBAL_YAW_SMC_KI            0.0f
#define GIMBAL_YAW_SMC_LINEAR_K     10.0f
#define GIMBAL_YAW_SMC_SWITCH_K   2100.0f
#define GIMBAL_YAW_SMC_BOUNDARY     18.0f
#define GIMBAL_YAW_SMC_OUT_LIMIT  5200.0f
#define GIMBAL_YAW_SMC_FILTER        0.35f
#define GIMBAL_YAW_SMC_INT_LIMIT    12.0f
#define GIMBAL_YAW_SMC_REF_VEL_FILTER 0.15f
#define GIMBAL_YAW_SMC_GYRO_TO_DEG  57.2957795f
#define GIMBAL_YAW_SMC_ERR_DEADBAND      0.03f
#define GIMBAL_YAW_SMC_VEL_DEADBAND      0.10f
#define GIMBAL_YAW_SMC_SURFACE_DEADBAND  0.20f
#define GIMBAL_YAW_SMC_STARTUP_STEP      0.010f

/* Pitch: rad / rad per second domain, output is DM torque feedforward. */
#define GIMBAL_PITCH_SMC_OUTPUT_SIGN   1.0f
#define GIMBAL_PITCH_SMC_LAMBDA        7.0f
#define GIMBAL_PITCH_SMC_KI            0.0f
#define GIMBAL_PITCH_SMC_LINEAR_K      0.035f
#define GIMBAL_PITCH_SMC_SWITCH_K      0.030f
#define GIMBAL_PITCH_SMC_BOUNDARY      1.5f
#define GIMBAL_PITCH_SMC_OUT_LIMIT     0.30f
#define GIMBAL_PITCH_SMC_FILTER        0.16f
#define GIMBAL_PITCH_SMC_INT_LIMIT     0.05f
#define GIMBAL_PITCH_SMC_REF_VEL_FILTER 0.0f
#define GIMBAL_PITCH_SMC_ERR_DEADBAND      0.010f
#define GIMBAL_PITCH_SMC_VEL_DEADBAND      0.080f
#define GIMBAL_PITCH_SMC_SURFACE_DEADBAND  0.120f
#define GIMBAL_PITCH_SMC_STARTUP_STEP      0.0030f
#define GIMBAL_PITCH_SMC_LINKAGE_ERR_GATE  0.015f
#define GIMBAL_PITCH_SMC_LINKAGE_GYRO_GATE 0.120f
#define GIMBAL_PITCH_LINKAGE_CRANK_ZERO_RAD 0.0f
#define GIMBAL_PITCH_LINKAGE_DEADZONE_SIN   0.25f
#define GIMBAL_PITCH_FF_TOTAL_MIN     -3.2f
#define GIMBAL_PITCH_FF_TOTAL_MAX      3.2f
#define GIMBAL_PITCH_ZERO_FORCE_SMC_GAIN 0.25f

/*
 * Keep pitch softly braked during command-layer emergency stop.
 * Set to 0 if GIMBAL_ZERO_FORCE must be true zero torque for safety tests.
 */
#define GIMBAL_PITCH_ZERO_FORCE_HOLD_ENABLE 1

/*
 * Pitch linkage gravity feedforward fitted from Ozone_DataSampling_260503.csv
 * and corrected near pos ~= 2.01 rad by Ozone_DataSampling_260504.csv.
 * Low-speed holding torque is fitted against DM motor output position:
 * torque = C0 + C1 * (pos - CENTER) + C2 * (pos - CENTER)^2.
 */
#define GIMBAL_PITCH_GRAVITY_USE_MOTOR_POS_FIT 1
#define GIMBAL_PITCH_GRAVITY_POS_CENTER        1.970000f
#define GIMBAL_PITCH_GRAVITY_FIT_C0            0.275000f
#define GIMBAL_PITCH_GRAVITY_FIT_C1           -0.540475f
#define GIMBAL_PITCH_GRAVITY_FIT_C2            0.752579f
#define GIMBAL_PITCH_GRAVITY_FIT_MIN           0.080000f
#define GIMBAL_PITCH_GRAVITY_FIT_MAX           0.550000f
#define GIMBAL_PITCH_GRAVITY_DAMPING_GAIN      0.100000f
#define GIMBAL_PITCH_MOTOR_VEL_DAMPING_GAIN    0.520000f
#define GIMBAL_PITCH_MOTOR_VEL_DAMPING_LIMIT   3.000000f

/* Stiffer PID for linkage-driven pitch, with extra damping for moving stops. */
#define GIMBAL_PITCH_ANGLE_KP                  6.00f
#define GIMBAL_PITCH_ANGLE_KI                  0.0f
#define GIMBAL_PITCH_ANGLE_KD                  0.080f
#define GIMBAL_PITCH_ANGLE_DEADBAND            0.0015f
#define GIMBAL_PITCH_ANGLE_MAXOUT              10.0f
#define GIMBAL_PITCH_ANGLE_INTEGRAL_LIMIT      0.15f
#define GIMBAL_PITCH_ANGLE_OUTPUT_FILTER       0.85f
#define GIMBAL_PITCH_ANGLE_DERIVATIVE_FILTER   0.45f
#define GIMBAL_PITCH_SPEED_KP                  1.20f
#define GIMBAL_PITCH_SPEED_KI                  0.0f
#define GIMBAL_PITCH_SPEED_KD                  0.0f
#define GIMBAL_PITCH_SPEED_DEADBAND            0.015f
#define GIMBAL_PITCH_SPEED_MAXOUT              3.40f
#define GIMBAL_PITCH_SPEED_INTEGRAL_LIMIT      0.20f
#define GIMBAL_PITCH_SPEED_OUTPUT_FILTER       0.75f
#define GIMBAL_PITCH_SPEED_DERIVATIVE_FILTER   0.35f

#endif // ROBOT_DEF_H
