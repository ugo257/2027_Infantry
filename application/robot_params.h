#pragma once
#ifndef ROBOT_PARAMS_H
#define ROBOT_PARAMS_H

#include <stdint.h>
#include "robot_board.h"

#define NUC_RECV_SIZE 29

/* Gimbal/chassis install & limits */
#define PITCH_POS_MAX_ECD         4650
#define PITCH_POS_MIN_ECD         5700
#define YAW_CHASSIS_ALIGN_ECD     621
#define YAW_ECD_GREATER_THAN_4096 0
#define PITCH_HORIZON_ECD         5225
#define PITCH_POS_UP_LIMIT_ECD    6191
#define PITCH_POS_DOWN_LIMIT_ECD  4830
#define JOINT_LEFT_UP_LIMIT       3.40f
#define JOINT_LEFT_DOWN_LIMIT     0.54f
#define JOINT_RIGHT_UP_LIMIT      -3.40f
#define JOINT_RIGHT_DOWN_LIMIT    -0.43f
#define PITCH_DOWN_LIMIT          -0.4f
#define PITCH_UP_LIMIT            0.4f

#define LOADER_ANGLE_PER_BULLET 3240.0f

#define PITCH_FEED_TYPE     1
#define PITCH_INS_FEED_TYPE 1
#define PITCH_ECD_UP_ADD    1

/* Shoot */
#define ONE_BULLET_DELTA_ANGLE 1330
#define REDUCTION_RATIO_LOADER 19.0f
#define NUM_PER_CIRCLE         6

/* Chassis mechanical params */
#define WHEEL_BASE             414.5f
#define TRACK_WIDTH            361.0f
#define CENTER_GIMBAL_OFFSET_X 0
#define CENTER_GIMBAL_OFFSET_Y 0
#define RADIUS_WHEEL           77.0f
#define REDUCTION_RATIO_WHEEL  19.0f
#define SYNC_BELT_SWITCH_SPEED_REF 5000.0f
#define SYNC_BELT_LEFT_REF_SIGN      1.0f
#define SYNC_BELT_RIGHT_REF_SIGN     -1.0f

/* Control gains & limits */
#define CHASSIS_SPEED 60000
#define CHASSIS_ROTATE_WHEEL_REF_LIMIT        ((float)CHASSIS_SPEED)
#define CHASSIS_ROTATE_WZ_RESERVE_RATIO       1.00f
#define CHASSIS_ROTATE_VXY_POWER_COEF         1.00f
#define CHASSIS_ROTATE_VXY_MIN_SCALE          0.20f
#define CHASSIS_ROTATE_VXY_SCALE_FILTER_ALPHA 0.15f
#define YAW_K         0.00025f
#define PITCH_K       0.000004f

#define BUZZER_SILENCE 0

/* BMI088 calibration defaults */
#define BMI088_PRE_CALI_GYRO_X_OFFSET 0.00142266625f
#define BMI088_PRE_CALI_GYRO_Y_OFFSET 0.000708730659f
#define BMI088_PRE_CALI_GYRO_Z_OFFSET 0.000225723968f
#define BMI088_AMBIENT_TEMPERATURE    25.0f

#define BMI088_BOARD_INSTALL_SPIN_MATRIX \
    {-1.0f, 0.0f, 0.0f},                 \
    {0.0f, -1.0f, 0.0f},                 \
    {0.0f, 0.0f, 1.0f}

#define INS_YAW_ADDRESS_OFFSET   2
#define INS_PITCH_ADDRESS_OFFSET 1
#define INS_ROLL_ADDRESS_OFFSET  0

#define IMU_DEF_PARAM_WARNING
#ifndef IMU_DEF_PARAM_WARNING
#define IMU_DEF_PARAM_WARNING
#pragma message "check if you have configured the parameters in robot_params.h, IF NOT, please refer to the comments AND DO IT, otherwise the robot will have FATAL ERRORS!!!"
#endif

#endif // ROBOT_PARAMS_H
