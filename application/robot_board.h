#pragma once
#ifndef ROBOT_BOARD_H
#define ROBOT_BOARD_H

/* Board selection: keep exactly one of the following enabled */
// #define ONE_BOARD
// 双板定义修改：只需在 CHASSIS_BOARD / GIMBAL_BOARD 中启用一个
#define CHASSIS_BOARD
//#define GIMBAL_BOARD

/* Feature selection */
#if defined(CHASSIS_BOARD)
#define CHASSIS_BASIC_MOTION // Keep the chassis build limited to explicitly enabled mechanisms.
#define CHASSIS_YAW_ENABLE   // Restore only the CAN1 GM6020 yaw axis on the chassis board.
#endif
#define VISION_USE_VCP
// #define VISION_USE_UART
// #define BIG_HEAD

/* Board macro conflict check */
#if (defined(ONE_BOARD) && defined(CHASSIS_BOARD)) || \
    (defined(ONE_BOARD) && defined(GIMBAL_BOARD)) ||  \
    (defined(CHASSIS_BOARD) && defined(GIMBAL_BOARD))
#error Conflict board definition! You can only define one board type.
#endif

#if defined(CHASSIS_YAW_ENABLE) && !defined(CHASSIS_BOARD)
#error CHASSIS_YAW_ENABLE requires CHASSIS_BOARD.
#endif

#endif // ROBOT_BOARD_H
