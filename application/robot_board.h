#pragma once
#ifndef ROBOT_BOARD_H
#define ROBOT_BOARD_H

/* Board selection: keep exactly one of the following enabled */
// #define ONE_BOARD
//双板定义修改
#define CHASSIS_BOARD
//#define GIMBAL_BOARD

/* Feature selection */
#define VISION_USE_VCP
// #define VISION_USE_UART
// #define BIG_HEAD

/* Board macro conflict check */
#if (defined(ONE_BOARD) && defined(CHASSIS_BOARD)) || \
    (defined(ONE_BOARD) && defined(GIMBAL_BOARD)) ||  \
    (defined(CHASSIS_BOARD) && defined(GIMBAL_BOARD))
#error Conflict board definition! You can only define one board type.
#endif

#endif // ROBOT_BOARD_H
