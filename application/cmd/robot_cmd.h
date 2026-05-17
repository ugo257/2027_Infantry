#ifndef ROBOT_CMD_H
#define ROBOT_CMD_H

#include <stdint.h>

#define ROBOTCMD_VISION_FIRE_SINGLE 1u
#define ROBOTCMD_VISION_FIRE_TRIPLE 3u

/**
 * @brief 机器人核心控制任务初始化,会被RobotInit()调用
 * 
 */
void RobotCMDInit();

/**
 * @brief 机器人核心控制任务，当前由 FreeRTOS 以 1ms 周期调度（约1000Hz）
 * 
 */
void RobotCMDTask();

/**
 * @brief 麦轮力控开关接口（与 chassis_mode 解耦）
 * @param enable 0:关闭  非0:开启
 */
void RobotCMDSetMecanumForceCtrl(uint8_t enable);

/**
 * @brief 获取麦轮力控开关状态
 */
uint8_t RobotCMDGetMecanumForceCtrl(void);

void RobotCMDSetVisionAngleFireEnable(uint8_t enable);
uint8_t RobotCMDGetVisionAngleFireEnable(void);
void RobotCMDSetVisionAngleFireBullets(uint8_t bullets);
uint8_t RobotCMDGetVisionAngleFireBullets(void);

extern  float cym1;

#endif // !ROBOT_CMD_H
