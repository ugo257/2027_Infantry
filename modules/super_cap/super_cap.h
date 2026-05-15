#ifndef SUPER_CUP_H
#define SUPER_CUP_H

#include "bsp_can.h"
#include "daemon.h"

#define SUPERCAP_MAX_VOLTAGE 24.0f
#define SUPERCAP_MIN_VOLTAGE 16.0f
#define SUPERCAP_HIGHER_THRESHOLD_VOLTAGE 22.0f
#define SUPERCAP_LOWER_THRESHOLD_VOLTAGE 16.0f
#define SUPERCAP_HIGHER_THRESHOLD_ENERGY 90.0f
#define SUPERCAP_LOWER_THRESHOLD_ENERGY 10.0f

typedef enum
{
    SUPERCAP_DISABLE = 0,
    SUPERCAP_ENABLE  = 1,
} SuperCap_Enable_Flag_e;

#pragma pack(1)
typedef struct
{
    uint8_t enable_flag;
    uint8_t charge_flag;
    uint8_t power_limit;
    uint8_t reserved[5];
} SuperCap_Tx_Data_s;

typedef enum
{
    SUPERCAP_STATE_DISCHARGE            = 0,
    SUPERCAP_STATE_CHARGE               = 1,
    SUPERCAP_STATE_WAIT                 = 2,
    SUPERCAP_STATE_SOFTSTART_PROTECTION = 3,
    SUPERCAP_STATE_OVER_LOAD_PROTECTION = 4,
    SUPERCAP_STATE_OVP_BAT_PROTECTION   = 5,
    SUPERCAP_STATE_UVP_BAT_PROTECTION   = 6,
    SUPERCAP_STATE_UVP_CAP_PROTECTION   = 7,
    SUPERCAP_STATE_OTP_PROTECTION       = 8,
    SUPERCAP_STATE_BOOM_PROTECTION      = 9,
    SUPERCAP_STATE_CAN_OFFLINE          = 10,
} SuperCap_State_e;

typedef struct
{
    uint8_t ready_flag;
    uint8_t SuperCapState;
    uint8_t energy;
    uint8_t chassis_real_power;
    uint8_t bat_voltage;
    uint8_t bat_power;
    uint8_t reserved[2];
} SuperCap_Rx_Data_s;
#pragma pack()

typedef struct
{
    CAN_Init_Config_s can_config;
    Daemon_Init_Config_s daemon_config;
} SuperCap_Init_Config_s;

typedef struct
{
    CANInstance *can_instance;
    DaemonInstance *daemon_instance;
    SuperCap_Tx_Data_s tx_data;
    SuperCap_Rx_Data_s rx_data;
} SuperCapInstance;

SuperCapInstance *SuperCapRegister(SuperCap_Init_Config_s *config);
SuperCapInstance *SuperCapInit(SuperCap_Init_Config_s *config);
void SuperCapTask(void);
void SuperCapEnable(SuperCapInstance *instance);
void SuperCapDisable(SuperCapInstance *instance);
void SuperCapSetPowerLimit(SuperCapInstance *instance, uint8_t power_limit);
uint8_t SuperCapIsOnline(SuperCapInstance *instance);
float SuperCapGetChassisRealPower(SuperCapInstance *instance);
uint8_t SuperCapGetCapEnergy(SuperCapInstance *instance);
uint8_t SuperCapGetReadyFlag(SuperCapInstance *instance);
float SuperCapGetChassisVoltage(SuperCapInstance *instance);

#endif
