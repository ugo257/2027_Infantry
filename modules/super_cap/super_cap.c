#include <memory.h>
#include <stdlib.h>
#include "cmsis_os.h"
#include "super_cap.h"

static SuperCapInstance *supercap = NULL;

void SuperCapEnable(SuperCapInstance *instance)
{
    if (instance == NULL)
        return;
    instance->tx_data.enable_flag = SUPERCAP_ENABLE;
}

void SuperCapDisable(SuperCapInstance *instance)
{
    if (instance == NULL)
        return;
    instance->tx_data.enable_flag = SUPERCAP_DISABLE;
}

void SuperCapSetPowerLimit(SuperCapInstance *instance, uint8_t power_limit)
{
    if (instance == NULL)
        return;
    instance->tx_data.power_limit = power_limit;
}

static void SuperCapRxCallback(CANInstance *instance)
{
    SuperCapInstance *owner = (SuperCapInstance *)instance->id;

    if (owner == NULL)
        return;

    DaemonReload(owner->daemon_instance);
    memcpy(&owner->rx_data, instance->rx_buff, sizeof(SuperCap_Rx_Data_s));
}

static void SuperCapLostCallback(void *instance)
{
    SuperCapInstance *owner = (SuperCapInstance *)instance;

    if (owner == NULL)
        return;

    memset(&owner->rx_data, 0, sizeof(SuperCap_Rx_Data_s));
}

uint8_t SuperCapIsOnline(SuperCapInstance *instance)
{
    if (instance == NULL)
        return 0;
    return DaemonIsOnline(instance->daemon_instance);
}

SuperCapInstance *SuperCapRegister(SuperCap_Init_Config_s *config)
{
    if (supercap == NULL)
    {
        supercap = (SuperCapInstance *)malloc(sizeof(SuperCapInstance));
        memset(supercap, 0, sizeof(SuperCapInstance));

        config->can_config.id                  = supercap;
        config->can_config.can_module_callback = SuperCapRxCallback;
        supercap->can_instance                 = CANRegister(&config->can_config);

        config->daemon_config.callback     = SuperCapLostCallback;
        config->daemon_config.init_count   = 200;
        config->daemon_config.owner_id     = (void *)supercap;
        config->daemon_config.reload_count = 100;
        supercap->daemon_instance          = DaemonRegister(&config->daemon_config);
    }
    return supercap;
}

SuperCapInstance *SuperCapInit(SuperCap_Init_Config_s *config)
{
    return SuperCapRegister(config);
}

void SuperCapTask(void)
{
    static uint8_t counter = 0;

    if (supercap == NULL)
        return;

    if (counter % 8 == 0)
    {
        memcpy(supercap->can_instance->tx_buff, &supercap->tx_data, sizeof(SuperCap_Tx_Data_s));
        CANTransmit(supercap->can_instance, 1);
    }
    counter++;
}

float SuperCapGetChassisRealPower(SuperCapInstance *instance)
{
    if (instance == NULL)
        return 0.0f;
    return (float)(instance->rx_data.chassis_real_power << 1);
}

uint8_t SuperCapGetCapEnergy(SuperCapInstance *instance)
{
    if (instance == NULL)
        return 0;
    return instance->rx_data.energy;
}

uint8_t SuperCapGetReadyFlag(SuperCapInstance *instance)
{
    if (instance == NULL)
        return 0;
    return instance->rx_data.ready_flag;
}

float SuperCapGetChassisVoltage(SuperCapInstance *instance)
{
    if (instance == NULL)
        return 0.0f;
    return (float)instance->rx_data.bat_voltage / 10.0f;
}
