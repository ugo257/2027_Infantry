#include <memory.h>
#include <stdlib.h>
#include "cmsis_os.h"
#include "super_cap.h"

static SuperCapInstance *supercap = NULL;

static float uint_to_float(uint32_t x_int, float x_min, float x_max, uint8_t bits)
{
    float span = x_max - x_min;
    return ((float)x_int) * span / ((float)((1u << bits) - 1u)) + x_min;
}

static float CapEnergyTransform(uint8_t raw_energy)
{
    float raw_energy_f = (float)raw_energy;
    float uvp_square   = SOFTWARE_UVP_VCAP * SOFTWARE_UVP_VCAP;
    float ovp_square   = SOFTWARE_OVP_VCAP * SOFTWARE_OVP_VCAP;
    float vcap;
    float energy;

    if (raw_energy_f > 100.0f)
        raw_energy_f = 100.0f;

    vcap   = SOFTWARE_UVP_VCAP + raw_energy_f * SUPERCAP_AVAILABLE_VOLTAGE / 100.0f;
    energy = (vcap * vcap - uvp_square) * 100.0f / (ovp_square - uvp_square);

    if (energy < 0.0f)
        return 0.0f;
    if (energy > 100.0f)
        return 100.0f;
    return energy;
}

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
    owner->chassis_real_power = uint_to_float(owner->rx_data.chassis_real_power, 0.0f, 512.0f, 16);
    owner->real_energy        = CapEnergyTransform(owner->rx_data.energy);
}

static void SuperCapLostCallback(void *instance)
{
    SuperCapInstance *owner = (SuperCapInstance *)instance;

    if (owner == NULL)
        return;

    memset(&owner->rx_data, 0, sizeof(SuperCap_Rx_Data_s));
    owner->chassis_real_power = 0.0f;
    owner->real_energy        = 0.0f;
}

uint8_t SuperCapIsOnline(SuperCapInstance *instance)
{
    if (instance == NULL)
        return 0;
    return DaemonIsOnline(instance->daemon_instance);
}

SuperCapInstance *SuperCapRegister(SuperCap_Init_Config_s *config)
{
    if (config == NULL)
        return NULL;

    if (supercap == NULL)
    {
        supercap = (SuperCapInstance *)malloc(sizeof(SuperCapInstance));
        if (supercap == NULL)
            return NULL;

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
    return instance->chassis_real_power;
}

uint8_t SuperCapGetCapEnergy(SuperCapInstance *instance)
{
    if (instance == NULL)
        return 0;
    return (uint8_t)(instance->real_energy + 0.5f);
}

float SuperCapGetRealEnergy(SuperCapInstance *instance)
{
    if (instance == NULL)
        return 0.0f;
    return instance->real_energy;
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
