#include "voltage_task.h"

#include "cmsis_os.h"

fp32 battery_voltage = 0.0f;
static fp32 battery_percentage = 0.0f;

void battery_voltage_task(void const *argument)
{
    (void)argument;

    while (1)
    {
        osDelay(100U);
    }
}

uint16_t get_battery_percentage(void)
{
    return (uint16_t)(battery_percentage * 100.0f);
}
