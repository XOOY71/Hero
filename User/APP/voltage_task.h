#ifndef VOLTAGE_TASK_H
#define VOLTAGE_TASK_H

#include "struct_typedef.h"

extern fp32 battery_voltage;
uint16_t get_battery_percentage(void);
void battery_voltage_task(void const *argument);

#endif
