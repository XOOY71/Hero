#ifndef CAN_RECEIVE_H
#define CAN_RECEIVE_H

#include "bsp_fdcan.h"

typedef motor_measure_t MOTOR_MEASURE_t;

void CAN_cmd_CHAS_3508(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
void CAN_cmd_CHAS_6020(int16_t motor5, int16_t motor6, int16_t motor7, int16_t motor8);
void CAN_cmd_CHASSIS_ALL(int16_t motor205, int16_t motor206, int16_t motor207, int16_t motor208);
MOTOR_MEASURE_t *get_chassis_motor_measure_point(uint8_t i);

#endif
