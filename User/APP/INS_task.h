#ifndef INS_TASK_H
#define INS_TASK_H

#include "project_config.h"
#include "struct_typedef.h"
#include "hwt_imu.h"

#ifndef INS_ROLL_ADDRESS_OFFSET
#define INS_ROLL_ADDRESS_OFFSET 2
#endif

const float *get_INS_angle_point(void);

#endif
