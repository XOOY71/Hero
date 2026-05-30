#ifndef ROBOT_PARAM_H
#define ROBOT_PARAM_H

#include "struct_typedef.h"

/* Hero four-omni chassis fixed configuration. */

/* Cross-layout omni chassis: x right, y forward, wheelbase 460 mm. */
#define CHASSIS_HALF_WHEELBASE 23.0f
#define CHASSIS_OMNI_ROTATE_RADIUS CHASSIS_HALF_WHEELBASE

#define CHASSIS_WHEEL_205_DIRECTION 1.0f
#define CHASSIS_WHEEL_206_DIRECTION 1.0f
#define CHASSIS_WHEEL_207_DIRECTION 1.0f
#define CHASSIS_WHEEL_208_DIRECTION 1.0f

/* Steering motor zero offsets. */
#define CHASSIS_6020_INIT_ANGLE_0 (-0.14f)
#define CHASSIS_6020_INIT_ANGLE_1 (-0.62f)
#define CHASSIS_6020_INIT_ANGLE_2 ( 0.88f)
#define CHASSIS_6020_INIT_ANGLE_3 ( 0.20f)

#define CHASSIS_RETURN_TARGET             ( 0.72f)
#define CHASSIS_RETURN_OFFSET             ( 1.15f)
#define CHASSIS_FOLLOW_GIMBAL_YAW_OFFSET  (-2.12f)
#define CHASSIS_SPIN_OFFSET               (-1.60f)

/* Chassis motor layout by CAN ID: 205 rear, 206 right, 207 front, 208 left. */
#define WHEEL_REAR_205  0
#define WHEEL_RIGHT_206 1
#define WHEEL_FRONT_207 2
#define WHEEL_LEFT_208  3

#define CHASSIS_WZ_RC_SEN  4.352e-5f
#define CHASSIS_SPIN_SPEED 0.075f

#define NORMAL_MAX_CHASSIS_SPEED_X 2.5f
#define NORMAL_MAX_CHASSIS_SPEED_Y 2.5f

#endif
