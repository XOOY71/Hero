#ifndef ROBOT_PARAM_H
#define ROBOT_PARAM_H

#include "struct_typedef.h"

/* Hero four-omni chassis fixed configuration. */

/*
 * Chassis geometry for inverse kinematics.
 * Unit follows the existing chassis controller scale: centimeter-level length.
 * A 400 mm diagonal square gives x/y projection from center to each diagonal wheel:
 * 400 / 2 / sqrt(2) = 141.421 mm = 14.1421 in this controller scale.
 */
#define HALF_LENGTH 14.142136f
#define HALF_WIDTH  14.142136f
#define CHASSIS_OMNI_ROTATE_RADIUS (HALF_LENGTH + HALF_WIDTH)

#define CHASSIS_WHEEL_LF_DIRECTION 1.0f
#define CHASSIS_WHEEL_LB_DIRECTION 1.0f
#define CHASSIS_WHEEL_RB_DIRECTION 1.0f
#define CHASSIS_WHEEL_RF_DIRECTION 1.0f

/* Steering motor zero offsets. */
#define CHASSIS_6020_INIT_ANGLE_0 (-0.14f)
#define CHASSIS_6020_INIT_ANGLE_1 (-0.62f)
#define CHASSIS_6020_INIT_ANGLE_2 ( 0.88f)
#define CHASSIS_6020_INIT_ANGLE_3 ( 0.20f)

#define CHASSIS_RETURN_TARGET             ( 0.72f)
#define CHASSIS_RETURN_OFFSET             ( 1.15f)
#define CHASSIS_FOLLOW_GIMBAL_YAW_OFFSET  (-2.12f)
#define CHASSIS_SPIN_OFFSET               (-1.60f)

#define WHEEL_RF 3
#define WHEEL_LF 0
#define WHEEL_LB 1
#define WHEEL_RB 2

#define CHASSIS_WZ_RC_SEN  4.352e-5f
#define CHASSIS_SPIN_SPEED 0.075f

#define NORMAL_MAX_CHASSIS_SPEED_X 2.5f
#define NORMAL_MAX_CHASSIS_SPEED_Y 2.5f

#endif
