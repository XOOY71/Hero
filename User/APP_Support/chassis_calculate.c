#include "bsp_usart.h"
#include "chassis_calculate.h"
#include "chassis_task.h"
#include "CAN_receive.h"
#include "robot_param.h"
#include "user_lib.h"
#include <math.h>
#include "stdlib.h"

void chassis_wheel_angle_offset_init(void)
{
	for(uint8_t i = 0; i < CHASSIS_MODULE_NUM; i++)
	{
		chassis_move.wheel_angle_offset.now[i] = 0.0f;
		chassis_move.wheel_angle_offset.last[i] = 0.0f;
		chassis_move.wheel_angle_offset.initial[i] = 0.0f;
	}
}

void vector_rotate(fp32 angle, fp32 *vector)
{
	if(vector == NULL) return;

	fp32 x_temp = vector[0];
	angle = rad_format(angle);

	fp32 cos_value = cosf(angle);
	fp32 sin_value = sinf(angle);

	vector[0] = cos_value * x_temp - sin_value * vector[1];
	vector[1] = sin_value * x_temp + cos_value * vector[1];
}

static void limit_chassis_wheel_speed(fp32 *wheel_speed)
{
	fp32 max_speed = 0.0f;

	for(uint8_t i = 0; i < CHASSIS_MODULE_NUM; i++)
	{
		fp32 speed_abs = fabsf(wheel_speed[i]);
		if(speed_abs > max_speed)
		{
			max_speed = speed_abs;
		}
	}

	if(max_speed > MAX_WHEEL_SPEED)
	{
		fp32 scale = MAX_WHEEL_SPEED / max_speed;
		for(uint8_t i = 0; i < CHASSIS_MODULE_NUM; i++)
		{
			wheel_speed[i] *= scale;
		}
	}
}

fp32 offset = 0.0f;

void slip_control(chassis_move_t *chassis_move)
{
	(void)chassis_move;
}

void chas_inv_cal(fp32 vx_set, fp32 vy_set, fp32 wz_set, fp32 *wheel_angle, fp32 *wheel_speed)
{
	if((wheel_angle == NULL) || (wheel_speed == NULL)) return;

	const fp32 wz_speed = wz_set * CHASSIS_OMNI_ROTATE_RADIUS;

	wheel_speed[WHEEL_REAR_205]  = (vx_set + wz_speed) * CHASSIS_WHEEL_205_DIRECTION;
	wheel_speed[WHEEL_RIGHT_206] = (vy_set + wz_speed) * CHASSIS_WHEEL_206_DIRECTION;
	wheel_speed[WHEEL_FRONT_207] = (vx_set - wz_speed) * CHASSIS_WHEEL_207_DIRECTION;
	wheel_speed[WHEEL_LEFT_208]  = (vy_set - wz_speed) * CHASSIS_WHEEL_208_DIRECTION;

	for(uint8_t i = 0; i < CHASSIS_MODULE_NUM; i++)
	{
		wheel_angle[i] = 0.0f;
	}

	limit_chassis_wheel_speed(wheel_speed);
}
