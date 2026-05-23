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

	chassis_move.wheel_angle_offset.now[0] = chassis_move.wheel_angle_offset.initial[0] = CHASSIS_6020_INIT_ANGLE_0;
	chassis_move.wheel_angle_offset.last[0] = CHASSIS_6020_INIT_ANGLE_0;
	chassis_move.wheel_angle_offset.now[1] = chassis_move.wheel_angle_offset.initial[1] = CHASSIS_6020_INIT_ANGLE_1;
	chassis_move.wheel_angle_offset.last[1] = CHASSIS_6020_INIT_ANGLE_1;
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

static void smooth_control(fp32 *wheel_angle, fp32 *wheel_speed)
{
	fp32 factor = 0.50f;
	fp32 angle_delta[CHASSIS_MODULE_NUM], Current_angle[CHASSIS_MODULE_NUM];

	for(uint8_t i = 0; i < CHASSIS_MODULE_NUM; i++)
	{
		Current_angle[i] = chassis_move.chassis_6020[i].angle;
		angle_delta[i] = rad_format(wheel_angle[i] - Current_angle[i]);

		if(angle_delta[i] > PI * factor)
		{
			wheel_angle[i] -= PI;
			wheel_speed[i] = -wheel_speed[i];
		}
		else if(angle_delta[i] < -PI * factor)
		{
			wheel_angle[i] += PI;
			wheel_speed[i] = -wheel_speed[i];
		}

		wheel_angle[i] = rad_format(wheel_angle[i]);
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

	fp32 vx_total[CHASSIS_MODULE_NUM], vy_total[CHASSIS_MODULE_NUM];
	const fp32 module_pos[CHASSIS_MODULE_NUM][2] = {
		{-HALF_LENGTH,  HALF_WIDTH},
		{ HALF_LENGTH, -HALF_WIDTH},
	};

	for(uint8_t i = 0; i < CHASSIS_MODULE_NUM; i++)
	{
		vx_total[i] = vx_set - wz_set * module_pos[i][1];
		vy_total[i] = vy_set + wz_set * module_pos[i][0];
		wheel_speed[i] = sqrtf(vx_total[i] * vx_total[i] + vy_total[i] * vy_total[i]);
		wheel_angle[i] = atan2f(vy_total[i], vx_total[i]) - chassis_move.wheel_angle_offset.now[i] + offset;
		wheel_angle[i] = rad_format(wheel_angle[i]);
	}

	smooth_control(wheel_angle, wheel_speed);
}
