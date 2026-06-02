#include "bsp_usart.h"
#include "chassis_calculate.h"
#include "chassis_task.h"
#include "bsp_fdcan.h"
#include "project_config.h"
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

/**
 * @brief  根据横向加速度限制底盘旋转角速度
 * @note   高速平移时，转弯横向加速度 a = v * wz 会增加侧翻力矩。
 *         该函数用于在高速平移叠加旋转时压低 wz，降低内侧轮卸载、抬轮和侧翻风险。
 * @param  vx 底盘 x 方向规划速度，单位 m/s
 * @param  vy 底盘 y 方向规划速度，单位 m/s
 * @param  wz 底盘规划旋转角速度，单位 rad/s
 * @retval 限制后的底盘旋转角速度，单位 rad/s
 */
fp32 chassis_limit_wz_by_lateral_accel(fp32 vx, fp32 vy, fp32 wz)
{
	fp32 translational_speed = sqrtf(vx * vx + vy * vy);
	fp32 wz_limit;

	if(translational_speed < CHASSIS_LAT_SPEED_EPS)
	{
		return wz;
	}

	wz_limit = CHASSIS_LAT_ACCEL_LIMIT / translational_speed;
	return fp32_constrain(wz, -wz_limit, wz_limit);
}

void chas_inv_cal(fp32 vx_set, fp32 vy_set, fp32 wz_set, fp32 *wheel_angle, fp32 *wheel_speed)
{
	if((wheel_angle == NULL) || (wheel_speed == NULL)) return;

	const fp32 wz_speed = wz_set * CHASSIS_OMNI_ROTATE_RADIUS;

	wheel_speed[WHEEL_REAR_205]  = (vy_set + wz_speed) * CHASSIS_WHEEL_205_DIRECTION;
	wheel_speed[WHEEL_RIGHT_206] = -(vx_set - wz_speed) * CHASSIS_WHEEL_206_DIRECTION;
	wheel_speed[WHEEL_FRONT_207] = -(vy_set - wz_speed) * CHASSIS_WHEEL_207_DIRECTION;
	wheel_speed[WHEEL_LEFT_208]  = (vx_set + wz_speed) * CHASSIS_WHEEL_208_DIRECTION;

	for(uint8_t i = 0; i < CHASSIS_MODULE_NUM; i++)
	{
		wheel_angle[i] = 0.0f;
	}

	limit_chassis_wheel_speed(wheel_speed);
}
