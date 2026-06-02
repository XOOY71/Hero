#include "bsp_usart.h"
#include "bsp_fdcan.h"
#include "chassis_behaviour.h"
#include "chassis_calculate.h"
#include "chassis_power_control.h"
#include "chassis_task.h"
#include "cmsis_os.h"
#include "detect_task.h"
#include "hwt_imu.h"

#include "pid.h"
#include "project_config.h"
#include "remote_control.h"

#include "user_lib.h"

#include <math.h>
#include <stdbool.h>

#ifndef PID_USUAL
#define PID_USUAL PID_POSITION
#endif

#ifndef PID_calc
#define PID_calc PID_Calc
#endif


#if INCLUDE_uxTaskGetStackHighWaterMark
	uint32_t chassis_high_water;
#endif


/* 搴曠洏杩愯鏁版嵁 */
chassis_move_t chassis_move;

static const fp32 chassis_yaw_pid_param[3] = {
	CHASSIS_ANGLE_PD_KP,
	0.0f,
	CHASSIS_ANGLE_PD_KD
};
static const fp32 chassis_yaw_return_pid_param[3] = {
	YAW_RETURN_PID_KP,
	YAW_RETURN_PID_KI,
	YAW_RETURN_PID_KD
};
static const fp32 chassis_x_order_filter = CHASSIS_ACCEL_X_NUM;
static const fp32 chassis_y_order_filter = CHASSIS_ACCEL_Y_NUM;
static fp32 chassis_return_target = CHASSIS_RETURN_TARGET;

static void chassis_feedback_update(chassis_move_t *chassis_move_update);
static void chassis_init(chassis_move_t *chassis_move_init);
static void chassis_set_mode(chassis_move_t *chassis_move_mode);
static void chassis_mode_change_control_transit(chassis_move_t *chassis_move_transit);
static void chassis_set_contorl(chassis_move_t *chassis_move_control);
static uint8_t chassis_hwt101_yaw_ready(void);
static fp32 chassis_angle_pd_calc(pid_type_def *pd, fp32 actual, fp32 target, uint8_t wrap_enable, fp32 target_vel, fp32 actual_vel);
static fp32 chassis_follow_yaw_control(chassis_move_t *chassis_move_follow, fp32 manual_wz);
static fp32 chassis_yaw_hold_control(chassis_move_t *chassis_move_yaw_hold, fp32 manual_wz);
static fp32 chassis_limit_abs(fp32 value, fp32 max_abs);
static fp32 chassis_s_curve_update(fp32 cmd, fp32 *plan, fp32 *accel, fp32 max_speed, fp32 max_accel, fp32 max_jerk, fp32 stop_accel, fp32 stop_jerk);
static fp32 chassis_speed_pi_calc(chassis_move_t *chassis_move_pi, uint8_t motor_idx, fp32 error_v);
static void chassis_body_feedforward_clear(chassis_move_t *chassis_move_ff);
static void chassis_body_feedforward_update(chassis_move_t *chassis_move_ff);
static fp32 Model_Based_Control(uint8_t motor_idx, fp32 set_speed, fp32 ref_speed);
static void PID_Calc_Jump(chassis_move_t *chassis_pid_calc);
static void chassis_dynamic_current_limit_update(chassis_move_t *chassis_move_limit);
static void chassis_control_loop(chassis_move_t *chassis_move_control_loop);

/* 搴曠洏浠诲姟涓诲惊鐜?*/
void chassis_task(void const *pvParameters)
{
	vTaskDelay(CHASSIS_TASK_INIT_TIME);
	chassis_init(&chassis_move);

	/*
	while (toe_is_error(CHASSIS_MOTOR1_TOE) || toe_is_error(CHASSIS_MOTOR2_TOE) ||
	       toe_is_error(CHASSIS_MOTOR3_TOE) || toe_is_error(CHASSIS_MOTOR4_TOE) ||
	       toe_is_error(DBUS_TOE))
	{
		vTaskDelay(CHASSIS_CONTROL_TIME_MS);
	}
	*/

	while (1)
	{
		chassis_set_mode(&chassis_move);
		chassis_mode_change_control_transit(&chassis_move);
		chassis_feedback_update(&chassis_move);
		chassis_set_contorl(&chassis_move);
		chassis_control_loop(&chassis_move);

		if (!(toe_is_error(CHASSIS_MOTOR1_TOE) && toe_is_error(CHASSIS_MOTOR2_TOE) &&
		      toe_is_error(CHASSIS_MOTOR3_TOE) && toe_is_error(CHASSIS_MOTOR4_TOE)))
		{
			CAN_cmd_CHASSIS_ALL(chassis_move.chassis_3508[0].give_current,
			                    chassis_move.chassis_3508[1].give_current,
			                    chassis_move.chassis_3508[2].give_current,
			                    chassis_move.chassis_3508[3].give_current);
		}

		osDelay(CHASSIS_CONTROL_TIME_MS);

		#if INCLUDE_uxTaskGetStackHighWaterMark
			chassis_high_water = uxTaskGetStackHighWaterMark(NULL);
		#endif
	}
}

/**
  * @brief          鏇存柊搴曠洏鍙嶉閲忥紝鍖呮嫭 3508 閫熷害銆両MU 濮挎€佺瓑
  * @param[out]     chassis_move_update: 搴曠洏鐘舵€佺粨鏋勪綋鎸囬拡
  * @retval         none
  */
static void chassis_feedback_update(chassis_move_t *chassis_move_update)
{
	if (chassis_move_update == NULL) return;
	static fp32 last_speed[CHASSIS_MODULE_NUM] = {0.0f};
	const hwt_imu_info_t *hwt101 = hwt101_get_info();

	for (uint8_t i = 0; i < CHASSIS_MODULE_NUM; i++)
	{
		chassis_move_update->chassis_3508[i].speed = chassis_move_update->chassis_3508[i].chassis_motor_measure->speed_rpm / MPS_to_RPM;
		chassis_move_update->chassis_3508[i].speed_rad_s = (fp32)chassis_move_update->chassis_3508[i].chassis_motor_measure->speed_rpm * CHASSIS_RPM_TO_RAD_PER_SEC;
		chassis_move_update->chassis_3508[i].given_current_a = (fp32)chassis_move_update->chassis_3508[i].chassis_motor_measure->given_current * CHASSIS_CURRENT_CMD_TO_A;
		chassis_move_update->chassis_3508[i].accel = (chassis_move_update->chassis_3508[i].speed - last_speed[i]) * CHASSIS_CONTROL_FREQUENCE;
		last_speed[i] = chassis_move_update->chassis_3508[i].speed;
	}

	if ((hwt101 != NULL) && (hwt101->angle.yaw_init_flag != 0u))
	{
		chassis_move_update->chassis_yaw = hwt101->angle.yaw_total_rad;
		chassis_move_update->chassis_yaw_rate =
			CHASSIS_YAW_RATE_FEEDBACK_SIGN * hwt101->gyro.radps[HWT_AXIS_YAW];
		chassis_move_update->chassis_pitch = rad_format(hwt101->angle.rad[HWT_AXIS_PITCH]);
		chassis_move_update->chassis_roll = rad_format(hwt101->angle.rad[HWT_AXIS_ROLL]);
	}

	if ((chassis_move_update->chassis_yaw_motor != NULL) &&
	    (chassis_move_update->chassis_yaw_motor->angle_offset_init != 0u))
	{
		chassis_move_update->chassis_relative_angle =
			rad_format(chassis_move_update->chassis_yaw_motor->relative_angle);
		chassis_move_update->gimbal_radian_of_ecd = chassis_move_update->chassis_relative_angle;
	}
}

/**
  * @brief          鍒濆鍖栧簳鐩樻帶鍒剁粨鏋勪綋鍜屽悇鎺у埗鍣?
  * @param[out]     chassis_move_init: 搴曠洏鐘舵€佺粨鏋勪綋鎸囬拡
  * @retval         none
  */
static void chassis_init(chassis_move_t *chassis_move_init)
{
	if (chassis_move_init == NULL) return;

	chassis_move_init->chassis_mode = CHASSIS_VECTOR_NO_MOVE;
	chassis_move_init->chassis_RC = get_remote_control_point();
	chassis_move_init->chassis_INS_angle = hwt101_get_chassis_yaw_point();
	chassis_move_init->chassis_yaw_motor = get_yaw_motor_point();
	chassis_move_init->chassis_pitch_motor = get_pitch_motor_point();

	for(uint8_t i = 0; i < CHASSIS_MODULE_NUM; i++)
	{
		chassis_move_init->chassis_3508[i].chassis_motor_measure = get_chassis_motor_measure_point(i);
		chassis_move_init->model_3508_out[i] = 0.0f;
		chassis_move_init->model_accel[i] = 0.0f;
		chassis_move_init->model_last_speed_set[i] = 0.0f;
		chassis_move_init->speed_pi_iout[i] = 0.0f;
		chassis_move_init->stop_brake_active[i] = 0u;
		chassis_move_init->motor_current_limit_a[i] = CHASSIS_CURRENT_BASE_LIMIT_A;
		chassis_move_init->last_motor_current_limit_a[i] = CHASSIS_CURRENT_BASE_LIMIT_A;
	}
	PID_init(&chassis_move_init->chas_return_pid, PID_USUAL, chassis_yaw_return_pid_param, YAW_RETURN_PID_MAX_OUT, YAW_RETURN_PID_MAX_IOUT);

	PID_init(&chassis_move_init->chassis_angle_pid, PID_USUAL, chassis_yaw_pid_param, CHASSIS_ANGLE_PD_MAX_OUT, 0.0f);

	first_order_filter_init(&chassis_move_init->chassis_cmd_slow_set_vx, CHASSIS_CONTROL_TIME, &chassis_x_order_filter);
	first_order_filter_init(&chassis_move_init->chassis_cmd_slow_set_vy, CHASSIS_CONTROL_TIME, &chassis_y_order_filter);

	chassis_move_init->vx_max_speed =  NORMAL_MAX_CHASSIS_SPEED_X;
	chassis_move_init->vx_min_speed = -NORMAL_MAX_CHASSIS_SPEED_X;
	chassis_move_init->vy_max_speed =  NORMAL_MAX_CHASSIS_SPEED_Y;
	chassis_move_init->vy_min_speed = -NORMAL_MAX_CHASSIS_SPEED_Y;
	chassis_move_init->vx_plan = 0.0f;
	chassis_move_init->vy_plan = 0.0f;
	chassis_move_init->wz_plan = 0.0f;
	chassis_move_init->vx_plan_accel = 0.0f;
	chassis_move_init->vy_plan_accel = 0.0f;
	chassis_move_init->wz_plan_accel = 0.0f;
	chassis_move_init->chassis_relative_angle = 0.0f;
	chassis_move_init->chassis_relative_angle_target = 0.0f;
	chassis_move_init->chassis_relative_angle_set = 0.0f;
	chassis_move_init->chassis_relative_angle_set_vel = 0.0f;
	chassis_move_init->chassis_yaw_target = 0.0f;
	chassis_move_init->chassis_yaw_set = 0.0f;
	chassis_move_init->chassis_yaw_set_vel = 0.0f;
	chassis_body_feedforward_clear(chassis_move_init);
	chassis_move_init->chassis_yaw_rate = 0.0f;

	chassis_wheel_angle_offset_init();

	chassis_move_init->chassis_return_flag = 1;

	chassis_feedback_update(chassis_move_init);
}

/**
  * @brief          鏇存柊搴曠洏妯″紡
  * @param[out]     chassis_move_mode: 搴曠洏鐘舵€佺粨鏋勪綋鎸囬拡
  * @retval         none
  */
static void chassis_set_mode(chassis_move_t *chassis_move_mode)
{
	if(chassis_move_mode == NULL) return;
	chassis_behaviour_mode_set(chassis_move_mode);
}

/**
  * @brief          澶勭悊搴曠洏妯″紡鍒囨崲鏃剁殑鐘舵€佽繃娓?
  * @param[out]     chassis_move_transit: 搴曠洏鐘舵€佺粨鏋勪綋鎸囬拡
  * @retval         none
  */
static void chassis_mode_change_control_transit(chassis_move_t *chassis_move_transit)
{
	if(chassis_move_transit == NULL) return;

	if((chassis_move_transit->last_chassis_mode != CHASSIS_VECTOR_NO_MOVE) && chassis_move_transit->chassis_mode == CHASSIS_VECTOR_NO_MOVE)
	{
		chassis_move_transit->chassis_relative_angle_target = 0.0f;
		chassis_move_transit->chassis_relative_angle_set = 0.0f;
		chassis_move_transit->chassis_relative_angle_set_vel = 0.0f;
		chassis_move_transit->chassis_yaw_target = chassis_move_transit->chassis_yaw;
		chassis_move_transit->chassis_yaw_set = chassis_move_transit->chassis_yaw;
		chassis_move_transit->chassis_yaw_set_vel = 0.0f;
		chassis_move_transit->wz_set = 0.0f;
		chassis_move_transit->wz_plan = 0.0f;
		chassis_move_transit->wz_plan_accel = 0.0f;
		PID_clear(&chassis_move_transit->chassis_angle_pid);
		chassis_body_feedforward_clear(chassis_move_transit);
	}
	else if((chassis_move_transit->last_chassis_mode != CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW) && chassis_move_transit->chassis_mode == CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW)
	{
		chassis_move_transit->chassis_relative_angle_target = 0.0f;
		chassis_move_transit->chassis_relative_angle_set = 0.0f;
		chassis_move_transit->chassis_relative_angle_set_vel = 0.0f;
		PID_clear(&chassis_move_transit->chassis_angle_pid);
		chassis_body_feedforward_clear(chassis_move_transit);
	}
	else if((chassis_move_transit->last_chassis_mode != CHASSIS_VECTOR_YAW_HOLD) && chassis_move_transit->chassis_mode == CHASSIS_VECTOR_YAW_HOLD)
	{
		chassis_move_transit->chassis_yaw_target = chassis_move_transit->chassis_yaw;
		chassis_move_transit->chassis_yaw_set = chassis_move_transit->chassis_yaw;
		chassis_move_transit->chassis_yaw_set_vel = 0.0f;
		PID_clear(&chassis_move_transit->chassis_angle_pid);
		chassis_body_feedforward_clear(chassis_move_transit);
	}
	else if((chassis_move_transit->last_chassis_mode != CHASSIS_VECTOR_SPIN) && chassis_move_transit->chassis_mode == CHASSIS_VECTOR_SPIN)
	{
		chassis_move_transit->chassis_relative_angle_target = chassis_move_transit->chassis_yaw;
		chassis_move_transit->chassis_relative_angle_set = chassis_move_transit->chassis_yaw;
		chassis_move_transit->chassis_relative_angle_set_vel = 0.0f;
		PID_clear(&chassis_move_transit->chassis_angle_pid);
		chassis_body_feedforward_clear(chassis_move_transit);
	}

	chassis_move_transit->last_chassis_mode = chassis_move_transit->chassis_mode;
}

/**
  * @brief          灏嗛仴鎺у櫒杈撳叆杞崲涓哄簳鐩橀€熷害鎸囦护
  * @param[out]     vx_set: X 鏂瑰悜閫熷害杈撳嚭
  * @param[out]     vy_set: Y 鏂瑰悜閫熷害杈撳嚭
  * @param[out]     chassis_move_rc_to_vector: 搴曠洏鐘舵€佺粨鏋勪綋鎸囬拡
  * @retval         none
  */
void chassis_rc_to_control_vector(fp32 *vx_set, fp32 *vy_set, chassis_move_t *chassis_move_rc_to_vector)
{
	if (chassis_move_rc_to_vector == NULL || vx_set == NULL || vy_set == NULL) return;

	int16_t vx_channel, vy_channel;
	fp32 vx_set_channel, vy_set_channel;
	fp32 slope_percentage = 0.30f;
	static uint8_t orientation_count[4] = {0};

	rc_deadband_limit(chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_X_CHANNEL], vx_channel, CHASSIS_RC_DEADLINE);
	rc_deadband_limit(chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_Y_CHANNEL], vy_channel, CHASSIS_RC_DEADLINE);
	vx_set_channel = vx_channel * (CHASSIS_VX_RC_SEN);
	vy_set_channel = vy_channel * (CHASSIS_VY_RC_SEN);

	if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_FRONT_KEY)
	{
		if (orientation_count[0] < 210){	orientation_count[0]++; }
		vx_set_channel = chassis_move_rc_to_vector->vx_max_speed * (slope_percentage + (((fp32)orientation_count[0]) / 300.0f));
	}
	else if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_BACK_KEY)
	{
		if (orientation_count[1] < 210){	orientation_count[1]++; }
		vx_set_channel = chassis_move_rc_to_vector->vx_min_speed * (slope_percentage + (((fp32)orientation_count[1]) / 300.0f));
	}

	if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_LEFT_KEY)
	{
		if (orientation_count[2] < 210){	orientation_count[2]++; }
		vy_set_channel = chassis_move_rc_to_vector->vy_max_speed * (slope_percentage + (((fp32)orientation_count[2]) / 300.0f));
	}
	else if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_RIGHT_KEY)
	{
		if (orientation_count[2] < 210){	orientation_count[3]++; }
		vy_set_channel = chassis_move_rc_to_vector->vy_min_speed * (slope_percentage + (((fp32)orientation_count[3]) / 300.0f));
	}

	if (!(chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_FRONT_KEY)){ orientation_count[0] = 0; }
	if (!(chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_BACK_KEY )){ orientation_count[1] = 0; }
	if (!(chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_LEFT_KEY )){ orientation_count[2] = 0; }
	if (!(chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_RIGHT_KEY)){ orientation_count[3] = 0; }

	// 涓€闃舵护娉㈠钩婊戦€熷害鎸囦护
	first_order_filter_cali(&chassis_move_rc_to_vector->chassis_cmd_slow_set_vx, vx_set_channel);
	first_order_filter_cali(&chassis_move_rc_to_vector->chassis_cmd_slow_set_vy, vy_set_channel);

	if (vx_set_channel < CHASSIS_RC_DEADLINE * CHASSIS_VX_RC_SEN && vx_set_channel > -CHASSIS_RC_DEADLINE * CHASSIS_VX_RC_SEN)
	{
		chassis_move_rc_to_vector->chassis_cmd_slow_set_vx.out = 0.0f;
	}
	if (vy_set_channel < CHASSIS_RC_DEADLINE * CHASSIS_VY_RC_SEN && vy_set_channel > -CHASSIS_RC_DEADLINE * CHASSIS_VY_RC_SEN)
	{
		chassis_move_rc_to_vector->chassis_cmd_slow_set_vy.out = 0.0f;
	}

	*vx_set =  chassis_move_rc_to_vector->chassis_cmd_slow_set_vx.out;
	*vy_set = -chassis_move_rc_to_vector->chassis_cmd_slow_set_vy.out;
}


/**
  * @brief          鏍规嵁搴曠洏妯″紡鐢熸垚閫熷害鍜岃閫熷害鎸囦护
  * @param[out]     chassis_move_control: 搴曠洏鐘舵€佺粨鏋勪綋鎸囬拡
  * @retval         none
  */
static void chassis_set_contorl(chassis_move_t *chassis_move_control)
{
	fp32 vector[2];

	if (chassis_move_control == NULL) return;

	fp32 vx_set = 0.0f, vy_set = 0.0f, wz_set = 0.0f;

	// 鐢辫涓哄眰鐢熸垚鍩虹閫熷害鎸囦护
	chassis_behaviour_control_set(&vx_set, &vy_set, &wz_set, chassis_move_control);
	wz_set = - wz_set;

	if (chassis_move_control->chassis_mode == CHASSIS_VECTOR_RETURN)
	{
		chassis_move_control->wz_set = -chassis_move_control->return_wz_set * CHASSIS_RETURN_WZ_SCALE;
		if (fabs(chassis_move_control->wz_set) < 0.001f &&
		    chassis_move_control->chassis_return_record == CHASSIS_VECTOR_RETURN)
		{
			chassis_move_control->wz_set = 0.0f;
			chassis_move_control->chassis_return_flag = 0;
		}

		vector[0] = vx_set;
		vector[1] = vy_set;
		vector_rotate(chassis_move_control->gimbal_radian_of_ecd + CHASSIS_RETURN_OFFSET, vector);
		vx_set = vector[0];
		vy_set = vector[1];
		chassis_move_control->vx_set = fp32_constrain(vx_set, chassis_move_control->vx_min_speed, chassis_move_control->vx_max_speed);
		chassis_move_control->vy_set = fp32_constrain(vy_set, chassis_move_control->vy_min_speed, chassis_move_control->vy_max_speed);
	}
	else if (chassis_move_control->chassis_mode == CHASSIS_VECTOR_NO_MOVE)
	{
		chassis_move_control->vx_set = vx_set = 0.0f;
		chassis_move_control->vy_set = vy_set = 0.0f;
		chassis_move_control->wz_set = wz_set = 0.0f;
	}
	else if (chassis_move_control->chassis_mode == CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW)
	{
		chassis_move_control->wz_set = chassis_follow_yaw_control(chassis_move_control, wz_set);

		chassis_move_control->vx_set = fp32_constrain(vx_set, chassis_move_control->vx_min_speed, chassis_move_control->vx_max_speed);
		chassis_move_control->vy_set = fp32_constrain(vy_set, chassis_move_control->vy_min_speed, chassis_move_control->vy_max_speed);
	}
	else if (chassis_move_control->chassis_mode == CHASSIS_VECTOR_YAW_HOLD)
	{
		chassis_move_control->wz_set = chassis_yaw_hold_control(chassis_move_control, wz_set);

		chassis_move_control->vx_set = fp32_constrain(vx_set, chassis_move_control->vx_min_speed, chassis_move_control->vx_max_speed);
		chassis_move_control->vy_set = fp32_constrain(vy_set, chassis_move_control->vy_min_speed, chassis_move_control->vy_max_speed);
	}
	else if (chassis_move_control->chassis_mode == CHASSIS_VECTOR_SPIN)
	{
		chassis_move_control->wz_set = wz_set;

		chassis_move_control->vx_set = fp32_constrain(vx_set, chassis_move_control->vx_min_speed, chassis_move_control->vx_max_speed);
		chassis_move_control->vy_set = fp32_constrain(vy_set, chassis_move_control->vy_min_speed, chassis_move_control->vy_max_speed);
	}
}

static uint8_t chassis_hwt101_yaw_ready(void)
{
	const hwt_imu_info_t *hwt101 = hwt101_get_info();

	if ((hwt101 != NULL) && (hwt101->angle.yaw_init_flag != 0u))
	{
		return 1u;
	}

	return 0u;
}

static void chassis_body_feedforward_clear(chassis_move_t *chassis_move_ff)
{
	uint8_t i;

	if (chassis_move_ff == NULL) return;

	chassis_move_ff->last_vx_plan_ff = 0.0f;
	chassis_move_ff->last_vy_plan_ff = 0.0f;
	chassis_move_ff->last_wz_plan_ff = 0.0f;
	chassis_move_ff->body_ff_ax = 0.0f;
	chassis_move_ff->body_ff_ay = 0.0f;
	chassis_move_ff->body_ff_alpha = 0.0f;
	for (i = 0; i < CHASSIS_MODULE_NUM; i++)
	{
		chassis_move_ff->body_ff_current[i] = 0.0f;
	}
	chassis_move_ff->body_ff_init = 0u;
}

static fp32 chassis_body_force_to_current_cmd(fp32 force_n)
{
	fp32 torque_output;
	fp32 current_a;
	fp32 current_cmd;

	torque_output = (force_n * Wheel_Radius) / CHASSIS_EFFICIENCY;
	torque_output = chassis_limit_abs(torque_output, M3508_MAX_CONT_TORQUE * M3508_REDUCTION_RATIO);
	current_a = torque_output / M3508_TORQUE_CONSTANT;
	current_cmd = current_a * (CHASSIS_CURRENT_CMD_FULL_SCALE / CHASSIS_CURRENT_FULL_SCALE_A);

	return chassis_limit_abs(current_cmd, CHASSIS_BODY_FF_MAX_CURRENT_CMD);
}

static void chassis_body_feedforward_update(chassis_move_t *chassis_move_ff)
{
	fp32 ax;
	fp32 ay;
	fp32 alpha;
	fp32 force_x;
	fp32 force_y;
	fp32 torque_z;
	fp32 yaw_force;
	fp32 wheel_force[CHASSIS_MODULE_NUM] = {0.0f};
	const fp32 accel_limit = (CHASSIS_STOP_DECEL > CHASSIS_MAX_ACCEL) ? CHASSIS_STOP_DECEL : CHASSIS_MAX_ACCEL;

	if (chassis_move_ff == NULL) return;

	if (chassis_move_ff->body_ff_init == 0u)
	{
		chassis_move_ff->last_vx_plan_ff = chassis_move_ff->vx_plan;
		chassis_move_ff->last_vy_plan_ff = chassis_move_ff->vy_plan;
		chassis_move_ff->last_wz_plan_ff = chassis_move_ff->wz_plan;
		chassis_move_ff->body_ff_init = 1u;
		return;
	}

	ax = (chassis_move_ff->vx_plan - chassis_move_ff->last_vx_plan_ff) / CHASSIS_CONTROL_TIME;
	ay = (chassis_move_ff->vy_plan - chassis_move_ff->last_vy_plan_ff) / CHASSIS_CONTROL_TIME;
	alpha = (chassis_move_ff->wz_plan - chassis_move_ff->last_wz_plan_ff) / CHASSIS_CONTROL_TIME;
	ax = chassis_limit_abs(ax, accel_limit);
	ay = chassis_limit_abs(ay, accel_limit);
	alpha = chassis_limit_abs(alpha, CHASSIS_BODY_FF_YAW_ACCEL_LIMIT);

	chassis_move_ff->last_vx_plan_ff = chassis_move_ff->vx_plan;
	chassis_move_ff->last_vy_plan_ff = chassis_move_ff->vy_plan;
	chassis_move_ff->last_wz_plan_ff = chassis_move_ff->wz_plan;
	chassis_move_ff->vx_plan_accel = ax;
	chassis_move_ff->vy_plan_accel = ay;
	chassis_move_ff->wz_plan_accel = alpha;
	chassis_move_ff->body_ff_ax = ax;
	chassis_move_ff->body_ff_ay = ay;
	chassis_move_ff->body_ff_alpha = alpha;

	force_x = ROBOT_MASS * ax;
	force_y = ROBOT_MASS * ay;
	torque_z = CHASSIS_BODY_FF_YAW_INERTIA_KGM2 * alpha;
	yaw_force = (fabsf(CHASSIS_OMNI_ROTATE_RADIUS) > 0.0001f) ? (torque_z / CHASSIS_OMNI_ROTATE_RADIUS) : 0.0f;

	wheel_force[WHEEL_REAR_205]  = ( 0.5f * force_y + 0.25f * yaw_force) * CHASSIS_WHEEL_205_DIRECTION;
	wheel_force[WHEEL_RIGHT_206] = (0.5f * force_x - 0.25f * yaw_force) * CHASSIS_WHEEL_206_DIRECTION;
	wheel_force[WHEEL_FRONT_207] = (0.5f * force_y - 0.25f * yaw_force) * CHASSIS_WHEEL_207_DIRECTION;
	wheel_force[WHEEL_LEFT_208]  = ( 0.5f * force_x + 0.25f * yaw_force) * CHASSIS_WHEEL_208_DIRECTION;

	for (uint8_t i = 0U; i < CHASSIS_MODULE_NUM; i++)
	{
		chassis_move_ff->body_ff_current[i] = chassis_body_force_to_current_cmd(wheel_force[i]);
	}
}

static fp32 chassis_angle_pd_calc(pid_type_def *pd, fp32 actual, fp32 target, uint8_t wrap_enable, fp32 target_vel, fp32 actual_vel)
{
	fp32 error;

	if (pd == NULL) return 0.0f;

	error = target - actual;
	if (wrap_enable != 0u)
	{
		error = rad_format(error);
	}

	pd->set = target;
	pd->fdb = actual;
	pd->error[2] = pd->error[1];
	pd->error[1] = pd->error[0];
	pd->error[0] = error;
	pd->Pout = pd->Kp * error;
	pd->Iout = 0.0f;
	pd->Dout = pd->Kd * (target_vel - actual_vel);
	pd->out = pd->Pout + pd->Dout;
	pd->out = chassis_limit_abs(pd->out, pd->max_out);

	return pd->out;
}

static fp32 chassis_follow_yaw_control(chassis_move_t *chassis_move_follow, fp32 manual_wz)
{
	fp32 min_relative;
	fp32 max_relative;
	fp32 manual_wz_limited;
	uint8_t wrap_enable = 1u;
	fp32 target_vel;
	fp32 pd_out;
	fp32 wz_cmd;

	if (chassis_move_follow == NULL) return 0.0f;

	if ((chassis_hwt101_yaw_ready() == 0u) ||
	    (chassis_move_follow->chassis_yaw_motor == NULL) ||
	    (chassis_move_follow->chassis_yaw_motor->angle_offset_init == 0u))
	{
		PID_clear(&chassis_move_follow->chassis_angle_pid);
		return 0.0f;
	}

	manual_wz_limited = chassis_limit_wz_by_lateral_accel(chassis_move_follow->vx_plan,
	                                                       chassis_move_follow->vy_plan,
	                                                       manual_wz);
	chassis_move_follow->chassis_relative_angle_target -= manual_wz_limited * CHASSIS_CONTROL_TIME;
	target_vel = -manual_wz_limited;
	min_relative = chassis_move_follow->chassis_yaw_motor->min_relative_angle;
	max_relative = chassis_move_follow->chassis_yaw_motor->max_relative_angle;
	if (min_relative < max_relative)
	{
		wrap_enable = 0u;
		chassis_move_follow->chassis_relative_angle_target =
			fp32_constrain(chassis_move_follow->chassis_relative_angle_target, min_relative, max_relative);
		if ((chassis_move_follow->chassis_relative_angle_target <= min_relative) ||
		    (chassis_move_follow->chassis_relative_angle_target >= max_relative))
		{
			target_vel = 0.0f;
		}
	}
	else
	{
		chassis_move_follow->chassis_relative_angle_target =
			rad_format(chassis_move_follow->chassis_relative_angle_target);
	}
	chassis_move_follow->chassis_relative_angle_set =
		chassis_move_follow->chassis_relative_angle_target;
	chassis_move_follow->chassis_relative_angle_set_vel = target_vel;

	pd_out = chassis_angle_pd_calc(&chassis_move_follow->chassis_angle_pid,
	                               chassis_move_follow->chassis_relative_angle,
	                               chassis_move_follow->chassis_relative_angle_set,
	                               wrap_enable,
	                               chassis_move_follow->chassis_relative_angle_set_vel,
	                               0.0f);
	wz_cmd = -chassis_limit_abs(pd_out, CHASSIS_WZ_MAX_SPEED);
	return wz_cmd;
}

static fp32 chassis_yaw_hold_control(chassis_move_t *chassis_move_yaw_hold, fp32 manual_wz)
{
	fp32 yaw_rate_set;
	fp32 pd_out;
	fp32 wz_cmd;

	if (chassis_move_yaw_hold == NULL) return 0.0f;

	if (chassis_hwt101_yaw_ready() == 0u)
	{
		PID_clear(&chassis_move_yaw_hold->chassis_angle_pid);
		chassis_move_yaw_hold->chassis_yaw_target = chassis_move_yaw_hold->chassis_yaw;
		chassis_move_yaw_hold->chassis_yaw_set = chassis_move_yaw_hold->chassis_yaw;
		chassis_move_yaw_hold->chassis_yaw_set_vel = 0.0f;
		return 0.0f;
	}

	yaw_rate_set = -manual_wz * CHASSIS_YAW_HOLD_RC_SEN;
	yaw_rate_set = chassis_limit_wz_by_lateral_accel(chassis_move_yaw_hold->vx_plan,
	                                                 chassis_move_yaw_hold->vy_plan,
	                                                 yaw_rate_set);
	yaw_rate_set = fp32_constrain(yaw_rate_set, -CHASSIS_WZ_MAX_SPEED, CHASSIS_WZ_MAX_SPEED);

	chassis_move_yaw_hold->chassis_yaw_target += yaw_rate_set * CHASSIS_CONTROL_TIME;
	chassis_move_yaw_hold->chassis_yaw_set =
		chassis_move_yaw_hold->chassis_yaw_target;
	chassis_move_yaw_hold->chassis_yaw_set_vel = yaw_rate_set;

	pd_out = chassis_angle_pd_calc(&chassis_move_yaw_hold->chassis_angle_pid,
	                               chassis_move_yaw_hold->chassis_yaw,
	                               chassis_move_yaw_hold->chassis_yaw_set,
	                               0u,
	                               chassis_move_yaw_hold->chassis_yaw_set_vel,
	                               chassis_move_yaw_hold->chassis_yaw_rate);
	wz_cmd = -chassis_limit_abs(pd_out, CHASSIS_WZ_MAX_SPEED);
	return wz_cmd;
}

static fp32 chassis_limit_abs(fp32 value, fp32 max_abs)
{
	if (value > max_abs)
	{
		return max_abs;
	}
	else if (value < -max_abs)
	{
		return -max_abs;
	}

	return value;
}

static fp32 chassis_s_curve_update(fp32 cmd, fp32 *plan, fp32 *accel, fp32 max_speed, fp32 max_accel, fp32 max_jerk, fp32 stop_accel, fp32 stop_jerk)
{
	fp32 accel_target;
	fp32 accel_step;
	fp32 next_plan;
	fp32 target;
	fp32 decel_limit;
	fp32 jerk_limit;
	const fp32 stop_epsilon = 0.0001f;

	if (plan == NULL || accel == NULL) return 0.0f;

	target = chassis_limit_abs(cmd, max_speed);
	if (fabsf(target) < stop_epsilon)
	{
		decel_limit = (stop_accel > 0.0f) ? stop_accel : max_accel;
		jerk_limit = (stop_jerk > 0.0f) ? stop_jerk : max_jerk;
		jerk_limit = fabsf(jerk_limit);

		if (fabsf(*plan) <= decel_limit * CHASSIS_CONTROL_TIME + stop_epsilon)
		{
			*plan = 0.0f;
			*accel = 0.0f;
			return 0.0f;
		}

		if ((*plan) * (*accel) > 0.0f)
		{
			*accel = 0.0f;
		}

		accel_target = (*plan > 0.0f) ? -decel_limit : decel_limit;
		accel_step = chassis_limit_abs(accel_target - *accel, jerk_limit * CHASSIS_CONTROL_TIME);
		*accel = chassis_limit_abs(*accel + accel_step, decel_limit);
		next_plan = *plan + *accel * CHASSIS_CONTROL_TIME;
		if ((*plan) * next_plan <= 0.0f)
		{
			next_plan = 0.0f;
			*accel = 0.0f;
		}

		*plan = chassis_limit_abs(next_plan, max_speed);
		return *plan;
	}

	accel_target = chassis_limit_abs((target - *plan) / CHASSIS_CONTROL_TIME, max_accel);
	accel_step = chassis_limit_abs(accel_target - *accel, max_jerk * CHASSIS_CONTROL_TIME);
	*accel = chassis_limit_abs(*accel + accel_step, max_accel);

	next_plan = *plan + *accel * CHASSIS_CONTROL_TIME;
	if (((target - *plan) * (target - next_plan) <= 0.0f) &&
	    (fabsf(target - *plan) <= fabsf(*accel * CHASSIS_CONTROL_TIME) + 0.0001f))
	{
		next_plan = target;
		*accel = 0.0f;
	}

	*plan = chassis_limit_abs(next_plan, max_speed);
	return *plan;
}

static fp32 chassis_speed_pi_calc(chassis_move_t *chassis_move_pi, uint8_t motor_idx, fp32 error_v)
{
	fp32 out;

	if (chassis_move_pi == NULL || motor_idx >= CHASSIS_MODULE_NUM) return 0.0f;

	chassis_move_pi->speed_pi_iout[motor_idx] += CHASSIS_SPEED_PI_KI * error_v * CHASSIS_CONTROL_TIME;
	chassis_move_pi->speed_pi_iout[motor_idx] = chassis_limit_abs(chassis_move_pi->speed_pi_iout[motor_idx],
	                                                              CHASSIS_SPEED_PI_MAX_IOUT);
	out = CHASSIS_SPEED_PI_KP * error_v + chassis_move_pi->speed_pi_iout[motor_idx];

	return chassis_limit_abs(out, CHASSIS_SPEED_PI_MAX_OUT);
}

/*************************************************************
  * @brief          3508 鐢垫満妯″瀷鎺у埗
  * @param[in]      motor_idx: 鐢垫満绱㈠紩 0-3
  * @param[in]      set_speed: 鐩爣閫熷害
  * @param[in]      ref_speed: 瀹為檯閫熷害
  * @retval         鐢垫祦杈撳嚭
 ************************************************************/
static fp32 Model_Based_Control(uint8_t motor_idx, fp32 set_speed, fp32 ref_speed)
{
	fp32 error_v = set_speed - ref_speed;
	fp32 accel_raw = 0.0f;
	fp32 I_accel = 0.0f;
	fp32 I_viscous = 0.0f;
	fp32 I_coulomb = 0.0f;
	fp32 I_static = 0.0f;
	fp32 I_brake = 0.0f;
	fp32 I_pi = 0.0f;
	fp32 out = 0.0f;
	fp32 friction_ff_scale = 1.0f;
	uint8_t brake_mode = 0u;

	if (motor_idx >= CHASSIS_MODULE_NUM) return 0.0f;

	if ((fabsf(set_speed) < 0.0001f) && (fabsf(ref_speed) < CHASSIS_BRAKE_RELEASE_SPEED_EPS))
	{
		chassis_move.model_accel[motor_idx] = 0.0f;
		chassis_move.model_last_speed_set[motor_idx] = 0.0f;
		chassis_move.speed_pi_iout[motor_idx] = 0.0f;
		chassis_move.stop_brake_active[motor_idx] = 0u;
		return 0.0f;
	}

	accel_raw = (set_speed - chassis_move.model_last_speed_set[motor_idx]) / CHASSIS_CONTROL_TIME;
	if ((fabsf(ref_speed) >= CHASSIS_BRAKE_ENTER_SPEED_EPS) &&
	    (((accel_raw * ref_speed) < 0.0f) || (fabsf(set_speed) < 0.0001f)))
	{
		chassis_move.stop_brake_active[motor_idx] = 1u;
	}
	else if ((chassis_move.stop_brake_active[motor_idx] != 0u) &&
	         ((fabsf(set_speed) >= 0.0001f) || (fabsf(ref_speed) < CHASSIS_BRAKE_RELEASE_SPEED_EPS)))
	{
		chassis_move.stop_brake_active[motor_idx] = 0u;
	}
	brake_mode = chassis_move.stop_brake_active[motor_idx];

	if (brake_mode != 0u)
	{
		friction_ff_scale = CHASSIS_BRAKE_FRICTION_FF_SCALE;
	}

	chassis_move.model_last_speed_set[motor_idx] = set_speed;

	I_accel = chassis_move.body_ff_current[motor_idx];
	I_viscous = friction_ff_scale * CHASSIS_FF_VISCOUS_GAIN * set_speed;
	I_coulomb = friction_ff_scale * CHASSIS_FF_COULOMB_CURRENT * tanhf(set_speed / CHASSIS_FF_COULOMB_SPEED_EPS);
	I_static = friction_ff_scale * CHASSIS_FF_STATIC_CURRENT * tanhf(set_speed / CHASSIS_FF_STATIC_SPEED_EPS);
	if (brake_mode != 0u)
	{
		I_brake = -(CHASSIS_BRAKE_FF_CURRENT_A / CHASSIS_CURRENT_CMD_TO_A) *
		          tanhf(ref_speed / CHASSIS_BRAKE_FF_SPEED_EPS);
	}
	I_pi = chassis_speed_pi_calc(&chassis_move, motor_idx, error_v);
	out = I_accel + I_viscous + I_coulomb + I_static + I_brake + I_pi;

	if (out > M3505_MOTOR_SPEED_PID_MAX_OUT)
	{
		out = M3505_MOTOR_SPEED_PID_MAX_OUT;
	}
	else if (out < -M3505_MOTOR_SPEED_PID_MAX_OUT)
	{
		out = -M3505_MOTOR_SPEED_PID_MAX_OUT;
	}

	return out;
}

/*************************************************************
  * @brief          澶勭悊搴曠洏 PID 涓庡洖姝ｉ€昏緫
  * @param[in]      chassis_pid_calc: 搴曠洏鐘舵€佺粨鏋勪綋鎸囬拡
  * @retval         none
 ************************************************************/
static void PID_Calc_Jump(chassis_move_t *chassis_pid_calc)
{
	fp32 target = 0.0f;
	fp32 actual = 0.0f;
	fp32 adjusted_target = 0.0f;
	uint8_t i;

	if (chassis_pid_calc == NULL) return;

	if (chassis_pid_calc->chassis_mode == CHASSIS_VECTOR_RETURN)
	{
		target = rad_format(chassis_return_target);
		actual = rad_format(chassis_pid_calc->gimbal_radian_of_ecd);

		if (fabs(actual - target) > PI)
		{
			adjusted_target = (target < 0) ? target + 2 * PI : target - 2 * PI;
			chassis_pid_calc->return_wz_set = PID_calc(&chassis_pid_calc->chas_return_pid, actual, adjusted_target);
		}
		else
		{
			chassis_pid_calc->return_wz_set = PID_calc(&chassis_pid_calc->chas_return_pid, actual, target);
		}
	}

	chassis_pid_calc->chassis_return_record = chassis_pid_calc->chassis_mode;

	for (i = 0; i < CHASSIS_MODULE_NUM; i++)
	{
		if (chassis_pid_calc->chassis_mode == CHASSIS_VECTOR_NO_MOVE)
		{
			chassis_pid_calc->chassis_3508[i].speed_set = 0.0f;
		}
		if ((fabsf(chassis_pid_calc->chassis_3508[i].speed_set) < 0.0001f) &&
		    (fabsf(chassis_pid_calc->chassis_3508[i].speed) < CHASSIS_BRAKE_RELEASE_SPEED_EPS))
		{
			chassis_pid_calc->model_accel[i] = 0.0f;
			chassis_pid_calc->model_last_speed_set[i] = 0.0f;
			chassis_pid_calc->speed_pi_iout[i] = 0.0f;
			chassis_pid_calc->stop_brake_active[i] = 0u;
		}

		chassis_pid_calc->model_3508_out[i] = Model_Based_Control(i, chassis_pid_calc->chassis_3508[i].speed_set, chassis_pid_calc->chassis_3508[i].speed);
	}
}

/* chassis dynamic current limit */
static void chassis_dynamic_current_limit_update(chassis_move_t *chassis_move_limit)
{
	fp32 demand_a[CHASSIS_MODULE_NUM] = {0.0f};
	fp32 total_demand_a = 0.0f;
	fp32 max_step_a = CHASSIS_CURRENT_LIMIT_SLEW_A_PER_S * CHASSIS_CONTROL_TIME;
	uint8_t i;

	if (chassis_move_limit == NULL) return;

	for (i = 0; i < CHASSIS_MODULE_NUM; i++)
	{
		demand_a[i] = fabsf(chassis_move_limit->model_3508_out[i]) * CHASSIS_CURRENT_CMD_TO_A;
		total_demand_a += demand_a[i];
	}

	for (i = 0; i < CHASSIS_MODULE_NUM; i++)
	{
		fp32 target_limit_a = CHASSIS_CURRENT_BASE_LIMIT_A;
		fp32 limit_step_a;
		fp32 limit_cmd;

		if (total_demand_a > CHASSIS_CURRENT_DEMAND_EPS_A)
		{
			target_limit_a += CHASSIS_CURRENT_DYNAMIC_POOL_A * demand_a[i] / total_demand_a;
		}

		target_limit_a = fp32_constrain(target_limit_a,
		                                CHASSIS_CURRENT_BASE_LIMIT_A,
		                                CHASSIS_CURRENT_SINGLE_MAX_A);
		limit_step_a = fp32_constrain(target_limit_a - chassis_move_limit->last_motor_current_limit_a[i],
		                               -max_step_a,
		                                max_step_a);
		chassis_move_limit->motor_current_limit_a[i] =
			chassis_move_limit->last_motor_current_limit_a[i] + limit_step_a;
		chassis_move_limit->last_motor_current_limit_a[i] =
			chassis_move_limit->motor_current_limit_a[i];

		limit_cmd = chassis_move_limit->motor_current_limit_a[i] / CHASSIS_CURRENT_CMD_TO_A;
		chassis_move_limit->model_3508_out[i] =
			fp32_constrain(chassis_move_limit->model_3508_out[i], -limit_cmd, limit_cmd);
	}
}

static void chassis_control_loop(chassis_move_t *chassis_move_control_loop)
{
	fp32 wheel_speed[CHASSIS_MODULE_NUM] = {0.0f};
	fp32 wheel_angle[CHASSIS_MODULE_NUM] = {0.0f};
	uint8_t i;

	chassis_move_control_loop->last_vx_set = chassis_move_control_loop->vx_set;
	chassis_move_control_loop->last_vy_set = chassis_move_control_loop->vy_set;
	chassis_move_control_loop->last_wz_set = chassis_move_control_loop->wz_set;
	chassis_move_control_loop->vx_plan = chassis_s_curve_update(chassis_move_control_loop->vx_set,
	                                                            &chassis_move_control_loop->vx_plan,
	                                                            &chassis_move_control_loop->vx_plan_accel,
	                                                            chassis_move_control_loop->vx_max_speed,
	                                                            CHASSIS_MAX_ACCEL,
	                                                            CHASSIS_MAX_JERK,
	                                                            CHASSIS_STOP_DECEL,
	                                                            CHASSIS_STOP_JERK);
	chassis_move_control_loop->vy_plan = chassis_s_curve_update(chassis_move_control_loop->vy_set,
	                                                            &chassis_move_control_loop->vy_plan,
	                                                            &chassis_move_control_loop->vy_plan_accel,
	                                                            chassis_move_control_loop->vy_max_speed,
	                                                            CHASSIS_MAX_ACCEL,
	                                                            CHASSIS_MAX_JERK,
	                                                            CHASSIS_STOP_DECEL,
	                                                            CHASSIS_STOP_JERK);
	if ((chassis_move_control_loop->chassis_mode == CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW) ||
	    (chassis_move_control_loop->chassis_mode == CHASSIS_VECTOR_YAW_HOLD))
	{
		chassis_move_control_loop->wz_plan =
			fp32_constrain(chassis_move_control_loop->wz_set, -CHASSIS_WZ_MAX_SPEED, CHASSIS_WZ_MAX_SPEED);
		chassis_move_control_loop->wz_plan_accel = 0.0f;
	}
	else
	{
		chassis_move_control_loop->wz_plan = chassis_s_curve_update(chassis_move_control_loop->wz_set,
		                                                            &chassis_move_control_loop->wz_plan,
		                                                            &chassis_move_control_loop->wz_plan_accel,
		                                                            CHASSIS_WZ_MAX_SPEED,
		                                                            CHASSIS_WZ_MAX_ACCEL,
		                                                            CHASSIS_WZ_MAX_JERK,
		                                                            CHASSIS_WZ_MAX_ACCEL,
		                                                            CHASSIS_WZ_MAX_JERK);
	}
	chassis_move_control_loop->wz_plan = chassis_limit_wz_by_lateral_accel(chassis_move_control_loop->vx_plan,
	                                                                       chassis_move_control_loop->vy_plan,
	                                                                       chassis_move_control_loop->wz_plan);
	chassis_body_feedforward_update(chassis_move_control_loop);

	chas_inv_cal(chassis_move_control_loop->vx_plan,
	             chassis_move_control_loop->vy_plan,
	             chassis_move_control_loop->wz_plan,
	             wheel_angle,
	             wheel_speed);

	for (i = 0; i < CHASSIS_MODULE_NUM; i++)
	{
		chassis_move_control_loop->chassis_3508[i].speed_set = wheel_speed[i];
	}

	slip_control(chassis_move_control_loop);
	PID_Calc_Jump(chassis_move_control_loop);
	chassis_dynamic_current_limit_update(chassis_move_control_loop);
	for (i = 0; i < CHASSIS_MODULE_NUM; i++)
	{
		chassis_move_control_loop->chassis_3508[i].give_current = (int16_t)chassis_move_control_loop->model_3508_out[i];
	}
	chassis_power_control(chassis_move_control_loop);
}

