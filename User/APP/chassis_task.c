/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       chassis_task.c
  * @brief      搴曠洏鎺у埗浠诲姟
	*
	*     000000000000000     00               00    00     00   00         00 
	*           00     0      00                00    00   00     00       00  
	*       00  00000        00000000000000      00  000000000   00000000000000
	*       00  00          00           00    00    000000000     00     00   
	*      00000000000     00  000000    00     000     00           000000    
	*     00    00   0000     00    00   00      00   000000           00      
	*       00000000000       00    00   00           000000           00      
	*       0   00    0       0000000 00 00       00    00       00000000000000
	*       00000000000       00       000       00 00000000000        00      
	*           00            00                000 00000000000        00      
	*           00  00        00          0    000      00             00      
	*      000000000000        00        000  000       00          00 00      
	*       00        00        000000000000            00            00       
	********************************************************************************/
	
#include "bsp_usart.h"
#include "CAN_receive.h"
#include "chassis_behaviour.h"
#include "chassis_calculate.h"
#include "chassis_power_control.h"
#include "chassis_task.h"
#include "cmsis_os.h"
#include "detect_task.h"
#include "hwt_imu.h"
#include "pid.h"
#include "remote_control.h"
#include "robot_param.h"
#include "user_lib.h"
#include <math.h>
#include <stdbool.h>

#ifndef PID_USUAL
#define PID_USUAL PID_POSITION
#endif

#ifndef PID_calc
#define PID_calc PID_Calc
#endif

/********************************************
 * * * * * * * * * * * * * * * * * * * * * * 
	PID 璁＄畻涓嫢鏈夊崟鐙姞鍑忓彉閲忥紝鍧囪涓哄亸绉婚噺
 ********************************************/

#if INCLUDE_uxTaskGetStackHighWaterMark
	uint32_t chassis_high_water;
#endif

/* 搴曠洏杩愯鏁版嵁 */
chassis_move_t chassis_move;

static const fp32 chas_6020_angle_pid_param[3] = {
	GM6020_MOTOR_ANGLE_PID_KP,
	GM6020_MOTOR_ANGLE_PID_KI,
	GM6020_MOTOR_ANGLE_PID_KD
};
static const fp32 chas_6020_speed_pid_param[3] = {
	GM6020_MOTOR_SPEED_PID_KP,
	GM6020_MOTOR_SPEED_PID_KI,
	GM6020_MOTOR_SPEED_PID_KD
};
static const fp32 chassis_yaw_pid_param[3] = {
	CHASSIS_FOLLOW_GIMBAL_PID_KP,
	CHASSIS_FOLLOW_GIMBAL_PID_KI,
	CHASSIS_FOLLOW_GIMBAL_PID_KD
};
static const fp32 chassis_yaw_return_pid_param[3] = {
	YAW_RETURN_PID_KP,
	YAW_RETURN_PID_KI,
	YAW_RETURN_PID_KD
};
static const fp32 chassis_x_order_filter = CHASSIS_ACCEL_X_NUM;
static const fp32 chassis_y_order_filter = CHASSIS_ACCEL_Y_NUM;
static fp32 chassis_follow_gimbal_yaw_offset = CHASSIS_FOLLOW_GIMBAL_YAW_OFFSET;
static fp32 chassis_spin_offset = CHASSIS_SPIN_OFFSET;
static fp32 chassis_return_target = CHASSIS_RETURN_TARGET;

/**
  * @brief          鏇存柊搴曠洏鍙嶉鏁版嵁锛屽寘鎷?3508 閫熷害銆?020 瑙掑害鍜?IMU 濮挎€?
  * @param[out]     chassis_move_update: 搴曠洏杩愯鏁版嵁鎸囬拡
  * @retval         none
  */
static void chassis_feedback_update(chassis_move_t *chassis_move_update)
{
	if (chassis_move_update == NULL) return;
	static fp32 last_speed[CHASSIS_MODULE_NUM] = {0.0f, 0.0f};
	
	for (uint8_t i = 0; i < CHASSIS_MODULE_NUM; i++)
	{
		// 3508 鐢垫満锛氱敱缂栫爜鍣ㄩ€熷害鎹㈢畻绾块€熷害骞惰绠楀姞閫熷害
chassis_move_update->chassis_3508[i].speed = chassis_move_update->chassis_3508[i].chassis_motor_measure->speed_rpm / MPS_to_RPM;
		chassis_move_update->chassis_3508[i].accel = (chassis_move_update->chassis_3508[i].speed - last_speed[i]) * CHASSIS_CONTROL_FREQUENCE;
		last_speed[i] = chassis_move_update->chassis_3508[i].speed;
		// 6020 鐢垫満锛氱敱缂栫爜鍣ㄦ崲绠楄搴?
		chassis_move_update->chassis_6020[i].angle = rad_format(chassis_move_update->chassis_6020[i].chassis_motor_measure->ecd / GM6020_Angle_Ratio);
	}
	
	// IMU 濮挎€佽
	chassis_move_update->chassis_yaw 	 = rad_format(*(chassis_move_update->chassis_INS_angle + INS_YAW_ADDRESS_OFFSET	 ));
	chassis_move_update->chassis_pitch = rad_format(*(chassis_move_update->chassis_INS_angle + INS_PITCH_ADDRESS_OFFSET));
	chassis_move_update->chassis_roll	 = rad_format(*(chassis_move_update->chassis_INS_angle + INS_ROLL_ADDRESS_OFFSET ));
}

/**
  * @brief          鍒濆鍖栧簳鐩樹换鍔″弬鏁般€丳ID 鍜屽弽棣堟寚閽?
  * @param[out]     chassis_move_init: 搴曠洏杩愯鏁版嵁鎸囬拡
  * @retval         none
  */
static void chassis_init(chassis_move_t *chassis_move_init)
{
	if (chassis_move_init == NULL) return;
	
	// 鍒濆鐘舵€佺疆涓洪潤姝?
	chassis_move_init->chassis_mode = CHASSIS_VECTOR_NO_MOVE;
	// 鑾峰彇閬ユ帶鍣ㄦ寚閽?
chassis_move_init->chassis_RC = get_remote_control_point();
	// 鑾峰彇 IMU 濮挎€佹寚閽?
	chassis_move_init->chassis_INS_angle = get_INS_angle_point();
	// 鑾峰彇浜戝彴鐢垫満鎸囬拡
	chassis_move_init->chassis_yaw_motor = get_yaw_motor_point();
	chassis_move_init->chassis_pitch_motor = get_pitch_motor_point();
	
	// 缁戝畾搴曠洏鍥涗釜鐢垫満鐨勫弽棣堟寚閽堝苟鍒濆鍖?PID
	for(uint8_t i = 0; i < CHASSIS_MODULE_NUM; i++)
	{
		chassis_move_init->chassis_3508[i].chassis_motor_measure = get_chassis_motor_measure_point(i);
		chassis_move_init->model_3508_out[i] = 0.0f;
		chassis_move_init->model_accel[i] = 0.0f;
		chassis_move_init->chassis_6020[i].chassis_motor_measure = get_chassis_motor_measure_point(i + CHASSIS_MODULE_NUM);
		PID_init(&chassis_move_init->chas_6020_angle_pid[i], PID_USUAL, chas_6020_angle_pid_param, GM6020_MOTOR_ANGLE_PID_MAX_OUT, GM6020_MOTOR_ANGLE_PID_MAX_IOUT);
		PID_init(&chassis_move_init->chas_6020_speed_pid[i], PID_USUAL, chas_6020_speed_pid_param, GM6020_MOTOR_SPEED_PID_MAX_OUT, GM6020_MOTOR_SPEED_PID_MAX_IOUT);
	}
	// 鍒濆鍖栧洖姝ｆā寮?PID
	PID_init(&chassis_move_init->chas_return_pid, PID_USUAL, chassis_yaw_return_pid_param, YAW_RETURN_PID_MAX_OUT, YAW_RETURN_PID_MAX_IOUT);
	
	// 鍒濆鍖栬窡闅忎簯鍙?PID
	PID_init(&chassis_move_init->chassis_angle_pid, PID_USUAL, chassis_yaw_pid_param, CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT, CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT);
	
	// 涓€闃舵护娉㈠櫒鍙傛暟
	first_order_filter_init(&chassis_move_init->chassis_cmd_slow_set_vx, CHASSIS_CONTROL_TIME, &chassis_x_order_filter);
	first_order_filter_init(&chassis_move_init->chassis_cmd_slow_set_vy, CHASSIS_CONTROL_TIME, &chassis_y_order_filter);
		
	// 璁剧疆搴曠洏閫熷害涓婁笅闄?
	chassis_move_init->vx_max_speed =  NORMAL_MAX_CHASSIS_SPEED_X;
	chassis_move_init->vx_min_speed = -NORMAL_MAX_CHASSIS_SPEED_X;
	chassis_move_init->vy_max_speed =  NORMAL_MAX_CHASSIS_SPEED_Y;
	chassis_move_init->vy_min_speed = -NORMAL_MAX_CHASSIS_SPEED_Y;
	
	// 鍒濆鍖栬埖杞浂浣嶅亸缃?
chassis_wheel_angle_offset_init();
	
	// 鍥炴鏍囧織榛樿缃?1
	chassis_move_init->chassis_return_flag = 1;
	
	// 鍒锋柊涓€娆″簳鐩樺弽棣?
chassis_feedback_update(chassis_move_init);
}

/**
  * @brief          鏍规嵁閬ユ帶鍣ㄦ垨閿洏璁剧疆搴曠洏鎺у埗妯″紡
  * @param[out]     chassis_move_mode: 搴曠洏杩愯鏁版嵁鎸囬拡
  * @retval         none
  */
static void chassis_set_mode(chassis_move_t *chassis_move_mode)
{
	if(chassis_move_mode == NULL) return;
	chassis_behaviour_mode_set(chassis_move_mode);	// 见 chassis_behaviour.c
}

/**
  * @brief          搴曠洏妯″紡鍒囨崲鏃跺鐞嗙姸鎬佽縼绉?
  * @param[out]     chassis_move_transit: 搴曠洏杩愯鏁版嵁鎸囬拡
  * @retval         none
  */
static void chassis_mode_change_control_transit(chassis_move_t *chassis_move_transit)
{
	if(chassis_move_transit == NULL) return;
	
	// 鍒囨崲鍒伴潤姝㈡ā寮忔椂娓呯┖鐩稿瑙掑害
	if((chassis_move_transit->last_chassis_mode != CHASSIS_VECTOR_NO_MOVE) && chassis_move_transit->chassis_mode == CHASSIS_VECTOR_NO_MOVE)
	{
		chassis_move_transit->chassis_relative_angle_set = 0.0f;
	}
	// 鍒囨崲鍒拌窡闅忎簯鍙版ā寮忔椂娓呯┖鐩稿瑙掑害
	else if((chassis_move_transit->last_chassis_mode != CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW) && chassis_move_transit->chassis_mode == CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW)
	{
		chassis_move_transit->chassis_relative_angle_set = 0.0f;
	}
	// 鍒囨崲鍒版棆杞ā寮忔椂璁板綍褰撳墠搴曠洏鏈濆悜
	else if((chassis_move_transit->last_chassis_mode != CHASSIS_VECTOR_SPIN) && chassis_move_transit->chassis_mode == CHASSIS_VECTOR_SPIN)
	{
		chassis_move_transit->chassis_relative_angle_set = chassis_move_transit->chassis_yaw;
	}
	
	chassis_move_transit->last_chassis_mode = chassis_move_transit->chassis_mode;
}

/**
  * @brief          灏嗛仴鎺у櫒杈撳叆杞崲涓哄簳鐩樺钩闈㈤€熷害鎸囦护
  * @param[out]     vx_set: 鍓嶅悜閫熷害杈撳嚭
  * @param[out]     vy_set: 宸﹀悜閫熷害杈撳嚭
  * @param[out]     chassis_move_rc_to_vector: 搴曠洏杩愯鏁版嵁鎸囬拡
  * @retval         none
  */
void chassis_rc_to_control_vector(fp32 *vx_set, fp32 *vy_set, chassis_move_t *chassis_move_rc_to_vector)
{
	if (chassis_move_rc_to_vector == NULL || vx_set == NULL || vy_set == NULL) return;
	
	int16_t vx_channel, vy_channel;
	fp32 vx_set_channel, vy_set_channel;
	fp32 slope_percentage = 0.30f;
	static uint8_t orientation_count[4] = {0};
	
	// 閬ユ帶鍣ㄦ憞鏉嗘鍖哄鐞?
	rc_deadband_limit(chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_X_CHANNEL], vx_channel, CHASSIS_RC_DEADLINE);
	rc_deadband_limit(chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_Y_CHANNEL], vy_channel, CHASSIS_RC_DEADLINE);
	vx_set_channel = vx_channel * (CHASSIS_VX_RC_SEN);
	vy_set_channel = vy_channel * (CHASSIS_VY_RC_SEN);
	
	// 閿洏鍔犻€熸帶鍒?
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
	
	// 涓€闃舵护娉㈠櫒骞虫粦閫熷害杈撳叆
	first_order_filter_cali(&chassis_move_rc_to_vector->chassis_cmd_slow_set_vx, vx_set_channel);
	first_order_filter_cali(&chassis_move_rc_to_vector->chassis_cmd_slow_set_vy, vy_set_channel);
	
	// 閬ユ帶鍣ㄦ鍖鸿緭鍑虹疆闆?
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

// 璋冭瘯鐢ㄥ叏灞€鍋忕疆锛屼究浜庡悗缁仈璋冩椂缁熶竴璋冩暣

/**
  * @brief          璁剧疆搴曠洏鎺у埗杈撳嚭
  * @param[out]     chassis_move_control: 搴曠洏杩愯鏁版嵁鎸囬拡
  * @retval         none
  */
static void chassis_set_contorl(chassis_move_t *chassis_move_control)
{
	fp32 vector[2];

	if (chassis_move_control == NULL) return;
	
	fp32 vx_set = 0.0f, vy_set = 0.0f, wz_set = 0.0f;

	// 鑾峰彇涓夎酱鎺у埗閲?
	chassis_behaviour_control_set(&vx_set, &vy_set, &wz_set, chassis_move_control);
	wz_set = - wz_set;
	
	if (chassis_move_control->chassis_mode == CHASSIS_VECTOR_RETURN)
	{
		chassis_move_control->wz_set = -chassis_move_control->return_wz_set * 0.00006f;
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
	// 璺熼殢浜戝彴妯″紡
	else if (chassis_move_control->chassis_mode == CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW)
	{
		chassis_move_control->wz_set = wz_set;
		
		vector[0] = vx_set;
		vector[1] = vy_set;
		vector_rotate(chassis_move_control->gimbal_radian_of_ecd + chassis_follow_gimbal_yaw_offset, vector);
		vx_set = vector[0];
		vy_set = vector[1];
		chassis_move_control->vx_set = fp32_constrain(vx_set, chassis_move_control->vx_min_speed, chassis_move_control->vx_max_speed);
		chassis_move_control->vy_set = fp32_constrain(vy_set, chassis_move_control->vy_min_speed, chassis_move_control->vy_max_speed);
	}
	else if (chassis_move_control->chassis_mode == CHASSIS_VECTOR_SPIN)
	{
		chassis_move_control->wz_set = wz_set;

		vector[0] = vx_set;
		vector[1] = vy_set;
		vector_rotate(chassis_move_control->gimbal_radian_of_ecd + chassis_spin_offset, vector);
		vx_set = vector[0];
		vy_set = vector[1];
		chassis_move_control->vx_set = fp32_constrain(vx_set, chassis_move_control->vx_min_speed, chassis_move_control->vx_max_speed);
		chassis_move_control->vy_set = fp32_constrain(vy_set, chassis_move_control->vy_min_speed, chassis_move_control->vy_max_speed);
	}
}

/*************************************************************
  * @brief          鍩轰簬杞﹁締鍔ㄥ姏瀛︾殑 3508 鐢垫満鐢垫祦妯″瀷
  * @param[in]      motor_idx: 鐢垫満绱㈠紩 0-3
  * @param[in]      set_speed: 鐩爣閫熷害
  * @param[in]      ref_speed: 褰撳墠閫熷害
  * @retval         鐢垫満鎺у埗鐢垫祦
 ************************************************************/
static fp32 Model_Based_Control(uint8_t motor_idx, fp32 set_speed, fp32 ref_speed)
{
	fp32 error_v = set_speed - ref_speed;
	fp32 accel_target = 0.0f;
	fp32 accel_diff = 0.0f;
	fp32 max_accel_diff = 0.0f;
	fp32 F_traction = 0.0f;
	fp32 I_friction = 0.0f;
	fp32 I_hold_p = 0.0f;
	fp32 F_total = 0.0f;
	fp32 Torque_output = 0.0f;
	fp32 Current_A = 0.0f;
	fp32 out = 0.0f;

	accel_target = error_v / CONTROL_PERIOD_MODEL;
	accel_diff = accel_target - chassis_move.model_accel[motor_idx];
	max_accel_diff = CHASSIS_MAX_JERK * CHASSIS_CONTROL_TIME;
	if (accel_diff > max_accel_diff)
	{
		accel_diff = max_accel_diff;
	}
	else if (accel_diff < -max_accel_diff)
	{
		accel_diff = -max_accel_diff;
	}

	chassis_move.model_accel[motor_idx] += accel_diff;
	if (chassis_move.model_accel[motor_idx] > CHASSIS_MAX_ACCEL)
	{
		chassis_move.model_accel[motor_idx] = CHASSIS_MAX_ACCEL;
	}
	else if (chassis_move.model_accel[motor_idx] < -CHASSIS_MAX_ACCEL)
	{
		chassis_move.model_accel[motor_idx] = -CHASSIS_MAX_ACCEL;
	}

	F_traction = (ROBOT_MASS / 4.0f) * chassis_move.model_accel[motor_idx];

	if (fabsf(ref_speed) < FRICTION_SPEED_BAND)
	{
		I_friction = ref_speed * FRICTION_LINEAR_GAIN;
	}
	else
	{
		I_friction = sign(ref_speed) * FRICTION_CONSTANT_CURRENT;
	}

	if (fabsf(error_v) < SPEED_HOLD_ERROR_THRESHOLD)
	{
		I_hold_p = error_v * SPEED_HOLD_KP;
	}

	F_total = F_traction;
	Torque_output = (F_total * Wheel_Radius) / CHASSIS_EFFICIENCY;
	if (Torque_output > M3508_MAX_CONT_TORQUE * M3508_REDUCTION_RATIO)
	{
		Torque_output = M3508_MAX_CONT_TORQUE * M3508_REDUCTION_RATIO;
	}
	else if (Torque_output < -M3508_MAX_CONT_TORQUE * M3508_REDUCTION_RATIO)
	{
		Torque_output = -M3508_MAX_CONT_TORQUE * M3508_REDUCTION_RATIO;
	}

	Current_A = Torque_output / M3508_TORQUE_CONSTANT;
	out = Current_A * (16384.0f / 20.0f) + I_friction + I_hold_p;

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
  * @brief          澶勭悊鑸佃疆瑙掑害璺冲彉鏃剁殑 PID 璁＄畻
  * @param[in]      chassis_pid_calc: 搴曠洏杩愯鏁版嵁鎸囬拡
  * @retval         none
  ************************************************************/
static void PID_Calc_Jump(chassis_move_t *chassis_pid_calc)
{
	static int32_t count = 0;
	fp32 target = 0.0f;
	fp32 actual = 0.0f;
	fp32 adjusted_target = 0.0f;
	bool no_control = false;
	uint8_t i;

	for (i = 0; i < CHASSIS_MODULE_NUM; i++)
	{
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

		no_control = (fabsf(chassis_pid_calc->vx_set) < 0.1f &&
		              fabsf(chassis_pid_calc->vy_set) < 0.1f &&
		              fabsf(chassis_pid_calc->wz_set) < 0.001f);

		if (no_control)
		{
			switch (chassis_pid_calc->chassis_mode)
			{
				case CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW:
					if (chassis_pid_calc->chassis_return_record != chassis_pid_calc->chassis_mode)
					{
						count = 2001;
					}

					if (count > 1)
					{
						count--;
						chassis_pid_calc->chassis_6020[i].angle_set = rad_format(
							-chassis_pid_calc->wheel_angle_offset.initial[i] + 1.15f +
							chassis_pid_calc->gimbal_radian_of_ecd);
						chassis_pid_calc->wheel_angle_offset.last[i] = chassis_pid_calc->chassis_6020[i].angle;
					}
					else
					{
						chassis_pid_calc->chassis_6020[i].angle_set = chassis_pid_calc->wheel_angle_offset.last[i];
					}
					break;

				case CHASSIS_VECTOR_NO_MOVE:
					chassis_pid_calc->chassis_3508[i].speed_set = 0;
					chassis_pid_calc->chassis_6020[i].angle_set = chassis_pid_calc->wheel_angle_offset.last[i];
					chassis_pid_calc->wheel_angle_offset.now[i] = chassis_pid_calc->wheel_angle_offset.initial[i];
					break;

				case CHASSIS_VECTOR_SPIN:
					chassis_pid_calc->wheel_angle_offset.now[i] = chassis_pid_calc->wheel_angle_offset.initial[i];
					break;

				default:
					break;
			}
		}
		else
		{
			chassis_pid_calc->wheel_angle_offset.last[i] = chassis_pid_calc->chassis_6020[i].angle;
		}

		chassis_pid_calc->chassis_return_record = chassis_pid_calc->chassis_mode;
		chassis_pid_calc->model_3508_out[i] = Model_Based_Control(i, chassis_pid_calc->chassis_3508[i].speed_set, chassis_pid_calc->chassis_3508[i].speed);

		target = chassis_pid_calc->chassis_6020[i].angle_set;
		actual = chassis_pid_calc->chassis_6020[i].angle;
		adjusted_target = target;
		if (fabs(actual - target) > PI)
		{
			adjusted_target += (target < 0) ? 2 * PI : -2 * PI;
		}

		PID_calc(&chassis_pid_calc->chas_6020_angle_pid[i], actual, adjusted_target);
		PID_calc(&chassis_pid_calc->chas_6020_speed_pid[i], chassis_pid_calc->chassis_6020[i].chassis_motor_measure->speed_rpm, chassis_pid_calc->chas_6020_angle_pid[i].out);
	}
}

static void chassis_control_loop(chassis_move_t *chassis_move_control_loop)
{
	fp32 wheel_speed[CHASSIS_MODULE_NUM] = {0.0f, 0.0f};
	fp32 wheel_angle[CHASSIS_MODULE_NUM] = {0.0f, 0.0f};
	uint8_t i;

	if(chassis_move_control_loop->last_vx_set != 0 && chassis_move_control_loop->vx_set == 0)
	{
		chassis_move_control_loop->vx_set = chassis_move_control_loop->last_vx_set * 0.994f;
		if(fabs(chassis_move_control_loop->vx_set) <= 0.1f) { chassis_move_control_loop->vx_set = 0.0f; }
	}
	if(chassis_move_control_loop->last_vy_set != 0 && chassis_move_control_loop->vy_set == 0)
	{
		chassis_move_control_loop->vy_set = chassis_move_control_loop->last_vy_set * 0.994f;
		if(fabs(chassis_move_control_loop->vy_set) <= 0.1f) { chassis_move_control_loop->vy_set = 0.0f; }
	}
	if(chassis_move_control_loop->last_wz_set != 0 && chassis_move_control_loop->wz_set == 0)
	{
		chassis_move_control_loop->wz_set = chassis_move_control_loop->last_wz_set * 0.994f;
		if(fabs(chassis_move_control_loop->wz_set) <= 0.001f) { chassis_move_control_loop->wz_set = 0.0f; }
	}

	chassis_move_control_loop->last_vx_set = chassis_move_control_loop->vx_set;
	chassis_move_control_loop->last_vy_set = chassis_move_control_loop->vy_set;
	chassis_move_control_loop->last_wz_set = chassis_move_control_loop->wz_set;

	chas_inv_cal(chassis_move_control_loop->vx_set,
	             chassis_move_control_loop->vy_set,
	             chassis_move_control_loop->wz_set,
	             wheel_angle,
	             wheel_speed);

	for (i = 0; i < CHASSIS_MODULE_NUM; i++)
	{
		chassis_move_control_loop->chassis_6020[i].angle_set = wheel_angle[i];
		chassis_move_control_loop->chassis_3508[i].speed_set = wheel_speed[i];
	}

	slip_control(chassis_move_control_loop);
	PID_Calc_Jump(chassis_move_control_loop);
	chassis_power_control(chassis_move_control_loop);
}

void chassis_task(void const *pvParameters)
{
	vTaskDelay(CHASSIS_TASK_INIT_TIME);
	chassis_init(&chassis_move);

	while (toe_is_error(CHASSIS_MOTOR1_TOE) || toe_is_error(CHASSIS_MOTOR2_TOE) ||
	       toe_is_error(CHASSIS_MOTOR3_TOE) || toe_is_error(CHASSIS_MOTOR4_TOE) ||
	       toe_is_error(DBUS_TOE))
	{
		vTaskDelay(CHASSIS_CONTROL_TIME_MS);
	}

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
			/*
			if (toe_is_error(DBUS_TOE))
			{
				CAN_cmd_CHASSIS_ALL(0, 0, 0, 0);
			}
			else
			{
				CAN_cmd_CHASSIS_ALL(chassis_move.chassis_3508[0].give_current,
				                    chassis_move.chassis_3508[1].give_current,
				                    chassis_move.chassis_6020[0].give_current,
				                    chassis_move.chassis_6020[1].give_current);
			}
			*/
			CAN_cmd_CHASSIS_ALL(0, 0, 0, 0);
		}

		osDelay(1);

		#if INCLUDE_uxTaskGetStackHighWaterMark
			chassis_high_water = uxTaskGetStackHighWaterMark(NULL);
		#endif
	}
}
