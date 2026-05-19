/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       chassis.c
  * @brief     	底盘控制任务
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
	PID计算里面若有单独加减的变量, 均为偏移量
 * * * * * * * * * * * * * * * * * * * * * * 
 ********************************************/

#if INCLUDE_uxTaskGetStackHighWaterMark
	uint32_t chassis_high_water;
#endif

/* 底盘运动数据 */
chassis_move_t chassis_move;

/**
  * @brief          底盘测量数据更新，包括3508电机速度、6020电机角度、欧拉角度、机器人速度
  * @param[out]     chassis_move_update:"chassis_move"变量指针.
  * @retval         none
  */
static void chassis_feedback_update(chassis_move_t *chassis_move_update)
{
	if (chassis_move_update == NULL) return;
	static fp32 last_speed[4] = {0.0f, 0.0f, 0.0f, 0.0f};
	
	for (uint8_t i = 0; i < 4; i++)
	{
		//更新3508电机速度和加速度（由速度差分计算）
		chassis_move_update->chassis_3508[i].speed = chassis_move_update->chassis_3508[i].chassis_motor_measure->speed_rpm / MPS_to_RPM;
		chassis_move_update->chassis_3508[i].accel = (chassis_move_update->chassis_3508[i].speed - last_speed[i]) * CHASSIS_CONTROL_FREQUENCE;
		last_speed[i] = chassis_move_update->chassis_3508[i].speed;
		//更新6020电机角度
		chassis_move_update->chassis_6020[i].angle = rad_format(chassis_move_update->chassis_6020[i].chassis_motor_measure->ecd / GM6020_Angle_Ratio);
	}
	
	//底盘有陀螺仪
	chassis_move_update->chassis_yaw 	 = rad_format(*(chassis_move_update->chassis_INS_angle + INS_YAW_ADDRESS_OFFSET	 ));
	chassis_move_update->chassis_pitch = rad_format(*(chassis_move_update->chassis_INS_angle + INS_PITCH_ADDRESS_OFFSET));
	chassis_move_update->chassis_roll	 = rad_format(*(chassis_move_update->chassis_INS_angle + INS_ROLL_ADDRESS_OFFSET ));
}

/**
  * @brief          初始化"chassis_move"变量，包括pid初始化, 遥控器指针初始化, 3508底盘电机指针初始化, 云台电机初始化, 陀螺仪角度指针初始化
  * @param[out]     chassis_move_init:"chassis_move"变量指针.
  * @retval         none
  */
static void chassis_init(chassis_move_t *chassis_move_init)
{
	if (chassis_move_init == NULL) return;

	//底盘6020角度环pid值
	const static fp32 chas_6020_angle_pid_param[3] = {GM6020_MOTOR_ANGLE_PID_KP, GM6020_MOTOR_ANGLE_PID_KI, GM6020_MOTOR_ANGLE_PID_KD};
	//底盘6020速度环pid值
	const static fp32 chas_6020_speed_pid_param[3] = {GM6020_MOTOR_SPEED_PID_KP, GM6020_MOTOR_SPEED_PID_KI, GM6020_MOTOR_SPEED_PID_KD};
	//底盘角度pid值
	const static fp32 chassis_yaw_pid_param[3] = {CHASSIS_FOLLOW_GIMBAL_PID_KP, CHASSIS_FOLLOW_GIMBAL_PID_KI, CHASSIS_FOLLOW_GIMBAL_PID_KD};
	//回正模式pid值
	const static fp32 chassis_yaw_return_pid_param[3] = {YAW_RETURN_PID_KP, YAW_RETURN_PID_KI, YAW_RETURN_PID_KD};
	
	//底盘开机状态为原始
	chassis_move_init->chassis_mode = CHASSIS_VECTOR_NO_MOVE;
	//获取遥控器指针
	chassis_move_init->chassis_RC = get_remote_control_point();
	//获取陀螺仪姿态角指针
	chassis_move_init->chassis_INS_angle = get_INS_angle_point();
	//获取云台电机数据指针
	chassis_move_init->chassis_yaw_motor = get_yaw_motor_point();
	chassis_move_init->chassis_pitch_motor = get_pitch_motor_point();
	
	//获取底盘电机数据指针，初始化PID 
	for(uint8_t i = 0; i < 4; i++)
	{
		chassis_move_init->chassis_3508[i].chassis_motor_measure = get_chassis_motor_measure_point(i);
		chassis_move_init->model_3508_out[i] = 0.0f;
		chassis_move_init->model_accel[i] = 0.0f;
		chassis_move_init->chassis_6020[i].chassis_motor_measure = get_chassis_motor_measure_point(i + 4);
		PID_init(&chassis_move_init->chas_6020_angle_pid[i], PID_USUAL, chas_6020_angle_pid_param, GM6020_MOTOR_ANGLE_PID_MAX_OUT, GM6020_MOTOR_ANGLE_PID_MAX_IOUT);
		PID_init(&chassis_move_init->chas_6020_speed_pid[i], PID_USUAL, chas_6020_speed_pid_param, GM6020_MOTOR_SPEED_PID_MAX_OUT, GM6020_MOTOR_SPEED_PID_MAX_IOUT);
	}
	//初始化回正模式pid
	PID_init(&chassis_move_init->chas_return_pid, PID_USUAL, chassis_yaw_return_pid_param, YAW_RETURN_PID_MAX_OUT, YAW_RETURN_PID_MAX_IOUT);
	
	//初始化角度PID
	PID_init(&chassis_move_init->chassis_angle_pid, PID_USUAL, chassis_yaw_pid_param, CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT, CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT);
	
	//用一阶滤波代替斜波函数生成
	const static fp32 chassis_x_order_filter = CHASSIS_ACCEL_X_NUM;
	const static fp32 chassis_y_order_filter = CHASSIS_ACCEL_Y_NUM;
	first_order_filter_init(&chassis_move_init->chassis_cmd_slow_set_vx, CHASSIS_CONTROL_TIME, &chassis_x_order_filter);
	first_order_filter_init(&chassis_move_init->chassis_cmd_slow_set_vy, CHASSIS_CONTROL_TIME, &chassis_y_order_filter);
		
	//最大 最小速度
	chassis_move_init->vx_max_speed =  NORMAL_MAX_CHASSIS_SPEED_X;
	chassis_move_init->vx_min_speed = -NORMAL_MAX_CHASSIS_SPEED_X;
	chassis_move_init->vy_max_speed =  NORMAL_MAX_CHASSIS_SPEED_Y;
	chassis_move_init->vy_min_speed = -NORMAL_MAX_CHASSIS_SPEED_Y;
	
	//底盘舵向电机角度初始化
	chassis_wheel_angle_offset_init();
	
	//底盘回正标志位初始化为 1
	chassis_move_init->chassis_return_flag = 1;
	
	//更新一下数据
	chassis_feedback_update(chassis_move_init);
}

/**
  * @brief          用遥控器或键盘设置底盘控制模式
  * @param[out]     chassis_move_mode:"chassis_move"变量指针.
  * @retval         none
  */
static void chassis_set_mode(chassis_move_t *chassis_move_mode)
{
	if(chassis_move_mode == NULL) return;
	chassis_behaviour_mode_set(chassis_move_mode);	//in file "chassis_behaviour.c"
}

/**
  * @brief          底盘模式改变
  * @param[out]     chassis_move_transit:"chassis_move"变量指针.
  * @retval         none
  */
static void chassis_mode_change_control_transit(chassis_move_t *chassis_move_transit)
{
	if(chassis_move_transit == NULL) return;
	
	//切入跟随云台模式
	if((chassis_move_transit->last_chassis_mode != CHASSIS_VECTOR_NO_MOVE) && chassis_move_transit->chassis_mode == CHASSIS_VECTOR_NO_MOVE)
	{
		chassis_move_transit->chassis_relative_angle_set = 0.0f;
	}
	//切入跟随云台模式
	else if((chassis_move_transit->last_chassis_mode != CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW) && chassis_move_transit->chassis_mode == CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW)
	{
		chassis_move_transit->chassis_relative_angle_set = 0.0f;
	}
	//切入底盘旋转模式
	else if((chassis_move_transit->last_chassis_mode != CHASSIS_VECTOR_SPIN) && chassis_move_transit->chassis_mode == CHASSIS_VECTOR_SPIN)
	{
		chassis_move_transit->chassis_relative_angle_set = chassis_move_transit->chassis_yaw;
	}
	
	chassis_move_transit->last_chassis_mode = chassis_move_transit->chassis_mode;
}

/**
  * @brief          根据遥控器通道值，计算纵向和横移速度
  * @param[out]     vx_set: 纵向速度指针
  * @param[out]     vy_set: 横向速度指针
  * @param[out]     chassis_move_rc_to_vector: "chassis_move" 变量指针
  * @retval         none
  */
void chassis_rc_to_control_vector(fp32 *vx_set, fp32 *vy_set, chassis_move_t *chassis_move_rc_to_vector)
{
	if (chassis_move_rc_to_vector == NULL || vx_set == NULL || vy_set == NULL) return;
	
	int16_t vx_channel, vy_channel;
	fp32 vx_set_channel, vy_set_channel;
	fp32 slope_percentage = 0.30f;
	static uint8_t orientation_count[4] = {0};
	
	//死区限制，因为遥控器可能存在差异 摇杆在中间，其值不为0
	rc_deadband_limit(chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_X_CHANNEL], vx_channel, CHASSIS_RC_DEADLINE);
	rc_deadband_limit(chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_Y_CHANNEL], vy_channel, CHASSIS_RC_DEADLINE);
	vx_set_channel = vx_channel * (CHASSIS_VX_RC_SEN);
	vy_set_channel = vy_channel * (CHASSIS_VY_RC_SEN);
	
	//键盘控制
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
	
	//一阶低通滤波代替斜波作为底盘速度输入
	first_order_filter_cali(&chassis_move_rc_to_vector->chassis_cmd_slow_set_vx, vx_set_channel);
	first_order_filter_cali(&chassis_move_rc_to_vector->chassis_cmd_slow_set_vy, vy_set_channel);
	
	//遥感死区限制
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

//把小写的变量赋值成对应宏，用于调试，调试结束后应删除变量使用宏定义
fp32 chassis_follow_gimbal_yaw_offset = CHASSIS_FOLLOW_GIMBAL_YAW_OFFSET;
fp32 chassis_spin_offset = CHASSIS_SPIN_OFFSET;

/**
  * @brief          设置底盘控制设置值, 三运动控制值是通过 chassis_behaviour_control_set 函数设置的
  * @param[out]     chassis_move_update:"chassis_move"变量指针.
  * @retval         none
  */
static void chassis_set_contorl(chassis_move_t *chassis_move_control)
{
	if (chassis_move_control == NULL) return;
	
	fp32 vx_set = 0.0f, vy_set = 0.0f, wz_set = 0.0f;

	//获取三个控制设置值
	chassis_behaviour_control_set(&vx_set, &vy_set, &wz_set, chassis_move_control);
	wz_set = - wz_set;
	
	if (chassis_move_control->chassis_mode == CHASSIS_VECTOR_RETURN) //底盘零点回当前云台朝向
	{
		
		/* 在电脑屏幕上添加显示 回正模式标志 的代码，如：当前为回正模式 */
		
		//“wz_set”是旋转速度设置，此时为后面pid算出的值
		chassis_move_control->wz_set = -chassis_move_control->return_wz_set * 0.00006f;  //输出最大为 限幅 * 0.00006f, 原来为0.00004f
		//chassis_move_control->wz_set = 0.0;
		if (fabs(chassis_move_control->wz_set) < 0.001f && chassis_move_control->chassis_return_record == CHASSIS_VECTOR_RETURN)  //回正后恢复正常控制
		{
			chassis_move_control->wz_set = 0;
			chassis_move_control->chassis_return_flag = 0;  //这是在左档依旧打下时退出回正模式转入正常控制的关键
			
		/* 在电脑屏幕上添加显示 回正模式完成 的代码，如：已完成回正 */
		
		}
		
		//为满足旋转变换而定义的数组变量
		fp32 vector[2] = {vx_set, vy_set};
		
		//对速度向量旋转变换
		vector_rotate(chassis_move_control->gimbal_radian_of_ecd + CHASSIS_RETURN_OFFSET, vector);
		
		vx_set = vector[0];
		vy_set = vector[1];
		
		//速度限幅
		chassis_move_control->vx_set = fp32_constrain(vx_set, chassis_move_control->vx_min_speed, chassis_move_control->vx_max_speed);
		chassis_move_control->vy_set = fp32_constrain(vy_set, chassis_move_control->vy_min_speed, chassis_move_control->vy_max_speed);
	}
	else if (chassis_move_control->chassis_mode == CHASSIS_VECTOR_NO_MOVE)
	{
		chassis_move_control->vx_set = vx_set = 0.0f;
		chassis_move_control->vy_set = vy_set = 0.0f;
		chassis_move_control->wz_set = wz_set = 0.0f;
	}
	//跟随云台模式
	else if (chassis_move_control->chassis_mode == CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW)
	{
		chassis_move_control->wz_set = wz_set;
		
		//为满足旋转变换而定义的数组变量
		fp32 vector[2] = {vx_set, vy_set};
		
		//对速度向量旋转变换
		vector_rotate(chassis_move_control->gimbal_radian_of_ecd + chassis_follow_gimbal_yaw_offset, vector);
		
		vx_set = vector[0];
		vy_set = vector[1];
		
		//速度限幅
		chassis_move_control->vx_set = fp32_constrain(vx_set, chassis_move_control->vx_min_speed, chassis_move_control->vx_max_speed);
		chassis_move_control->vy_set = fp32_constrain(vy_set, chassis_move_control->vy_min_speed, chassis_move_control->vy_max_speed);
	}
	//底盘旋转
	else if (chassis_move_control->chassis_mode == CHASSIS_VECTOR_SPIN)
	{
		//“wz_set”是旋转速度设置
		chassis_move_control->wz_set = wz_set;
		
		if (chassis_move_control->chassis_mode == CHASSIS_VECTOR_RETURN)  
		{
			chassis_move_control->wz_set = 0;
		}
		//为满足旋转变换而定义的数组变量
		fp32 vector[2] = {vx_set, vy_set};
		
		//对速度向量旋转变换
		vector_rotate(chassis_move_control->gimbal_radian_of_ecd + chassis_spin_offset, vector);
		
		vx_set = vector[0];
		vy_set = vector[1];
		
		//速度限幅
		chassis_move_control->vx_set = fp32_constrain(vx_set, chassis_move_control->vx_min_speed, chassis_move_control->vx_max_speed);
		chassis_move_control->vy_set = fp32_constrain(vy_set, chassis_move_control->vy_min_speed, chassis_move_control->vy_max_speed);
	}
}

//把小写的变量赋值成对应宏，用于调试，调试结束后应删除变量使用宏定义
fp32 chassis_return_target = CHASSIS_RETURN_TARGET;

/*************************************************************
  * @brief          基于物理模型的电机控制计算
  * @param[in]			motor_idx	电机索引 (0-3)
  * @param[in]			set_speed	目标速度 (m/s)
  * @param[in]			ref_speed	当前速度 (m/s)
  * @retval         电机控制电流值 (fp32)
  ************************************************************/
static fp32 Model_Based_Control(uint8_t motor_idx, fp32 set_speed, fp32 ref_speed)
{
    fp32 error_v = set_speed - ref_speed;
    
    // 1. 计算期望加速度 (目标速度 - 当前速度) / 时间常数
    // 时间常数越小，响应越快，但也越容易导致加速度跳变
    fp32 accel_target = error_v / CONTROL_PERIOD_MODEL;
    
    // 2. 引入加加速度限制 (Jerk Limit) 使加速度平滑上升
    // 计算加速度的变化量
    fp32 accel_diff = accel_target - chassis_move.model_accel[motor_idx];
    
    // 限制加速度的变化率 (Jerk * dt)
    fp32 max_accel_diff = CHASSIS_MAX_JERK * CHASSIS_CONTROL_TIME;
    if (accel_diff > max_accel_diff) accel_diff = max_accel_diff;
    else if (accel_diff < -max_accel_diff) accel_diff = -max_accel_diff;
    
    // 更新当前加速度
    chassis_move.model_accel[motor_idx] += accel_diff;
    
    // 3. 加速度绝对值限幅
    if (chassis_move.model_accel[motor_idx] > CHASSIS_MAX_ACCEL) 
        chassis_move.model_accel[motor_idx] = CHASSIS_MAX_ACCEL;
    else if (chassis_move.model_accel[motor_idx] < -CHASSIS_MAX_ACCEL) 
        chassis_move.model_accel[motor_idx] = -CHASSIS_MAX_ACCEL;

    // 4. 计算牵引力 (F = m * a)
    fp32 F_traction = (ROBOT_MASS / 4.0f) * chassis_move.model_accel[motor_idx];

    // 5. 摩擦补偿逻辑：基于当前速度方向
    fp32 I_friction = 0.0f;
    if (fabsf(ref_speed) < FRICTION_SPEED_BAND)
    {
        // 零速附近的线性过渡区域
        I_friction = ref_speed * FRICTION_LINEAR_GAIN;
    }
    else
    {
        // 常态摩擦补偿（根据速度正负）
        I_friction = sign(ref_speed) * FRICTION_CONSTANT_CURRENT;
    }

    // 6. 小P补偿：在目标速度附近辅助平衡摩擦力
    fp32 I_hold_p = 0.0f;
    if (fabsf(error_v) < SPEED_HOLD_ERROR_THRESHOLD)
    {
        I_hold_p = error_v * SPEED_HOLD_KP;
    }

    // 7. 总力仅由牵引力计算
    fp32 F_total = F_traction;
    
    // 8. 力矩转换: 
    // F = (Torque_output * Efficiency) / Radius
    // 反推: Torque_output = (F * Radius) / Efficiency
    // 注意：此处力矩已对应输出轴，不再包含减速比
    fp32 Torque_output = (F_total * Wheel_Radius) / CHASSIS_EFFICIENCY;
    if (Torque_output > M3508_MAX_CONT_TORQUE * M3508_REDUCTION_RATIO) 
        Torque_output = M3508_MAX_CONT_TORQUE * M3508_REDUCTION_RATIO;
    else if (Torque_output < -M3508_MAX_CONT_TORQUE * M3508_REDUCTION_RATIO) 
        Torque_output = -M3508_MAX_CONT_TORQUE * M3508_REDUCTION_RATIO;
    
    // 9. 电流转换: I = Torque / Kt
    fp32 Current_A = Torque_output / M3508_TORQUE_CONSTANT;
    
    // 10. 映射到电调控制值: 20A -> 16384，并叠加摩擦补偿和小P补偿
    fp32 out = Current_A * (16384.0f / 20.0f) + I_friction + I_hold_p;
    
    // 总输出限幅
    if (out > M3505_MOTOR_SPEED_PID_MAX_OUT) out = M3505_MOTOR_SPEED_PID_MAX_OUT;
    else if (out < -M3505_MOTOR_SPEED_PID_MAX_OUT) out = -M3505_MOTOR_SPEED_PID_MAX_OUT;
    
    return out;
}

/*************************************************************
  * @brief          底盘在抑制舵向电机角度跳变下的PID计算
  * @param[in]			chassis_pid_calc	底盘运动数据结构体指针
  * @retval         none
  ************************************************************/
static void PID_Calc_Jump(chassis_move_t *chassis_pid_calc)
{
	static int32_t count = 0;
	
	for(uint8_t i = 0; i < 4; i++)
	{
		// 云台回正模式的角度处理
		if (chassis_pid_calc->chassis_mode == CHASSIS_VECTOR_RETURN)
		{
			fp32 target = rad_format(chassis_return_target);  // 直接使用宏
			fp32 actual = rad_format(chassis_pid_calc->gimbal_radian_of_ecd);
			
			// 避免角度跳变
			if(fabs(actual - target) > PI)
			{
				fp32 adjusted_target = (target < 0) ? target + 2*PI : target - 2*PI;
				chassis_pid_calc->return_wz_set = PID_calc(&chassis_pid_calc->chas_return_pid, actual, adjusted_target);
			}
			else
			{
				chassis_pid_calc->return_wz_set = PID_calc(&chassis_pid_calc->chas_return_pid, actual, target);
			}
  	}
		
		// 无控制状态下的角度处理
		bool no_control = (fabsf(chassis_pid_calc->vx_set) < 0.1f && fabsf(chassis_pid_calc->vy_set) < 0.1f && fabsf(chassis_pid_calc->wz_set) < 0.001f);
		
		if(no_control)
		{
			switch(chassis_pid_calc->chassis_mode)
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
		else  // 有控制时记录角度
		{
			chassis_pid_calc->wheel_angle_offset.last[i] = chassis_pid_calc->chassis_6020[i].angle;
		}
		
		chassis_pid_calc->chassis_return_record = chassis_pid_calc->chassis_mode;
		
		// 3508电机基于物理模型的计算 (牵引力前馈 + 阻力动态累积)
		chassis_pid_calc->model_3508_out[i] = Model_Based_Control(i, chassis_pid_calc->chassis_3508[i].speed_set, chassis_pid_calc->chassis_3508[i].speed);
		
		// 6020电机角度和速度PID计算
		fp32 angle_error = chassis_pid_calc->chassis_6020[i].angle - chassis_pid_calc->chassis_6020[i].angle_set;
		fp32 angle_set_adjusted = chassis_pid_calc->chassis_6020[i].angle_set;
		
		if(fabs(angle_error) > PI)
		{
			angle_set_adjusted += (chassis_pid_calc->chassis_6020[i].angle_set < 0) ? 2*PI : -2*PI;
		}
		
		PID_calc(&chassis_pid_calc->chas_6020_angle_pid[i], chassis_pid_calc->chassis_6020[i].angle, angle_set_adjusted);
		PID_calc(&chassis_pid_calc->chas_6020_speed_pid[i], chassis_pid_calc->chassis_6020[i].chassis_motor_measure->speed_rpm, chassis_pid_calc->chas_6020_angle_pid[i].out);
	}
}

/******************************** 核心 ********************************
  * @brief          控制循环，根据控制设定值，计算电机电流值，进行控制
  * @param[out]     chassis_move_control_loop:"chassis_move"变量指针.
  * @retval         none
  *********************************************************************/
static void chassis_control_loop(chassis_move_t *chassis_move_control_loop)
{
	fp32 wheel_speed[4] = {0.0f, 0.0f, 0.0f, 0.0f};
	fp32 wheel_angle[4] = {0.0f, 0.0f, 0.0f, 0.0f};
	
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
	
	
	//底盘舵轮运动学逆解算
	chas_inv_cal(chassis_move_control_loop->vx_set,
							 chassis_move_control_loop->vy_set,
							 chassis_move_control_loop->wz_set,
							 wheel_angle, wheel_speed);
		
	//将算出的舵轮目标角度和目标速度赋值给angle_set、speed_set
	for(uint8_t i = 0; i < 4; i ++)
	{
		chassis_move_control_loop->chassis_6020[i].angle_set = wheel_angle[i];
		chassis_move_control_loop->chassis_3508[i].speed_set = wheel_speed[i];
	}
	
//	//打滑抑制
//	slip_control(chassis_move_control_loop);
	
	//底盘在抑制舵向电机角度跳变下的PID计算，定义在这个函数前面
	PID_Calc_Jump(chassis_move_control_loop);
	
	//功率控制(在 chassis_power_control() 函数中给舵轮3508和6020电机赋值电流值)
	chassis_power_control(chassis_move_control_loop);
}

/**
  * @brief          底盘任务，间隔 CHASSIS_CONTROL_TIME_MS 2ms
  * @param[in]      pvParameters: 空
  * @retval         none
  */
void chassis_task(void const *pvParameters)
{
	vTaskDelay(CHASSIS_TASK_INIT_TIME);	//空闲一段时间
	chassis_init(&chassis_move); //底盘初始化
	
	//判断底盘电机是否都在线
	while (toe_is_error(CHASSIS_MOTOR1_TOE) || toe_is_error(CHASSIS_MOTOR2_TOE) || toe_is_error(CHASSIS_MOTOR3_TOE) || toe_is_error(CHASSIS_MOTOR4_TOE) || \
				 toe_is_error(CHASSIS_MOTOR5_TOE) || toe_is_error(CHASSIS_MOTOR6_TOE) || toe_is_error(CHASSIS_MOTOR7_TOE) || toe_is_error(CHASSIS_MOTOR8_TOE) || \
				 toe_is_error(DBUS_TOE))
	{ vTaskDelay(CHASSIS_CONTROL_TIME_MS); }
	
	while (1)
	{
		//设置底盘控制模式
		chassis_set_mode(&chassis_move);
		//模式切换数据保存
		chassis_mode_change_control_transit(&chassis_move);
		//底盘数据更新
		chassis_feedback_update(&chassis_move);
		//底盘控制量设置
		chassis_set_contorl(&chassis_move);
		//底盘控制PID计算
		chassis_control_loop(&chassis_move);

		//确保至少一个电机在线， 这样CAN控制包可以被接收到
		if (!(toe_is_error(CHASSIS_MOTOR1_TOE) && toe_is_error(CHASSIS_MOTOR2_TOE) && toe_is_error(CHASSIS_MOTOR3_TOE) && toe_is_error(CHASSIS_MOTOR4_TOE) && \
					toe_is_error(CHASSIS_MOTOR5_TOE) && toe_is_error(CHASSIS_MOTOR6_TOE) && toe_is_error(CHASSIS_MOTOR7_TOE) && toe_is_error(CHASSIS_MOTOR8_TOE)))
		{
			//当遥控器掉线的时候，发送给底盘电机零电流.
			if (toe_is_error(DBUS_TOE))
			{
				CAN_cmd_CHAS_6020(0, 0, 0, 0);
				CAN_cmd_CHAS_3508(0, 0, 0, 0);
			}
			else
			{

				//发送控制电流
				CAN_cmd_CHAS_6020(chassis_move.chassis_6020[0].give_current, chassis_move.chassis_6020[1].give_current,
													chassis_move.chassis_6020[2].give_current, chassis_move.chassis_6020[3].give_current);
				CAN_cmd_CHAS_3508(chassis_move.chassis_3508[0].give_current,chassis_move.chassis_3508[1].give_current,
													chassis_move.chassis_3508[2].give_current,chassis_move.chassis_3508[3].give_current);
				
			}
		}
		//系统延时
		//vTaskDelay(CHASSIS_CONTROL_TIME_MS);
		osDelay(1);

		#if INCLUDE_uxTaskGetStackHighWaterMark
			chassis_high_water = uxTaskGetStackHighWaterMark(NULL);
		#endif
	}
}
