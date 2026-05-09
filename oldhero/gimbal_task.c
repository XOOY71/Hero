/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       gimbal_task.c/h
  * @brief      gimbal control task, because use the euler angle calculated by
  *             gyro sensor, range (-pi,pi), angle set-point must be in this 
  *             range.gimbal has two control mode, gyro mode and enconde mode
  *             gyro mode: use euler angle to control, encond mode: use enconde
  *             angle to control. and has some special mode:cali mode, motionless
  *             mode.
  *             完成云台控制任务，由于云台使用陀螺仪解算出的角度，其范围在（-pi,pi）
  *             故而设置目标角度均为范围，存在许多对角度计算的函数。云台主要分为2种
  *             状态，陀螺仪控制状态是利用板载陀螺仪解算的姿态角进行控制，编码器控制
  *             状态是通过电机反馈的编码值控制的校准，此外还有校准状态，停止状态等。
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V1.1.0     Nov-11-2019     RM              1. add some annotation
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "arm_math.h"
#include "auto_aim.h"
#include "bsp_usart.h"
#include "can_bsp.h"
#include "cmsis_os.h"
#include "detect_task.h"
#include "gimbal_behaviour.h"
#include "gimbal_task.h"
#include "INS_task.h"
#include "main.h"
#include "pid.h"
#include "remote_control.h"
#include "robot_param.h"
#include "shoot.h"
#include "user_lib.h"
#include "yaw_pitch_direct.h"

#if  ROBOT_TYPE   ==   Infantry_robot
#include "Infantry_robot.h"
#endif

#if  ROBOT_TYPE   ==   Hero_robot
#include "Hero.h"
#endif

#if  ROBOT_TYPE   ==   Balance_robot
#include "Balance.h"
#endif

#if  ROBOT_TYPE   ==   Sentinel_robot
#include "Sentinel.h"
#endif

#if  ROBOT_TYPE   ==   Engineer_robot
#include "Engineer.h"
#endif

#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t gimbal_high_water;
#endif

#define LimitMax(input, max)   			\
    {                          			\
        if ((input) > (max))       	\
        {                      			\
            (input) = (max);       	\
        }                      			\
        else if (input < -(max)) 		\
        {                      			\
            (input) = -(max);      	\
        }                      			\
    }

// Outer angle-loop integral management near zero crossing
#define GIMBAL_I_FREEZE_ERR   (0.8f * PI / 180.0f)
#define GIMBAL_I_ACTIVE_ERR   (3.0f * PI / 180.0f)
#define GIMBAL_I_FREEZE_W     (6.0f * PI / 180.0f)
#define GIMBAL_I_LEAK         (0.98f)

//让云台保持在固定的值
fp32 yaw_target = 0;
fp32 aim_flag = 0;
fp32 yaw_curret = 0;
uint8_t auto_aim_flag = 0;
pid_type_def AIM;


__weak void gimbal_init(gimbal_control_t *init);
__weak void gimbal_set_mode(gimbal_control_t *set_mode);
__weak void gimbal_feedback_update(gimbal_control_t *feedback_update);
__weak void gimbal_mode_change_control_transit(gimbal_control_t *mode_change);
fp32 motor_ecd_to_angle_change(uint16_t ecd, uint16_t offset_ecd);
__weak void gimbal_set_control(gimbal_control_t *set_control);
__weak void gimbal_control_loop(gimbal_control_t *control_loop);
__weak void gimbal_send_cmd(gimbal_control_t *control_send);
void gimbal_motor_absolute_angle_control(gimbal_motor_t *gimbal_motor);
void gimbal_motor_relative_angle_control(gimbal_motor_t *gimbal_motor);
void gimbal_motor_raw_angle_control(gimbal_motor_t *gimbal_motor);
void gimbal_absolute_angle_limit(gimbal_motor_t *gimbal_motor, fp32 add);
void gimbal_relative_angle_limit(gimbal_motor_t *gimbal_motor, fp32 add);
void gimbal_PID_init(gimbal_PID_t *pid, fp32 maxout, fp32 intergral_limit, fp32 kp, fp32 ki, fp32 kd);
void gimbal_PID_clear(gimbal_PID_t *pid_clear);
fp32 gimbal_PID_Calc(gimbal_PID_t *pid, fp32 get, fp32 set, fp32 error_delta);
void calc_gimbal_cali(const gimbal_step_cali_t *gimbal_cali, uint16_t *yaw_offset, uint16_t *pitch_offset, fp32 *max_yaw, fp32 *min_yaw, fp32 *max_pitch, fp32 *min_pitch);


#if GIMBAL_TEST_MODE
//j-scope 帮助pid调参
static void J_scope_gimbal_test(void);
#endif

//gimbal control data
//云台控制所有相关数据
gimbal_control_t gimbal_control;

extern int16_t yaw_can_set_current, pitch_can_set_current, shoot_can_set_current;


/**
  * @brief          云台任务，间隔 GIMBAL_CONTROL_TIME 1ms
  * @param[in]      pvParameters: 空
  * @retval         none
  */

void gimbal_task(void const *pvParameters)
{
	vTaskDelay(GIMBAL_TASK_INIT_TIME);
	gimbal_init(&gimbal_control);
	shoot_init();

	while (1)
	{
		gimbal_set_mode(&gimbal_control);                    //设置云台控制模式
		gimbal_mode_change_control_transit(&gimbal_control); //控制模式切换 控制数据过渡
		gimbal_feedback_update(&gimbal_control);             //云台数据反馈
		gimbal_set_control(&gimbal_control);                 //设置云台控制量
		gimbal_control_loop(&gimbal_control);                //云台控制PID计算
		shoot_can_set_current = shoot_control_loop();        //射击任务控制循环
		gimbal_send_cmd(&gimbal_control);
		
		#if GIMBAL_TEST_MODE
			J_scope_gimbal_test();
		#endif

		vTaskDelay(GIMBAL_CONTROL_TIME);

		#if INCLUDE_uxTaskGetStackHighWaterMark
			gimbal_high_water = uxTaskGetStackHighWaterMark(NULL);
		#endif
	}
}

/**
  * @brief          返回yaw 电机数据指针
  * @param[in]      none
  * @retval         yaw电机指针
  */
const gimbal_motor_t *get_yaw_motor_point(void)
{
    return &gimbal_control.gimbal_yaw_motor;
}

/**
  * @brief          返回pitch 电机数据指针
  * @param[in]      none
  * @retval         pitch
  */
const gimbal_motor_t *get_pitch_motor_point(void)
{
    return &gimbal_control.gimbal_pitch_motor;
}

/**
  * @brief          计算ecd与offset_ecd之间的相对角度
  * @param[in]      ecd: 电机当前编码
  * @param[in]      offset_ecd: 电机中值编码
  * @retval         相对角度，单位rad
  */
fp32 motor_ecd_to_angle_change(uint16_t ecd, uint16_t offset_ecd)
{
    int32_t relative_ecd = ecd - offset_ecd;
		//while(relative_ecd > 0 && )
    if (relative_ecd > HALF_ECD_RANGE)
    {
        relative_ecd -= ECD_RANGE;
    }
    else if (relative_ecd < -HALF_ECD_RANGE)
    {
        relative_ecd += ECD_RANGE;
    }

    return relative_ecd * MOTOR_ECD_TO_RAD;
}

/**
  * @brief          云台控制模式:GIMBAL_MOTOR_GYRO，使用陀螺仪计算的欧拉角进行控制
  * @param[out]     gimbal_motor:yaw电机或者pitch电机
  * @retval         none
  */
void gimbal_absolute_angle_limit(gimbal_motor_t *gimbal_motor, fp32 add)
{
	fp32 bias_angle = 0.0f;
	fp32 angle_set;
	if (gimbal_motor == NULL)
	{
			return;
	}
	//自瞄这一块
	if(gimbal_motor == &gimbal_control.gimbal_yaw_motor)
	{
		bias_angle = aim.receive.yaw;
		aim.receive.yaw = 0.0f;
	}
	else if(gimbal_motor == &gimbal_control.gimbal_pitch_motor)
	{			
		bias_angle = aim.receive.pitch;
		aim.receive.pitch = 0.0f;
	}
	
	angle_set = gimbal_motor->absolute_angle_set + bias_angle + add;
	aim.yaw_delay   = bias_angle;
	aim.pitch_delay = angle_set;
//	aim.shoot_delay = gimbal_motor->absolute_angle_set;
	gimbal_motor->absolute_angle_set = rad_format(angle_set );
}

/**
  * @brief          云台控制模式:GIMBAL_MOTOR_ENCONDE，使用编码相对角进行控制
  * @param[out]     gimbal_motor:yaw电机或者pitch电机
  * @retval         none
  */
void gimbal_relative_angle_limit(gimbal_motor_t *gimbal_motor, fp32 add)
{
	static fp32 bias_angle;
	if (gimbal_motor == NULL)
	{
		return;
	}

	//自瞄这一块
	if(gimbal_motor == &gimbal_control.gimbal_yaw_motor)
	{
		bias_angle = aim.receive.yaw;
		aim.receive.yaw = 0.0f;
	}
	else if(gimbal_motor == &gimbal_control.gimbal_pitch_motor)
	{
		bias_angle = aim.receive.pitch;
		aim.receive.pitch = 0.0f;
	}

	gimbal_motor->relative_angle_set = gimbal_motor->relative_angle_set + add + bias_angle;
	gimbal_motor->relative_angle_set = rad_format(gimbal_motor->relative_angle_set);
	//是否超过最大 最小值
	if (gimbal_motor->relative_angle_set > gimbal_motor->max_relative_angle)
	{
		gimbal_motor->relative_angle_set = gimbal_motor->max_relative_angle;
	}
	else if (gimbal_motor->relative_angle_set < gimbal_motor->min_relative_angle)
	{
		gimbal_motor->relative_angle_set = gimbal_motor->min_relative_angle;
	}
}

/**
  * @brief          云台控制模式:GIMBAL_MOTOR_GYRO，使用陀螺仪计算的欧拉角进行控制
  * @param[out]     gimbal_motor:yaw电机或者pitch电机
  * @retval         none
  */
__attribute__((used))void gimbal_motor_absolute_angle_control(gimbal_motor_t *gimbal_motor)
{
	fp32 out = 0;
	const static fp32 feed = 0.0f;
	
	if (gimbal_motor == NULL) { return; }
	
	gimbal_motor->absolute_angle_set = rad_format(gimbal_motor->absolute_angle_set);
	gimbal_motor->absolute_angle = rad_format(gimbal_motor->absolute_angle);
	
	if(fabs(gimbal_motor->absolute_angle - gimbal_motor->absolute_angle_set) > PI)
	{
		if(gimbal_motor->absolute_angle_set < 0)
		{
			out = gimbal_PID_Calc(&gimbal_motor->gimbal_motor_absolute_angle_pid, gimbal_motor->absolute_angle, gimbal_motor->absolute_angle_set + 2*PI, gimbal_motor->motor_gyro);
		}
		else
		{
			out = gimbal_PID_Calc(&gimbal_motor->gimbal_motor_absolute_angle_pid, gimbal_motor->absolute_angle, gimbal_motor->absolute_angle_set - 2*PI, gimbal_motor->motor_gyro);
		}
	}
	else
	{
		out = gimbal_PID_Calc(&gimbal_motor->gimbal_motor_absolute_angle_pid, gimbal_motor->absolute_angle, gimbal_motor->absolute_angle_set, gimbal_motor->motor_gyro);
	}
	if(gimbal_motor == &gimbal_control.gimbal_yaw_motor && YAW_TURN == 1){ out = -out; }
	if(gimbal_motor == &gimbal_control.gimbal_pitch_motor && PITCH_TURN == 1){ out = -out; }
	
	gimbal_motor->motor_gyro_set = out;
	
	if(gimbal_motor->motor_gyro_set > 0.0f){ gimbal_motor->motor_gyro_set += feed; }
	else if(gimbal_motor->motor_gyro_set < 0.0f){ gimbal_motor->motor_gyro_set -= feed; }
	
	gimbal_motor->current_set = PID_Calc(&gimbal_motor->gimbal_motor_gyro_pid, gimbal_motor->motor_gyro, gimbal_motor->motor_gyro_set);
	//控制值赋值
	gimbal_motor->given_current = (int16_t)(gimbal_motor->current_set);
}

#include <stm32h7xx.h>

/**
  * @brief          云台控制模式:GIMBAL_MOTOR_ENCONDE，使用编码相对角进行控制
  * @param[out]     gimbal_motor:yaw电机或者pitch电机
  * @retval         none
  */
void gimbal_motor_relative_angle_control(gimbal_motor_t *gimbal_motor)
{
		
		fp32 out = 0;
    if (gimbal_motor == NULL)
    {
      return;
    }
		
    //角度环，速度环串级pid调试
		out = gimbal_PID_Calc(&gimbal_motor->gimbal_motor_relative_angle_pid, gimbal_motor->relative_angle, gimbal_motor->relative_angle_set, gimbal_motor->motor_gyro);
		if(gimbal_motor == &gimbal_control.gimbal_yaw_motor &&YAW_TURN == 1){
			out = -out;
		}
		if(gimbal_motor == &gimbal_control.gimbal_pitch_motor && PITCH_TURN == 1){
			out = -out;
		}
    gimbal_motor->motor_gyro_set = out; 
    gimbal_motor->current_set = PID_Calc(&gimbal_motor->gimbal_motor_gyro_pid, gimbal_motor->motor_gyro, gimbal_motor->motor_gyro_set);
    //控制值赋值
		gimbal_motor->given_current = (int16_t)(gimbal_motor->current_set);
}

/**
  * @brief          云台控制模式:GIMBAL_MOTOR_RAW，电流值直接发送到CAN总线.
  * @param[out]     gimbal_motor:yaw电机或者pitch电机
  * @retval         none
  */
void gimbal_motor_raw_angle_control(gimbal_motor_t *gimbal_motor)
{
    if (gimbal_motor == NULL)
    {
        return;
    }
    gimbal_motor->current_set = gimbal_motor->raw_cmd_current;
    gimbal_motor->given_current = (int16_t)(gimbal_motor->current_set);
}

#if GIMBAL_TEST_MODE
int32_t yaw_ins_int_1000, pitch_ins_int_1000;
int32_t yaw_ins_set_1000, pitch_ins_set_1000;
int32_t pitch_relative_set_1000, pitch_relative_angle_1000;
int32_t yaw_speed_int_1000, pitch_speed_int_1000;
int32_t yaw_speed_set_int_1000, pitch_speed_set_int_1000;
static void J_scope_gimbal_test(void)
{
    yaw_ins_int_1000 = (int32_t)(gimbal_control.gimbal_yaw_motor.absolute_angle * 1000);
    yaw_ins_set_1000 = (int32_t)(gimbal_control.gimbal_yaw_motor.absolute_angle_set * 1000);
    yaw_speed_int_1000 = (int32_t)(gimbal_control.gimbal_yaw_motor.motor_gyro * 1000);
    yaw_speed_set_int_1000 = (int32_t)(gimbal_control.gimbal_yaw_motor.motor_gyro_set * 1000);

    pitch_ins_int_1000 = (int32_t)(gimbal_control.gimbal_pitch_motor.absolute_angle * 1000);
    pitch_ins_set_1000 = (int32_t)(gimbal_control.gimbal_pitch_motor.absolute_angle_set * 1000);
    pitch_speed_int_1000 = (int32_t)(gimbal_control.gimbal_pitch_motor.motor_gyro * 1000);
    pitch_speed_set_int_1000 = (int32_t)(gimbal_control.gimbal_pitch_motor.motor_gyro_set * 1000);
    pitch_relative_angle_1000 = (int32_t)(gimbal_control.gimbal_pitch_motor.relative_angle * 1000);
    pitch_relative_set_1000 = (int32_t)(gimbal_control.gimbal_pitch_motor.relative_angle_set * 1000);
}

#endif

/**
  * @brief          初始化"gimbal_control"变量，包括pid初始化， 遥控器指针初始化，云台电机指针初始化，陀螺仪角度指针初始化
  * @param[out]     gimbal_init:"gimbal_control"变量指针.
  * @retval         none
  */
void gimbal_PID_init(gimbal_PID_t *pid, fp32 maxout, fp32 max_iout, fp32 kp, fp32 ki, fp32 kd)
{
    if (pid == NULL)
    {
        return;
    }
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;

    pid->err = 0.0f;
    pid->get = 0.0f;

    pid->max_iout = max_iout;
    pid->max_out = maxout;
}

fp32 gimbal_PID_Calc(gimbal_PID_t *pid, fp32 get, fp32 set, fp32 error_delta)
{
    fp32 err;
    if (pid == NULL)
    {
        return 0.0f;
    }
    pid->get = get;
    pid->set = set;

    err = set - get;
    pid->err = rad_format(err);
    pid->Pout = pid->kp * pid->err;

    // Near zero error and low speed: freeze and leak integral to avoid "wind-up then release" oscillation.
    if (fabs(pid->err) < GIMBAL_I_FREEZE_ERR &&
        fabs(error_delta) < GIMBAL_I_FREEZE_W)
    {
        pid->Iout *= GIMBAL_I_LEAK;
    }
    // Keep current one-sided integral strategy, but limit it to medium-error region.
    else if (pid->err > 0.05f && fabs(pid->err) < GIMBAL_I_ACTIVE_ERR)
    {
        pid->Iout += pid->ki * pid->err;
    }

    pid->Dout = pid->kd * error_delta;
    LimitMax(pid->Iout, pid->max_iout);
    pid->out = pid->Pout + pid->Iout + pid->Dout;
    LimitMax(pid->out, pid->max_out);
    return pid->out;
}

/**
  * @brief          云台PID清除，清除pid的out,iout
  * @param[out]     gimbal_pid_clear:"gimbal_control"变量指针.
  * @retval         none
  */
void gimbal_PID_clear(gimbal_PID_t *gimbal_pid_clear)
{
    if (gimbal_pid_clear == NULL)
    {
        return;
    }
    gimbal_pid_clear->err = gimbal_pid_clear->set = gimbal_pid_clear->get = 0.0f;
    gimbal_pid_clear->out = gimbal_pid_clear->Pout = gimbal_pid_clear->Iout = gimbal_pid_clear->Dout = 0.0f;
}
