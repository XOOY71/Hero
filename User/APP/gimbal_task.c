/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       gimbal_task.c
  * @brief      minimal gimbal control framework
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "gimbal_task.h"
#include "gimbal_behaviour.h"
#include "cmsis_os.h"
#include <math.h>
#include <stddef.h>
#include "vofa.h"
gimbal_control_t gimbal_control;

/* 模块私有线程句柄 */
static osThreadId gimbalTaskHandle = NULL;

/* 模块私有任务入口声明 */
static void gimbal_task(void const *pvParameters);

#define GIMBAL_PI 3.14159265358979323846f
#define GIMBAL_PID_DEFAULT_LIMIT 1000000.0f

static float gimbal_wrap_angle(float angle)
{
    while (angle > GIMBAL_PI)
    {
        angle -= 2.0f * GIMBAL_PI;
    }
    while (angle < -GIMBAL_PI)
    {
        angle += 2.0f * GIMBAL_PI;
    }
    return angle;
}

static float gimbal_clamp(float value, float min_value, float max_value)
{
    if (value > max_value)
    {
        return max_value;
    }
    if (value < min_value)
    {
        return min_value;
    }
    return value;
}

/* 对外初始化接口：由 freertos.c 调用 */
void GimbalTask_Init(void)
{
    osThreadDef(gimbalTask, gimbal_task, osPriorityHigh, 0, 1024);
    gimbalTaskHandle = osThreadCreate(osThread(gimbalTask), NULL);
}

/**
  * @brief          模块私有云台任务主流程
  * @param[in]      none
  * @retval         none
  */

static void gimbal_task(void const *pvParameters)
{
    (void)pvParameters;

	vTaskDelay(GIMBAL_TASK_INIT_TIME);
	gimbal_init(&gimbal_control);
	while (1)
	{												 
    gimbal_set_mode(&gimbal_control);                      //设置云台控制模式
    gimbal_mode_change_control_transit(&gimbal_control);   //控制模式切换 控制数据过渡
    gimbal_feedback_update(&gimbal_control);               //云台数据反馈
    gimbal_set_control(&gimbal_control);                   //云台控制量
    gimbal_control_loop(&gimbal_control);                  //云台控制PID计算
    gimbal_send_cmd(&gimbal_control);                      
		
		VOFA_Send6(gimbal_control.gimbal_rc_ctrl->rc.ch[0],
								gimbal_control.gimbal_rc_ctrl->rc.ch[1],
								gimbal_control.gimbal_rc_ctrl->rc.ch[2],
								gimbal_control.gimbal_rc_ctrl->rc.ch[3],
								0,
								0);
		
		vTaskDelay(GIMBAL_CONTROL_TIME);
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
  * @retval         pitch电机指针
  */
const gimbal_motor_t *get_pitch_motor_point(void)
{
    return &gimbal_control.gimbal_pitch_motor;
}

/**
  * @brief          云台控制模式:GIMBAL_MOTOR_GYRO，更新绝对角目标
  * @param[out]     motor:yaw电机或者pitch电机
  * @param[in]      add:角度增量
  * @retval         none
  */
void gimbal_absolute_angle_limit(gimbal_motor_t *motor, float add)
{
    if (motor == 0)
    {
        return;
    }

    motor->absolute_angle_set = gimbal_wrap_angle(motor->absolute_angle_set + add);
}

/**
  * @brief          云台控制模式:GIMBAL_MOTOR_ENCODE，更新相对角目标
  * @param[out]     motor:yaw电机或者pitch电机
  * @param[in]      add:角度增量
  * @retval         none
  */
void gimbal_relative_angle_limit(gimbal_motor_t *motor, float add)
{
    if (motor == 0)
    {
        return;
    }

    motor->relative_angle_set += add;
    motor->relative_angle_set = gimbal_clamp(motor->relative_angle_set, motor->min_relative_angle, motor->max_relative_angle);
}

/**
  * @brief          云台控制模式:GIMBAL_MOTOR_GYRO
  * @param[out]     motor:yaw电机或者pitch电机
  * @retval         none
  */
void gimbal_motor_absolute_angle_control(gimbal_motor_t *motor)
{
    if (motor == 0)
    {
        return;
    }

    motor->absolute_angle_set = gimbal_wrap_angle(motor->absolute_angle_set);
    motor->absolute_angle = gimbal_wrap_angle(motor->absolute_angle);

    motor->gyro_set = gimbal_pid_calc(&motor->absolute_angle_pid, motor->absolute_angle, motor->absolute_angle_set, motor->gyro);
    motor->current_set = gimbal_pid_calc(&motor->gyro_pid, motor->gyro, motor->gyro_set, 0.0f);
    motor->output = motor->current_set;
    motor->given_current = (int16_t)(motor->output);
}

/**
  * @brief          云台控制模式:GIMBAL_MOTOR_ENCODE
  * @param[out]     motor:yaw电机或者pitch电机
  * @retval         none
  */
void gimbal_motor_relative_angle_control(gimbal_motor_t *motor)
{
    if (motor == 0)
    {
        return;
    }

    motor->gyro_set = gimbal_pid_calc(&motor->relative_angle_pid, motor->relative_angle, motor->relative_angle_set, motor->gyro);
    motor->current_set = gimbal_pid_calc(&motor->gyro_pid, motor->gyro, motor->gyro_set, 0.0f);
    motor->output = motor->current_set;
    motor->given_current = (int16_t)(motor->output);
}

/**
  * @brief          云台控制模式:GIMBAL_MOTOR_RAW
  * @param[out]     motor:yaw电机或者pitch电机
  * @retval         none
  */
void gimbal_motor_raw_angle_control(gimbal_motor_t *motor)
{
    if (motor == 0)
    {
        return;
    }

    motor->current_set = motor->raw_cmd;
    motor->output = motor->raw_cmd;
    motor->given_current = (int16_t)motor->output;
}

/**
  * @brief          简化PID初始化
  * @param[out]     pid:PID结构体指针
  * @param[in]      kp,ki,kd: PID参数
  * @retval         none
  */
void gimbal_pid_init(gimbal_pid_t *pid, float kp, float ki, float kd)
{
    if (pid == 0)
    {
        return;
    }

    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;

    pid->set = 0.0f;
    pid->get = 0.0f;
    pid->err = 0.0f;
    pid->last_err = 0.0f;
    pid->iout = 0.0f;
    pid->out = 0.0f;

    pid->max_out = GIMBAL_PID_DEFAULT_LIMIT;
    pid->max_iout = GIMBAL_PID_DEFAULT_LIMIT;
}

/**
  * @brief          简化PID清零
  * @param[out]     pid:PID结构体指针
  * @retval         none
  */
void gimbal_pid_clear(gimbal_pid_t *pid)
{
    if (pid == 0)
    {
        return;
    }

    pid->set = 0.0f;
    pid->get = 0.0f;
    pid->err = 0.0f;
    pid->last_err = 0.0f;
    pid->iout = 0.0f;
    pid->out = 0.0f;
}

/**
  * @brief          简化PID计算接口
  * @param[out]     pid:PID结构体指针
  * @param[in]      get,set,error_delta
  * @retval         PID输出
  */
float gimbal_pid_calc(gimbal_pid_t *pid, float get, float set, float error_delta)
{
    float p_out;
    float d_out;

    if (pid == 0)
    {
        return 0.0f;
    }

    pid->get = get;
    pid->set = set;
    pid->err = set - get;

    p_out = pid->kp * pid->err;
    pid->iout += pid->ki * pid->err;

    if (pid->max_iout > 0.0f)
    {
        pid->iout = gimbal_clamp(pid->iout, -pid->max_iout, pid->max_iout);
    }

    if (error_delta != 0.0f)
    {
        d_out = pid->kd * error_delta;
    }
    else
    {
        d_out = pid->kd * (pid->err - pid->last_err);
    }

    pid->out = p_out + pid->iout + d_out;

    if (pid->max_out > 0.0f)
    {
        pid->out = gimbal_clamp(pid->out, -pid->max_out, pid->max_out);
    }

    pid->last_err = pid->err;
    return pid->out;
}
