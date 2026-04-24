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
#define GIMBAL_CURRENT_CMD_LIMIT 30000.0f

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

static int16_t gimbal_float_to_current(float current)
{
    current = gimbal_clamp(current, -GIMBAL_CURRENT_CMD_LIMIT, GIMBAL_CURRENT_CMD_LIMIT);
    return (int16_t)current;
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
        gimbal_set_mode(&gimbal_control);                    // 设置云台控制模式
        gimbal_feedback_update(&gimbal_control);             // 云台数据反馈
        gimbal_mode_change_control_transit(&gimbal_control); // 控制模式切换 控制数据过渡
        gimbal_set_control(&gimbal_control);                 // 云台控制量
        gimbal_control_loop(&gimbal_control);                // 云台控制PID计算
        gimbal_send_cmd(&gimbal_control);

        VOFA_Send6(gimbal_control.gimbal_yaw_motor.absolute_angle_set,
                   gimbal_control.gimbal_yaw_motor.absolute_angle,
                   gimbal_control.gimbal_pitch_motor.absolute_angle_set,
                   gimbal_control.gimbal_pitch_motor.absolute_angle,
                   gimbal_control.gimbal_yaw_motor.given_current,
                   gimbal_control.gimbal_pitch_motor.given_current);

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

    if (motor == &gimbal_control.gimbal_yaw_motor)
    {
        motor->absolute_angle_set += add;
    }
    else
    {
        motor->absolute_angle_set = gimbal_wrap_angle(motor->absolute_angle_set + add);
    }
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
    float angle_get;
    float angle_set;

    if (motor == 0)
    {
        return;
    }

    if (motor == &gimbal_control.gimbal_yaw_motor)
    {
        angle_get = motor->absolute_angle;
        angle_set = motor->absolute_angle_set;
    }
    else
    {
        angle_get = gimbal_wrap_angle(motor->absolute_angle);
        angle_set = gimbal_wrap_angle(motor->absolute_angle_set);
    }

    motor->gyro_set = gimbal_pid_calc(&motor->absolute_angle_pid, angle_get, angle_set, motor->gyro);
    motor->current_set = gimbal_pid_calc(&motor->gyro_pid, motor->gyro, motor->gyro_set, 0.0f);
    motor->output = motor->current_set;
    motor->given_current = gimbal_float_to_current(motor->output);
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
    motor->given_current = gimbal_float_to_current(motor->output);
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
    motor->given_current = gimbal_float_to_current(motor->output);
}

/**
  * @brief          简化PID初始化
  * @param[out]     pid:PID结构体指针
  * @param[in]      kp,ki,kd: PID参数
  * @retval         none
  */
void gimbal_pid_init(gimbal_pid_t *pid, float kp, float ki, float kd)
{
    float pid_param[3];

    if (pid == NULL)
    {
        return;
    }

    pid_param[0] = kp;
    pid_param[1] = ki;
    pid_param[2] = kd;
    PID_init(pid, PID_POSITION, pid_param, GIMBAL_PID_DEFAULT_LIMIT, GIMBAL_PID_DEFAULT_LIMIT);
}

/**
  * @brief          简化PID清零
  * @param[out]     pid:PID结构体指针
  * @retval         none
  */
void gimbal_pid_clear(gimbal_pid_t *pid)
{
    if (pid == NULL)
    {
        return;
    }

    PID_clear(pid);
}

/**
  * @brief          简化PID计算接口
  * @param[out]     pid:PID结构体指针
  * @param[in]      get,set,error_delta
  * @retval         PID输出
  */
float gimbal_pid_calc(gimbal_pid_t *pid, float get, float set, float error_delta)
{
    (void)error_delta;

    if (pid == NULL)
    {
        return 0.0f;
    }

    return PID_Calc(pid, get, set);
}
