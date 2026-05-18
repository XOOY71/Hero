/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       gimbal_task.c
  * @brief      minimal gimbal control framework
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "gimbal_task.h"
#include "auto_aim.h"
#include "gimbal_behaviour.h"
#include "gravity_comp.h"
#include "shoot_task.h"
#include "bsp_fdcan.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include <math.h>
#include <stddef.h>
#include "vofa.h"

gimbal_control_t gimbal_control;

#ifndef GIMBAL_YAW_MIT_INDEX
#define GIMBAL_YAW_MIT_INDEX 0U
#endif

static osThreadId gimbalTaskHandle = NULL;

static void gimbal_task(void const *pvParameters);

#define GIMBAL_PI 3.14159265358979323846f
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

static float gimbal_take_auto_aim_bias(gimbal_motor_t *motor)
{
    float bias = 0.0f;

    (void)motor;

    return bias;
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

static float gimbal_float_to_torque_cmd(float output)
{
    return gimbal_clamp(output, T_MIN, T_MAX);
}

static float gimbal_calc_feedforward(gimbal_motor_t *motor)
{
    float velocity_torque = 0.0f;

    if (motor == 0)
    {
        return 0.0f;
    }

    if (motor == &gimbal_control.gimbal_yaw_motor)
    {
        velocity_torque = YAW_VELOCITY_FF_GAIN * motor->ref_vel;
    }
    else if (motor == &gimbal_control.gimbal_pitch_motor)
    {
        velocity_torque = PITCH_VELOCITY_FF_GAIN * motor->ref_vel;
    }

    motor->ff_torque = velocity_torque + motor->inertia_kgm2 * motor->ref_accel;

    return motor->ff_torque;
}

static float gimbal_calc_feedback_torque(gimbal_motor_t *motor, gimbal_pid_t *angle_pid, float angle_get, float angle_set)
{
    float angle_torque;

    if (motor == 0 || angle_pid == 0)
    {
        return 0.0f;
    }

    motor->gyro_set = motor->ref_vel;
    angle_torque = gimbal_pid_calc(angle_pid, angle_get, angle_set, 0.0f);
    motor->pid_torque = angle_torque;

    return motor->pid_torque;
}

static float gimbal_calc_angle_speed_torque(gimbal_motor_t *motor, gimbal_pid_t *pid, float angle_error)
{
    float speed_error;
    float output;

    if (motor == 0 || pid == 0)
    {
        return 0.0f;
    }

    motor->gyro_set = motor->ref_vel;
    if (motor == &gimbal_control.gimbal_pitch_motor)
    {
        speed_error = motor->gyro_set - motor->relative_speed;
    }
    else
    {
        speed_error = motor->gyro_set - motor->gyro;
    }

    pid->set = angle_error;
    pid->fdb = 0.0f;
    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->error[0] = angle_error;
    pid->Pout = pid->Kp * angle_error;
    pid->Iout = 0.0f;
    pid->Dout = pid->Kd * speed_error;

    output = pid->Pout + pid->Dout;
    output = gimbal_clamp(output, -pid->max_out, pid->max_out);
    pid->out = output;
    motor->pid_torque = output;

    return motor->pid_torque;
}

static float gimbal_calc_yaw_angle_speed_torque(gimbal_motor_t *motor, float angle_error)
{
    if (motor == 0)
    {
        return 0.0f;
    }

    return gimbal_calc_angle_speed_torque(motor, &motor->absolute_angle_pid, angle_error);
}

void GimbalTask_Init(void)
{
    osThreadDef(gimbalTask, gimbal_task, osPriorityHigh, 0, 1024);
    gimbalTaskHandle = osThreadCreate(osThread(gimbalTask), NULL);
}

static void gimbal_task(void const *pvParameters)
{
    TickType_t last_wake_time;

    (void)pvParameters;

    vTaskDelay(GIMBAL_TASK_INIT_TIME);
    gimbal_init(&gimbal_control);
    shoot_task_init();
    last_wake_time = xTaskGetTickCount();

    while (1)
    {
        gimbal_set_mode(&gimbal_control);
        gimbal_feedback_update(&gimbal_control);
        gimbal_mode_change_control_transit(&gimbal_control);
        gimbal_set_control(&gimbal_control);
        gimbal_control_loop(&gimbal_control);
        gravity_comp_execute(&gimbal_control);
        gimbal_send_cmd(&gimbal_control);
        shoot_task_loop();

        /* VOFA 通道说明：
         * ch0：pitch 实际位置角，单位 rad
         * ch1：pitch 目标位置角，单位 rad
         * ch2：pitch 编码器差分速度，单位 rad/s
         * ch3：pitch PID 输出力矩，单位 N*m
         * ch4：pitch 前馈输出力矩，单位 N*m
         * ch5：pitch 最终输出力矩，单位 N*m
         */
//        VOFA_Send6(gimbal_control.gimbal_pitch_motor.relative_angle,
//                   gimbal_control.gimbal_pitch_motor.relative_angle_set,
//                   aim.err_deg.yaw,
//                   aim.err_deg.pitch,
//                   aim.err_rad_lpf.yaw,
//                   aim.err_rad_lpf.pitch);
        VOFA_Send6(gimbal_control.gimbal_yaw_motor.relative_angle,
                   gimbal_control.gimbal_yaw_motor.relative_angle_set,
                   gimbal_control.gimbal_pitch_motor.relative_angle,
                   gimbal_control.gimbal_pitch_motor.relative_angle_set,
									 aim.err_rad_lpf.yaw,
                   aim.err_rad_lpf.pitch);
//        VOFA_Send6(gimbal_control.gimbal_yaw_motor.relative_angle,
//                   gimbal_control.gimbal_yaw_motor.relative_angle_set,
//                   gimbal_control.gimbal_yaw_motor.gyro,
//                   gimbal_control.gimbal_yaw_motor.gyro_accel,
//                   gimbal_control.gimbal_yaw_motor.ff_torque,
//                   gimbal_control.gimbal_yaw_motor.given_current);
        vTaskDelayUntil(&last_wake_time, GIMBAL_CONTROL_TIME);
    }
}

const gimbal_motor_t *get_yaw_motor_point(void)
{
    return &gimbal_control.gimbal_yaw_motor;
}

const gimbal_motor_t *get_pitch_motor_point(void)
{
    return &gimbal_control.gimbal_pitch_motor;
}

void gimbal_absolute_angle_limit(gimbal_motor_t *motor, float add)
{
    const float bias = gimbal_take_auto_aim_bias(motor);

    if (motor == 0)
    {
        return;
    }

    if (motor == &gimbal_control.gimbal_yaw_motor)
    {
        motor->absolute_angle_set += add + bias;
    }
    else
    {
        motor->absolute_angle_set =
            gimbal_clamp(motor->absolute_angle_set + add + bias,
                         motor->min_relative_angle,
                         motor->max_relative_angle);
    }
}

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
        gimbal_calc_yaw_angle_speed_torque(motor,
                                           gimbal_wrap_angle(angle_set - angle_get));
    }
    else
    {
        angle_get = gimbal_wrap_angle(motor->absolute_angle);
        angle_set = gimbal_wrap_angle(motor->absolute_angle_set);
        gimbal_calc_feedback_torque(motor, &motor->absolute_angle_pid, angle_get, angle_set);
    }

    motor->current_set = motor->pid_torque + gimbal_calc_feedforward(motor);
    motor->output = motor->current_set;
    motor->given_current = gimbal_float_to_torque_cmd(motor->output);
}

void gimbal_motor_relative_angle_control(gimbal_motor_t *motor)
{
    if (motor == 0)
    {
        return;
    }

    if (motor == &gimbal_control.gimbal_yaw_motor)
    {
        gimbal_calc_feedback_torque(motor,
                                    &motor->relative_angle_pid,
                                    0.0f,
                                    gimbal_wrap_angle(motor->relative_angle_set - motor->relative_angle));
    }
    else
    {
        gimbal_calc_angle_speed_torque(motor,
                                       &motor->relative_angle_pid,
                                       motor->relative_angle_set - motor->relative_angle);
    }

    motor->current_set = motor->pid_torque + gimbal_calc_feedforward(motor);
    motor->output = motor->current_set;
    motor->given_current = gimbal_float_to_torque_cmd(motor->output);
}

void gimbal_motor_raw_angle_control(gimbal_motor_t *motor)
{
    if (motor == 0)
    {
        return;
    }

    motor->current_set = motor->raw_cmd;
    motor->output = motor->raw_cmd;
    motor->pid_torque = 0.0f;
    motor->ff_torque = 0.0f;
    motor->given_current = gimbal_float_to_torque_cmd(motor->output);
}

void gimbal_pid_init(gimbal_pid_t *pid, float kp, float ki, float kd, float max_out, float max_iout)
{
    float pid_param[3];

    if (pid == NULL)
    {
        return;
    }

    pid_param[0] = kp;
    pid_param[1] = ki;
    pid_param[2] = kd;
    PID_init(pid, PID_POSITION, pid_param, max_out, max_iout);
}

void gimbal_pid_clear(gimbal_pid_t *pid)
{
    if (pid == NULL)
    {
        return;
    }

    PID_clear(pid);
}

float gimbal_pid_calc(gimbal_pid_t *pid, float get, float set, float error_delta)
{
    (void)error_delta;

    if (pid == NULL)
    {
        return 0.0f;
    }

    return PID_Calc(pid, get, set);
}
