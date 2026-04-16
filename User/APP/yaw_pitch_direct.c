/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       yaw_pitch_direct.c
  * @brief      minimal yaw-pitch direct framework
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "yaw_pitch_direct.h"
#include <string.h>

#ifndef INS_YAW_ADDRESS_OFFSET
#define INS_YAW_ADDRESS_OFFSET 0
#endif

#ifndef INS_PITCH_ADDRESS_OFFSET
#define INS_PITCH_ADDRESS_OFFSET 1
#endif

#ifndef INS_GYRO_X_ADDRESS_OFFSET
#define INS_GYRO_X_ADDRESS_OFFSET 0
#endif

#ifndef INS_GYRO_Y_ADDRESS_OFFSET
#define INS_GYRO_Y_ADDRESS_OFFSET 1
#endif

#ifndef INS_GYRO_Z_ADDRESS_OFFSET
#define INS_GYRO_Z_ADDRESS_OFFSET 2
#endif

#ifndef YAW_ABSOLUTE_PID_KP
#define YAW_ABSOLUTE_PID_KP 12.0f
#endif

#ifndef YAW_ABSOLUTE_PID_KI
#define YAW_ABSOLUTE_PID_KI 0.0f
#endif

#ifndef YAW_ABSOLUTE_PID_KD
#define YAW_ABSOLUTE_PID_KD 0.3f
#endif

#ifndef YAW_RELATIVE_PID_KP
#define YAW_RELATIVE_PID_KP 12.0f
#endif

#ifndef YAW_RELATIVE_PID_KI
#define YAW_RELATIVE_PID_KI 0.0f
#endif

#ifndef YAW_RELATIVE_PID_KD
#define YAW_RELATIVE_PID_KD 0.3f
#endif

#ifndef YAW_GYRO_PID_KP
#define YAW_GYRO_PID_KP 6.0f
#endif

#ifndef YAW_GYRO_PID_KI
#define YAW_GYRO_PID_KI 0.0f
#endif

#ifndef YAW_GYRO_PID_KD
#define YAW_GYRO_PID_KD 0.0f
#endif

#ifndef PITCH_ABSOLUTE_PID_KP
#define PITCH_ABSOLUTE_PID_KP 10.0f
#endif

#ifndef PITCH_ABSOLUTE_PID_KI
#define PITCH_ABSOLUTE_PID_KI 0.0f
#endif

#ifndef PITCH_ABSOLUTE_PID_KD
#define PITCH_ABSOLUTE_PID_KD 0.2f
#endif

#ifndef PITCH_RELATIVE_PID_KP
#define PITCH_RELATIVE_PID_KP 10.0f
#endif

#ifndef PITCH_RELATIVE_PID_KI
#define PITCH_RELATIVE_PID_KI 0.0f
#endif

#ifndef PITCH_RELATIVE_PID_KD
#define PITCH_RELATIVE_PID_KD 0.2f
#endif

#ifndef PITCH_GYRO_PID_KP
#define PITCH_GYRO_PID_KP 5.0f
#endif

#ifndef PITCH_GYRO_PID_KI
#define PITCH_GYRO_PID_KI 0.0f
#endif

#ifndef PITCH_GYRO_PID_KD
#define PITCH_GYRO_PID_KD 0.0f
#endif

#ifndef YAW_MAX_RELATIVE_ANGLE
#define YAW_MAX_RELATIVE_ANGLE 3.1415926f
#endif

#ifndef YAW_MIN_RELATIVE_ANGLE
#define YAW_MIN_RELATIVE_ANGLE -3.1415926f
#endif

#ifndef PITCH_MAX_RELATIVE_ANGLE
#define PITCH_MAX_RELATIVE_ANGLE 0.8f
#endif

#ifndef PITCH_MIN_RELATIVE_ANGLE
#define PITCH_MIN_RELATIVE_ANGLE -0.8f
#endif

int16_t yaw_can_set_current = 0;
int16_t pitch_can_set_current = 0;
int16_t shoot_can_set_current = 0;

__attribute__((weak)) const float *get_INS_angle_point(void)
{
    return 0;
}

__attribute__((weak)) const float *get_gyro_data_point(void)
{
    return 0;
}

__attribute__((weak)) void gimbal_platform_send_current(int16_t yaw_current, int16_t pitch_current, int16_t trigger_current)
{
    (void)yaw_current;
    (void)pitch_current;
    (void)trigger_current;
}

static void gimbal_total_pid_clear(gimbal_control_t *control)
{
    if (control == 0)
    {
        return;
    }

    gimbal_pid_clear(&control->gimbal_yaw_motor.absolute_angle_pid);
    gimbal_pid_clear(&control->gimbal_yaw_motor.relative_angle_pid);
    gimbal_pid_clear(&control->gimbal_yaw_motor.gyro_pid);
    gimbal_pid_clear(&control->gimbal_pitch_motor.absolute_angle_pid);
    gimbal_pid_clear(&control->gimbal_pitch_motor.relative_angle_pid);
    gimbal_pid_clear(&control->gimbal_pitch_motor.gyro_pid);
}

/**
  * @brief          初始化gimbal_control变量
  * @param[out]     control: 云台控制结构体指针
  * @retval         none
  */
void gimbal_init(gimbal_control_t *control)
{
    if (control == 0)
    {
        return;
    }

    memset(control, 0, sizeof(*control));

    control->gimbal_rc_ctrl = get_remote_control_point();
    control->gimbal_INT_angle_point = get_INS_angle_point();
    control->gimbal_INT_gyro_point = get_gyro_data_point();

    control->gimbal_yaw_motor.mode = GIMBAL_MOTOR_RAW;
    control->gimbal_yaw_motor.last_mode = GIMBAL_MOTOR_RAW;
    control->gimbal_pitch_motor.mode = GIMBAL_MOTOR_RAW;
    control->gimbal_pitch_motor.last_mode = GIMBAL_MOTOR_RAW;

    gimbal_pid_init(&control->gimbal_yaw_motor.absolute_angle_pid, YAW_ABSOLUTE_PID_KP, YAW_ABSOLUTE_PID_KI, YAW_ABSOLUTE_PID_KD);
    gimbal_pid_init(&control->gimbal_yaw_motor.relative_angle_pid, YAW_RELATIVE_PID_KP, YAW_RELATIVE_PID_KI, YAW_RELATIVE_PID_KD);
    gimbal_pid_init(&control->gimbal_yaw_motor.gyro_pid, YAW_GYRO_PID_KP, YAW_GYRO_PID_KI, YAW_GYRO_PID_KD);

    gimbal_pid_init(&control->gimbal_pitch_motor.absolute_angle_pid, PITCH_ABSOLUTE_PID_KP, PITCH_ABSOLUTE_PID_KI, PITCH_ABSOLUTE_PID_KD);
    gimbal_pid_init(&control->gimbal_pitch_motor.relative_angle_pid, PITCH_RELATIVE_PID_KP, PITCH_RELATIVE_PID_KI, PITCH_RELATIVE_PID_KD);
    gimbal_pid_init(&control->gimbal_pitch_motor.gyro_pid, PITCH_GYRO_PID_KP, PITCH_GYRO_PID_KI, PITCH_GYRO_PID_KD);

    control->gimbal_yaw_motor.max_relative_angle = YAW_MAX_RELATIVE_ANGLE;
    control->gimbal_yaw_motor.min_relative_angle = YAW_MIN_RELATIVE_ANGLE;
    control->gimbal_pitch_motor.max_relative_angle = PITCH_MAX_RELATIVE_ANGLE;
    control->gimbal_pitch_motor.min_relative_angle = PITCH_MIN_RELATIVE_ANGLE;

    gimbal_total_pid_clear(control);
    gimbal_feedback_update(control);

    control->gimbal_yaw_motor.absolute_angle_set = control->gimbal_yaw_motor.absolute_angle;
    control->gimbal_yaw_motor.relative_angle_set = control->gimbal_yaw_motor.relative_angle;
    control->gimbal_yaw_motor.gyro_set = control->gimbal_yaw_motor.gyro;

    control->gimbal_pitch_motor.absolute_angle_set = control->gimbal_pitch_motor.absolute_angle;
    control->gimbal_pitch_motor.relative_angle_set = control->gimbal_pitch_motor.relative_angle;
    control->gimbal_pitch_motor.gyro_set = control->gimbal_pitch_motor.gyro;
}

/**
  * @brief          设置云台控制模式
  * @param[out]     control: 云台控制结构体指针
  * @retval         none
  */
void gimbal_set_mode(gimbal_control_t *control)
{
    if (control == 0)
    {
        return;
    }

    gimbal_behaviour_mode_set(control);
}

/**
  * @brief          云台反馈更新
  * @param[out]     control: 云台控制结构体指针
  * @retval         none
  */
void gimbal_feedback_update(gimbal_control_t *control)
{
    if (control == 0)
    {
        return;
    }

    if (control->gimbal_INT_angle_point != 0)
    {
        control->gimbal_yaw_motor.absolute_angle = control->gimbal_INT_angle_point[INS_YAW_ADDRESS_OFFSET];
        control->gimbal_pitch_motor.absolute_angle = control->gimbal_INT_angle_point[INS_PITCH_ADDRESS_OFFSET];
    }

    if (control->gimbal_INT_gyro_point != 0)
    {
        control->gimbal_yaw_motor.gyro = control->gimbal_INT_gyro_point[INS_GYRO_Z_ADDRESS_OFFSET];
        control->gimbal_pitch_motor.gyro = control->gimbal_INT_gyro_point[INS_GYRO_Y_ADDRESS_OFFSET];
    }

    control->gimbal_yaw_motor.relative_angle = control->gimbal_yaw_motor.absolute_angle;
    control->gimbal_pitch_motor.relative_angle = control->gimbal_pitch_motor.absolute_angle;
}

/**
  * @brief          控制模式切换时的过渡处理
  * @param[out]     control: 云台控制结构体指针
  * @retval         none
  */
void gimbal_mode_change_control_transit(gimbal_control_t *control)
{
    if (control == 0)
    {
        return;
    }

    if (control->gimbal_yaw_motor.last_mode != GIMBAL_MOTOR_RAW && control->gimbal_yaw_motor.mode == GIMBAL_MOTOR_RAW)
    {
        control->gimbal_yaw_motor.raw_cmd = control->gimbal_yaw_motor.current_set = control->gimbal_yaw_motor.given_current;
    }
    else if (control->gimbal_yaw_motor.last_mode != GIMBAL_MOTOR_GYRO && control->gimbal_yaw_motor.mode == GIMBAL_MOTOR_GYRO)
    {
        control->gimbal_yaw_motor.absolute_angle_set = control->gimbal_yaw_motor.absolute_angle;
        control->gimbal_yaw_motor.gyro_set = control->gimbal_yaw_motor.gyro;
    }
    else if (control->gimbal_yaw_motor.last_mode != GIMBAL_MOTOR_ENCODE && control->gimbal_yaw_motor.mode == GIMBAL_MOTOR_ENCODE)
    {
        control->gimbal_yaw_motor.relative_angle_set = control->gimbal_yaw_motor.relative_angle;
        control->gimbal_yaw_motor.gyro_set = control->gimbal_yaw_motor.gyro;
    }
    control->gimbal_yaw_motor.last_mode = control->gimbal_yaw_motor.mode;

    if (control->gimbal_pitch_motor.last_mode != GIMBAL_MOTOR_RAW && control->gimbal_pitch_motor.mode == GIMBAL_MOTOR_RAW)
    {
        control->gimbal_pitch_motor.raw_cmd = control->gimbal_pitch_motor.current_set = control->gimbal_pitch_motor.given_current;
    }
    else if (control->gimbal_pitch_motor.last_mode != GIMBAL_MOTOR_GYRO && control->gimbal_pitch_motor.mode == GIMBAL_MOTOR_GYRO)
    {
        control->gimbal_pitch_motor.absolute_angle_set = control->gimbal_pitch_motor.absolute_angle;
        control->gimbal_pitch_motor.gyro_set = control->gimbal_pitch_motor.gyro;
    }
    else if (control->gimbal_pitch_motor.last_mode != GIMBAL_MOTOR_ENCODE && control->gimbal_pitch_motor.mode == GIMBAL_MOTOR_ENCODE)
    {
        control->gimbal_pitch_motor.relative_angle_set = control->gimbal_pitch_motor.relative_angle;
        control->gimbal_pitch_motor.gyro_set = control->gimbal_pitch_motor.gyro;
    }

    control->gimbal_pitch_motor.last_mode = control->gimbal_pitch_motor.mode;
}

/**
  * @brief          设置云台控制设定值
  * @param[out]     control: 云台控制结构体指针
  * @retval         none
  */
void gimbal_set_control(gimbal_control_t *control)
{
    static float add_yaw = 0.0f;
    static float add_pitch = 0.0f;

    if (control == 0)
    {
        return;
    }

    gimbal_behaviour_control_set(&add_yaw, &add_pitch, control);

    if (control->gimbal_yaw_motor.mode == GIMBAL_MOTOR_RAW)
    {
        control->gimbal_yaw_motor.raw_cmd = add_yaw;
    }
    else if (control->gimbal_yaw_motor.mode == GIMBAL_MOTOR_GYRO)
    {
        gimbal_absolute_angle_limit(&control->gimbal_yaw_motor, add_yaw);
    }
    else if (control->gimbal_yaw_motor.mode == GIMBAL_MOTOR_ENCODE)
    {
        gimbal_relative_angle_limit(&control->gimbal_yaw_motor, add_yaw);
    }

    if (control->gimbal_pitch_motor.mode == GIMBAL_MOTOR_RAW)
    {
        control->gimbal_pitch_motor.raw_cmd = add_pitch;
    }
    else if (control->gimbal_pitch_motor.mode == GIMBAL_MOTOR_GYRO)
    {
        gimbal_absolute_angle_limit(&control->gimbal_pitch_motor, add_pitch);
    }
    else if (control->gimbal_pitch_motor.mode == GIMBAL_MOTOR_ENCODE)
    {
        gimbal_relative_angle_limit(&control->gimbal_pitch_motor, add_pitch);
    }
}

/**
  * @brief          云台控制环
  * @param[out]     control: 云台控制结构体指针
  * @retval         none
  */
void gimbal_control_loop(gimbal_control_t *control)
{
    if (control == 0)
    {
        return;
    }

    if (control->gimbal_yaw_motor.mode == GIMBAL_MOTOR_RAW)
    {
        gimbal_motor_raw_angle_control(&control->gimbal_yaw_motor);
    }
    else if (control->gimbal_yaw_motor.mode == GIMBAL_MOTOR_GYRO)
    {
        gimbal_motor_absolute_angle_control(&control->gimbal_yaw_motor);
    }
    else if (control->gimbal_yaw_motor.mode == GIMBAL_MOTOR_ENCODE)
    {
        gimbal_motor_relative_angle_control(&control->gimbal_yaw_motor);
    }

    if (control->gimbal_pitch_motor.mode == GIMBAL_MOTOR_RAW)
    {
        gimbal_motor_raw_angle_control(&control->gimbal_pitch_motor);
    }
    else if (control->gimbal_pitch_motor.mode == GIMBAL_MOTOR_GYRO)
    {
        gimbal_motor_absolute_angle_control(&control->gimbal_pitch_motor);
    }
    else if (control->gimbal_pitch_motor.mode == GIMBAL_MOTOR_ENCODE)
    {
        gimbal_motor_relative_angle_control(&control->gimbal_pitch_motor);
    }
}

/**
  * @brief          发送控制命令
  * @param[out]     control: 云台控制结构体指针
  * @retval         none
  */
void gimbal_send_cmd(gimbal_control_t *control)
{
    if (control == 0)
    {
        return;
    }

    yaw_can_set_current = control->gimbal_yaw_motor.given_current;
    pitch_can_set_current = control->gimbal_pitch_motor.given_current;
    gimbal_platform_send_current(yaw_can_set_current, pitch_can_set_current, shoot_can_set_current);
}
