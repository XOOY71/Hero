/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       gimbal_behaviour.c
  * @brief      minimal gimbal behaviour framework
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "gimbal_behaviour.h"
#include <math.h>

volatile gimbal_behaviour_e gimbal_behaviour = GIMBAL_ZERO_FORCE;

#ifndef GIMBAL_MODE_CHANNEL
#define GIMBAL_MODE_CHANNEL 0
#endif

#ifndef YAW_CHANNEL
#define YAW_CHANNEL 0
#endif

#ifndef PITCH_CHANNEL
#define PITCH_CHANNEL 1
#endif

#ifndef RC_DEADBAND
#define RC_DEADBAND 10
#endif

#ifndef YAW_RC_SEN
#define YAW_RC_SEN 0.0005f
#endif

#ifndef PITCH_RC_SEN
#define PITCH_RC_SEN 0.0005f
#endif

#ifndef YAW_MOUSE_SEN
#define YAW_MOUSE_SEN 0.0010f
#endif

#ifndef PITCH_MOUSE_SEN
#define PITCH_MOUSE_SEN 0.0010f
#endif

#ifndef GIMBAL_SPIN_SPEED
#define GIMBAL_SPIN_SPEED 0.03f
#endif

#ifndef INIT_YAW_SET
#define INIT_YAW_SET 0.0f
#endif

#ifndef INIT_PITCH_SET
#define INIT_PITCH_SET 0.0f
#endif

#ifndef GIMBAL_INIT_YAW_SPEED
#define GIMBAL_INIT_YAW_SPEED 0.15f
#endif

#ifndef GIMBAL_INIT_PITCH_SPEED
#define GIMBAL_INIT_PITCH_SPEED 0.15f
#endif

#ifndef GIMBAL_INIT_ANGLE_ERROR
#define GIMBAL_INIT_ANGLE_ERROR 0.03f
#endif

#ifndef GIMBAL_INIT_TIME
#define GIMBAL_INIT_TIME 2000U
#endif

#ifndef GIMBAL_INIT_STOP_TIME
#define GIMBAL_INIT_STOP_TIME 200U
#endif

#ifndef GIMBAL_CALI_START_STEP
#define GIMBAL_CALI_START_STEP 1U
#endif

#ifndef GIMBAL_CALI_PITCH_MAX_STEP
#define GIMBAL_CALI_PITCH_MAX_STEP 1U
#endif

#ifndef GIMBAL_CALI_PITCH_MIN_STEP
#define GIMBAL_CALI_PITCH_MIN_STEP 2U
#endif

#ifndef GIMBAL_CALI_YAW_MAX_STEP
#define GIMBAL_CALI_YAW_MAX_STEP 3U
#endif

#ifndef GIMBAL_CALI_YAW_MIN_STEP
#define GIMBAL_CALI_YAW_MIN_STEP 4U
#endif

#ifndef GIMBAL_CALI_END_STEP
#define GIMBAL_CALI_END_STEP 5U
#endif

#ifndef GIMBAL_CALI_MOTOR_SET
#define GIMBAL_CALI_MOTOR_SET 100.0f
#endif

static int16_t gimbal_apply_deadband(int16_t value)
{
    if (value > RC_DEADBAND || value < -RC_DEADBAND)
    {
        return value;
    }
    return 0;
}

static void gimbal_read_manual_input(float *yaw, float *pitch, gimbal_control_t *control)
{
    int16_t yaw_channel = 0;
    int16_t pitch_channel = 0;
    int mouse_x = 0;
    int mouse_y = 0;

    if (yaw == 0 || pitch == 0 || control == 0)
    {
        return;
    }

    *yaw = 0.0f;
    *pitch = 0.0f;

    if (control->gimbal_rc_ctrl == 0)
    {
        return;
    }

    yaw_channel = gimbal_apply_deadband(control->gimbal_rc_ctrl->rc.ch[YAW_CHANNEL]);
    pitch_channel = gimbal_apply_deadband(control->gimbal_rc_ctrl->rc.ch[PITCH_CHANNEL]);
    mouse_x = control->gimbal_rc_ctrl->mouse.x;
    mouse_y = control->gimbal_rc_ctrl->mouse.y;

    *yaw = yaw_channel * YAW_RC_SEN - (float)mouse_x * YAW_MOUSE_SEN;
    *pitch = pitch_channel * PITCH_RC_SEN + (float)mouse_y * PITCH_MOUSE_SEN;
}

/**
  * @brief          云台行为状态机以及电机状态机设置
  * @param[out]     control: 云台数据指针
  * @retval         none
  */
void gimbal_behaviour_mode_set(gimbal_control_t *control)
{
    if (control == 0)
    {
        return;
    }

    gimbal_behavour_set(control);

    switch (gimbal_behaviour)
    {
    case GIMBAL_ZERO_FORCE:
        control->gimbal_yaw_motor.mode = GIMBAL_MOTOR_RAW;
        control->gimbal_pitch_motor.mode = GIMBAL_MOTOR_RAW;
        break;

    case GIMBAL_INIT:
        control->gimbal_yaw_motor.mode = GIMBAL_MOTOR_ENCODE;
        control->gimbal_pitch_motor.mode = GIMBAL_MOTOR_ENCODE;
        break;

    case GIMBAL_CALI:
        control->gimbal_yaw_motor.mode = GIMBAL_MOTOR_RAW;
        control->gimbal_pitch_motor.mode = GIMBAL_MOTOR_RAW;
        break;

    case GIMBAL_ABSOLUTE_ANGLE:
        control->gimbal_yaw_motor.mode = GIMBAL_MOTOR_GYRO;
        control->gimbal_pitch_motor.mode = GIMBAL_MOTOR_GYRO;
        break;

    case GIMBAL_RELATIVE_ANGLE:
        control->gimbal_yaw_motor.mode = GIMBAL_MOTOR_ENCODE;
        control->gimbal_pitch_motor.mode = GIMBAL_MOTOR_ENCODE;
        break;

    case GIMBAL_SPIN:
        control->gimbal_yaw_motor.mode = GIMBAL_MOTOR_GYRO;
        control->gimbal_pitch_motor.mode = GIMBAL_MOTOR_ENCODE;
        break;

    case GIMBAL_MOTIONLESS:
    default:
        control->gimbal_yaw_motor.mode = GIMBAL_MOTOR_RAW;
        control->gimbal_pitch_motor.mode = GIMBAL_MOTOR_RAW;
        break;
    }
}

/**
  * @brief          云台行为控制，根据不同行为采用不同控制函数
  * @param[out]     add_yaw: yaw角度增加值
  * @param[out]     add_pitch: pitch角度增加值
  * @param[in]      control: 云台数据指针
  * @retval         none
  */
void gimbal_behaviour_control_set(float *add_yaw, float *add_pitch, gimbal_control_t *control)
{
    if (add_yaw == 0 || add_pitch == 0 || control == 0)
    {
        return;
    }

    switch (gimbal_behaviour)
    {
    case GIMBAL_ZERO_FORCE:
        gimbal_zero_force_control(add_yaw, add_pitch, control);
        break;
    case GIMBAL_INIT:
        gimbal_init_control(add_yaw, add_pitch, control);
        break;
    case GIMBAL_CALI:
        gimbal_cali_control(add_yaw, add_pitch, control);
        break;
    case GIMBAL_ABSOLUTE_ANGLE:
        gimbal_absolute_angle_control(add_yaw, add_pitch, control);
        break;
    case GIMBAL_RELATIVE_ANGLE:
        gimbal_relative_angle_control(add_yaw, add_pitch, control);
        break;
    case GIMBAL_MOTIONLESS:
        gimbal_motionless_control(add_yaw, add_pitch, control);
        break;
    case GIMBAL_SPIN:
        gimbal_spin_control(add_yaw, add_pitch, control);
        break;
    default:
        *add_yaw = 0.0f;
        *add_pitch = 0.0f;
        break;
    }
}

/**
  * @brief          云台在某些行为下，需要底盘不动
  * @param[in]      none
  * @retval         true:no move false:normal
  */
bool gimbal_cmd_to_chassis_stop(void)
{
    return (gimbal_behaviour == GIMBAL_INIT ||
            gimbal_behaviour == GIMBAL_CALI ||
            gimbal_behaviour == GIMBAL_MOTIONLESS ||
            gimbal_behaviour == GIMBAL_ZERO_FORCE);
}

/**
  * @brief          云台在某些行为下，需要射击停止
  * @param[in]      none
  * @retval         true:no move false:normal
  */
bool gimbal_cmd_to_shoot_stop(void)
{
    return (gimbal_behaviour == GIMBAL_INIT ||
            gimbal_behaviour == GIMBAL_CALI ||
            gimbal_behaviour == GIMBAL_ZERO_FORCE ||
            gimbal_behaviour == GIMBAL_MOTIONLESS);
}

/**
  * @brief          云台行为状态机设置
  * @param[in]      control: 云台数据指针
  * @retval         none
  */
void gimbal_behavour_set(gimbal_control_t *control)
{
    static gimbal_behaviour_e last_gimbal_behaviour = GIMBAL_ZERO_FORCE;
    static unsigned int init_time = 0U;
    static unsigned int init_stop_time = 0U;
    int mode_switch = 0;

    if (control == 0)
    {
        return;
    }

    if (control->gimbal_rc_ctrl == 0)
    {
        gimbal_behaviour = GIMBAL_ZERO_FORCE;
        last_gimbal_behaviour = gimbal_behaviour;
        return;
    }

    if (gimbal_behaviour == GIMBAL_CALI &&
        control->gimbal_cali.step != 0U &&
        control->gimbal_cali.step != GIMBAL_CALI_END_STEP)
    {
        return;
    }

    if (control->gimbal_cali.step == GIMBAL_CALI_START_STEP)
    {
        gimbal_behaviour = GIMBAL_CALI;
        last_gimbal_behaviour = gimbal_behaviour;
        return;
    }

    if (gimbal_behaviour == GIMBAL_INIT)
    {
        if (fabsf(control->gimbal_yaw_motor.relative_angle - INIT_YAW_SET) < GIMBAL_INIT_ANGLE_ERROR &&
            fabsf(control->gimbal_pitch_motor.relative_angle - INIT_PITCH_SET) < GIMBAL_INIT_ANGLE_ERROR)
        {
            if (init_stop_time < GIMBAL_INIT_STOP_TIME)
            {
                init_stop_time++;
            }
        }
        else
        {
            if (init_time < GIMBAL_INIT_TIME)
            {
                init_time++;
            }
        }

        if (init_time < GIMBAL_INIT_TIME && init_stop_time < GIMBAL_INIT_STOP_TIME)
        {
            last_gimbal_behaviour = gimbal_behaviour;
            return;
        }

        init_time = 0U;
        init_stop_time = 0U;
    }

    mode_switch = control->gimbal_rc_ctrl->rc.s[GIMBAL_MODE_CHANNEL];

    if (switch_is_mid(mode_switch))
    {
        gimbal_behaviour = GIMBAL_MOTIONLESS;
    }
    else if (switch_is_up(mode_switch))
    {
        gimbal_behaviour = GIMBAL_SPIN;
    }
    else if (switch_is_down(mode_switch))
    {
        gimbal_behaviour = GIMBAL_RELATIVE_ANGLE;
    }
    else
    {
        gimbal_behaviour = GIMBAL_ZERO_FORCE;
    }

    if (switch_is_down(mode_switch) && (control->gimbal_rc_ctrl->key.v & GIMBAL_ZERO_KEYBOARD))
    {
        gimbal_behaviour = GIMBAL_MOTIONLESS;
    }
    else if (switch_is_down(mode_switch) && (control->gimbal_rc_ctrl->key.v & GIMBAL_SPIN_KEYBOARD))
    {
        gimbal_behaviour = GIMBAL_SPIN;
    }
    else if (switch_is_down(mode_switch) && (control->gimbal_rc_ctrl->key.v & GIMBAL_RELATIVE_KEYBOARD))
    {
        gimbal_behaviour = GIMBAL_RELATIVE_ANGLE;
    }

    if (last_gimbal_behaviour == GIMBAL_ZERO_FORCE &&
        gimbal_behaviour != GIMBAL_ZERO_FORCE)
    {
        gimbal_behaviour = GIMBAL_INIT;
    }

    last_gimbal_behaviour = gimbal_behaviour;
}

/**
  * @brief          当云台行为模式是GIMBAL_ZERO_FORCE时，控制量清零
  * @param[in]      yaw,pitch: 输出控制量
  * @param[in]      control: 云台数据指针
  * @retval         none
  */
void gimbal_zero_force_control(float *yaw, float *pitch, gimbal_control_t *control)
{
    (void)control;

    if (yaw == 0 || pitch == 0)
    {
        return;
    }

    *yaw = 0.0f;
    *pitch = 0.0f;
}

/**
  * @brief          云台初始化控制
  * @param[out]     yaw,pitch: 输出控制量
  * @param[in]      control: 云台数据指针
  * @retval         none
  */
void gimbal_init_control(float *yaw, float *pitch, gimbal_control_t *control)
{
    if (yaw == 0 || pitch == 0 || control == 0)
    {
        return;
    }

    *yaw = (INIT_YAW_SET - control->gimbal_yaw_motor.relative_angle) * GIMBAL_INIT_YAW_SPEED;
    *pitch = (INIT_PITCH_SET - control->gimbal_pitch_motor.relative_angle) * GIMBAL_INIT_PITCH_SPEED;
}

/**
  * @brief          云台校准控制
  * @param[out]     yaw,pitch: 输出控制量
  * @param[in]      control: 云台数据指针
  * @retval         none
  */
void gimbal_cali_control(float *yaw, float *pitch, gimbal_control_t *control)
{
    if (yaw == 0 || pitch == 0 || control == 0)
    {
        return;
    }

    switch (control->gimbal_cali.step)
    {
    case GIMBAL_CALI_PITCH_MAX_STEP:
        *yaw = 0.0f;
        *pitch = GIMBAL_CALI_MOTOR_SET;
        break;
    case GIMBAL_CALI_PITCH_MIN_STEP:
        *yaw = 0.0f;
        *pitch = -GIMBAL_CALI_MOTOR_SET;
        break;
    case GIMBAL_CALI_YAW_MAX_STEP:
        *yaw = GIMBAL_CALI_MOTOR_SET;
        *pitch = 0.0f;
        break;
    case GIMBAL_CALI_YAW_MIN_STEP:
        *yaw = -GIMBAL_CALI_MOTOR_SET;
        *pitch = 0.0f;
        break;
    default:
        *yaw = 0.0f;
        *pitch = 0.0f;
        break;
    }
}

/**
  * @brief          云台绝对角控制
  * @param[out]     yaw,pitch: 输出控制量
  * @param[in]      control: 云台数据指针
  * @retval         none
  */
void gimbal_absolute_angle_control(float *yaw, float *pitch, gimbal_control_t *control)
{
    if (yaw == 0 || pitch == 0 || control == 0)
    {
        return;
    }

    gimbal_read_manual_input(yaw, pitch, control);
}

/**
  * @brief          云台相对角控制
  * @param[out]     yaw,pitch: 输出控制量
  * @param[in]      control: 云台数据指针
  * @retval         none
  */
void gimbal_relative_angle_control(float *yaw, float *pitch, gimbal_control_t *control)
{
    if (yaw == 0 || pitch == 0 || control == 0)
    {
        return;
    }

    gimbal_read_manual_input(yaw, pitch, control);
}

/**
  * @brief          云台静止控制
  * @param[out]     yaw,pitch: 输出控制量
  * @param[in]      control: 云台数据指针
  * @retval         none
  */
void gimbal_motionless_control(float *yaw, float *pitch, gimbal_control_t *control)
{
    (void)control;

    if (yaw == 0 || pitch == 0)
    {
        return;
    }

    *yaw = 0.0f;
    *pitch = 0.0f;
}

/**
  * @brief          云台小陀螺控制
  * @param[out]     yaw,pitch: 输出控制量
  * @param[in]      control: 云台数据指针
  * @retval         none
  */
void gimbal_spin_control(float *yaw, float *pitch, gimbal_control_t *control)
{
    float manual_yaw = 0.0f;
    float manual_pitch = 0.0f;

    if (yaw == 0 || pitch == 0 || control == 0)
    {
        return;
    }

    gimbal_read_manual_input(&manual_yaw, &manual_pitch, control);
    *yaw = GIMBAL_SPIN_SPEED + manual_yaw;
    *pitch = manual_pitch;
}
