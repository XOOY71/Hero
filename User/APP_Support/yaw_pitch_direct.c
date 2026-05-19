/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       yaw_pitch_direct.c
  * @brief      minimal yaw-pitch direct framework
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "yaw_pitch_direct.h"
#include "auto_aim.h"
#include "hwt_imu.h"
#include "bsp_fdcan.h"
#include "project_config.h"
#include "cmsis_os.h"
#include <math.h>
#include <string.h>

#define YAW_PITCH_DIRECT_PI PI

#ifndef GIMBAL_MIT_FEEDBACK_INIT_DELAY
#define GIMBAL_MIT_FEEDBACK_INIT_DELAY 100U
#endif

#ifndef GIMBAL_AUTO_AIM_YAW_KP
#define GIMBAL_AUTO_AIM_YAW_KP 14.0f
#endif
#ifndef GIMBAL_AUTO_AIM_PITCH_KP
#define GIMBAL_AUTO_AIM_PITCH_KP 9.0f
#endif
#ifndef GIMBAL_AUTO_AIM_YAW_MAX_SPEED
#define GIMBAL_AUTO_AIM_YAW_MAX_SPEED (720.0f * YAW_PITCH_DIRECT_PI / 180.0f)
#endif
#ifndef GIMBAL_AUTO_AIM_PITCH_MAX_SPEED
#define GIMBAL_AUTO_AIM_PITCH_MAX_SPEED (720.0f * YAW_PITCH_DIRECT_PI / 180.0f)
#endif
#ifndef GIMBAL_AUTO_AIM_YAW_MAX_ACCEL
#define GIMBAL_AUTO_AIM_YAW_MAX_ACCEL (6000.0f * YAW_PITCH_DIRECT_PI / 180.0f)
#endif
#ifndef GIMBAL_AUTO_AIM_PITCH_MAX_ACCEL
#define GIMBAL_AUTO_AIM_PITCH_MAX_ACCEL (5100.0f * YAW_PITCH_DIRECT_PI / 180.0f)
#endif

float yaw_can_set_current = 0.0f;
float pitch_can_set_current = 0.0f;
int16_t shoot_can_set_current = 0;

static float yaw_ref_target_last = 0.0f;
static uint8_t yaw_ref_target_init = 0u;
static float pitch_ref_target_last = 0.0f;
static uint8_t pitch_ref_target_init = 0u;
static float yaw_auto_aim_target_vel = 0.0f;
static float pitch_auto_aim_target_vel = 0.0f;

__attribute__((weak)) const float *get_INS_angle_point(void)
{
    return 0;
}

__attribute__((weak)) const float *get_gyro_data_point(void)
{
    return 0;
}

__attribute__((weak)) const float *get_accel_data_point(void)
{
    return 0;
}

static void gimbal_auto_aim_clear_target_vel(gimbal_motor_t *motor);

static float gimbal_mit_clamp(float value, float min_value, float max_value)
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

static float gimbal_output_to_mit_torque(float output)
{
    return gimbal_mit_clamp(output, T_MIN, T_MAX);
}

static void gimbal_feedforward_clear(gimbal_motor_t *motor)
{
    if (motor == 0)
    {
        return;
    }

    motor->ref_vel = 0.0f;
    motor->ref_vel_last = 0.0f;
    motor->ref_accel = 0.0f;
    motor->ff_torque = 0.0f;
    gimbal_auto_aim_clear_target_vel(motor);

    if (motor == &gimbal_control.gimbal_yaw_motor)
    {
        yaw_ref_target_last = 0.0f;
        yaw_ref_target_init = 0u;
    }
    else if (motor == &gimbal_control.gimbal_pitch_motor)
    {
        pitch_ref_target_last = 0.0f;
        pitch_ref_target_init = 0u;
    }
}

static void gimbal_feedforward_track_target(gimbal_motor_t *motor, float target_angle)
{
    float ref_vel_cmd;
    float ref_accel_cmd;
    float alpha;
    float accel_limit;
    float *target_last;
    uint8_t *target_init;
    const float control_dt = (float)GIMBAL_CONTROL_TIME * 0.001f;

    if (motor == 0 || control_dt <= 0.0f)
    {
        return;
    }

    if (motor == &gimbal_control.gimbal_yaw_motor)
    {
        target_last = &yaw_ref_target_last;
        target_init = &yaw_ref_target_init;
    }
    else if (motor == &gimbal_control.gimbal_pitch_motor)
    {
        target_last = &pitch_ref_target_last;
        target_init = &pitch_ref_target_init;
    }
    else
    {
        return;
    }

    if (*target_init == 0u)
    {
        *target_last = target_angle;
        *target_init = 1u;
        motor->ref_vel_last = 0.0f;
        motor->ref_vel = 0.0f;
        motor->ref_accel = 0.0f;
        return;
    }

    alpha = gimbal_mit_clamp(YAW_REF_VEL_FILTER_ALPHA, 0.0f, 1.0f);
    ref_vel_cmd = (target_angle - *target_last) / control_dt;

    motor->ref_vel_last = motor->ref_vel;
    motor->ref_vel += alpha * (ref_vel_cmd - motor->ref_vel);

    ref_accel_cmd = (motor->ref_vel - motor->ref_vel_last) / control_dt;
    accel_limit = YAW_REF_ACCEL_LIMIT;
    if (accel_limit > 0.0f)
    {
        ref_accel_cmd = gimbal_mit_clamp(ref_accel_cmd,
                                         -accel_limit,
                                         accel_limit);
    }

    motor->ref_accel = ref_accel_cmd;
    *target_last = target_angle;
}

static float yaw_pitch_direct_wrap_angle(float angle)
{
    while (angle > YAW_PITCH_DIRECT_PI)
    {
        angle -= 2.0f * YAW_PITCH_DIRECT_PI;
    }
    while (angle < -YAW_PITCH_DIRECT_PI)
    {
        angle += 2.0f * YAW_PITCH_DIRECT_PI;
    }
    return angle;
}

static float gimbal_auto_aim_plan_target(float target_set,
                                         float desired_target,
                                         float *target_vel,
                                         float kp,
                                         float max_speed,
                                         float max_accel,
                                         float dt)
{
    float err;
    float vel_cmd;
    float vel_delta;
    float vel_delta_max;
    float step;

    if (target_vel == 0 || dt <= 0.0f)
    {
        return target_set;
    }

    err = desired_target - target_set;
    vel_cmd = gimbal_mit_clamp(kp * err, -max_speed, max_speed);
    vel_delta_max = max_accel * dt;
    vel_delta = gimbal_mit_clamp(vel_cmd - *target_vel,
                                 -vel_delta_max,
                                  vel_delta_max);
    *target_vel += vel_delta;

    step = *target_vel * dt;
    if ((err > 0.0f && step > err) || (err < 0.0f && step < err))
    {
        step = err;
        *target_vel = 0.0f;
    }

    return target_set + step;
}

static void gimbal_auto_aim_clear_target_vel(gimbal_motor_t *motor)
{
    if (motor == &gimbal_control.gimbal_yaw_motor)
    {
        yaw_auto_aim_target_vel = 0.0f;
    }
    else if (motor == &gimbal_control.gimbal_pitch_motor)
    {
        pitch_auto_aim_target_vel = 0.0f;
    }
}

static void gimbal_yaw_absolute_angle_limit(gimbal_control_t *control, float add)
{
    gimbal_motor_t *yaw_motor;
    float chassis_yaw;
    float relative_angle_set;
    float desired_relative_angle;
    const float control_dt = (float)GIMBAL_CONTROL_TIME * 0.001f;

    if (control == 0)
    {
        return;
    }

    yaw_motor = &control->gimbal_yaw_motor;
    chassis_yaw = hwt101_get_yaw_total_rad();
    relative_angle_set =
        yaw_motor->absolute_angle_set -
        chassis_yaw -
        yaw_motor->angle_offset;

    if (auto_aim_is_active())
    {
        desired_relative_angle =
            yaw_motor->relative_angle +
            auto_aim_get_yaw_err_rad() +
            add;
        desired_relative_angle =
            gimbal_mit_clamp(desired_relative_angle,
                             yaw_motor->min_relative_angle,
                             yaw_motor->max_relative_angle);
        relative_angle_set =
            gimbal_auto_aim_plan_target(relative_angle_set,
                                        desired_relative_angle,
                                        &yaw_auto_aim_target_vel,
                                        GIMBAL_AUTO_AIM_YAW_KP,
                                        GIMBAL_AUTO_AIM_YAW_MAX_SPEED,
                                        GIMBAL_AUTO_AIM_YAW_MAX_ACCEL,
                                        control_dt);
    }
    else
    {
        yaw_auto_aim_target_vel = 0.0f;
        relative_angle_set += add;
    }

    relative_angle_set =
        gimbal_mit_clamp(relative_angle_set,
                         yaw_motor->min_relative_angle,
                         yaw_motor->max_relative_angle);

    yaw_motor->relative_angle_set = relative_angle_set;
    yaw_motor->absolute_angle_set =
        chassis_yaw +
        yaw_motor->angle_offset +
        relative_angle_set;
}

static void gimbal_pitch_relative_angle_limit(gimbal_control_t *control, float add)
{
    gimbal_motor_t *pitch_motor;
    float desired_relative_angle;
    const float control_dt = (float)GIMBAL_CONTROL_TIME * 0.001f;

    if (control == 0)
    {
        return;
    }

    pitch_motor = &control->gimbal_pitch_motor;
    if (auto_aim_is_active())
    {
        desired_relative_angle =
            pitch_motor->relative_angle +
            auto_aim_get_pitch_err_rad() +
            add;
        desired_relative_angle =
            gimbal_mit_clamp(desired_relative_angle,
                             pitch_motor->min_relative_angle,
                             pitch_motor->max_relative_angle);
        pitch_motor->relative_angle_set =
            gimbal_auto_aim_plan_target(pitch_motor->relative_angle_set,
                                        desired_relative_angle,
                                        &pitch_auto_aim_target_vel,
                                        GIMBAL_AUTO_AIM_PITCH_KP,
                                        GIMBAL_AUTO_AIM_PITCH_MAX_SPEED,
                                        GIMBAL_AUTO_AIM_PITCH_MAX_ACCEL,
                                        control_dt);
    }
    else
    {
        pitch_auto_aim_target_vel = 0.0f;
        pitch_motor->relative_angle_set += add;
    }

    pitch_motor->relative_angle_set =
        gimbal_mit_clamp(pitch_motor->relative_angle_set,
                         pitch_motor->min_relative_angle,
                         pitch_motor->max_relative_angle);
}

static void gimbal_total_pid_clear(gimbal_control_t *control)
{
    if (control == 0)
    {
        return;
    }

    gimbal_pid_clear(&control->gimbal_yaw_motor.absolute_angle_pid);
    gimbal_pid_clear(&control->gimbal_yaw_motor.relative_angle_pid);
    gimbal_pid_clear(&control->gimbal_pitch_motor.absolute_angle_pid);
    gimbal_pid_clear(&control->gimbal_pitch_motor.relative_angle_pid);
}

/**
  * @brief          初始化gimbal_control变量
  * @param[out]     control: 云台控制结构体指针
  * @retval         none
  */
void gimbal_init(gimbal_control_t *control)
{
    gravity_comp_param_t gravity_comp_param;

    if (control == 0)
    {
        return;
    }

    memset(control, 0, sizeof(*control));

    control->gimbal_rc_ctrl = get_remote_control_point();
    control->gimbal_INT_angle_point = get_INS_angle_point();
    control->gimbal_INT_gyro_point = get_gyro_data_point();
    control->gimbal_INT_accel_point = get_accel_data_point();

    control->gimbal_yaw_motor.mode = GIMBAL_MOTOR_RAW;
    control->gimbal_yaw_motor.last_mode = GIMBAL_MOTOR_RAW;
    control->gimbal_pitch_motor.mode = GIMBAL_MOTOR_RAW;
    control->gimbal_pitch_motor.last_mode = GIMBAL_MOTOR_RAW;

    gimbal_pid_init(&control->gimbal_yaw_motor.absolute_angle_pid,
                    YAW_GYRO_ABSOLUTE_PID_KP,
                    YAW_GYRO_ABSOLUTE_PID_KI,
                    YAW_GYRO_ABSOLUTE_PID_KD,
                    YAW_GYRO_ABSOLUTE_PID_MAX_OUT,
                    YAW_GYRO_ABSOLUTE_PID_MAX_IOUT);
    gimbal_pid_init(&control->gimbal_yaw_motor.relative_angle_pid,
                    YAW_ENCODE_RELATIVE_PID_KP,
                    YAW_ENCODE_RELATIVE_PID_KI,
                    YAW_ENCODE_RELATIVE_PID_KD,
                    YAW_ENCODE_RELATIVE_PID_MAX_OUT,
                    YAW_ENCODE_RELATIVE_PID_MAX_IOUT);

    gimbal_pid_init(&control->gimbal_pitch_motor.absolute_angle_pid,
                    PITCH_GYRO_ABSOLUTE_PID_KP,
                    PITCH_GYRO_ABSOLUTE_PID_KI,
                    PITCH_GYRO_ABSOLUTE_PID_KD,
                    PITCH_GYRO_ABSOLUTE_PID_MAX_OUT,
                    PITCH_GYRO_ABSOLUTE_PID_MAX_IOUT);
    gimbal_pid_init(&control->gimbal_pitch_motor.relative_angle_pid,
                    PITCH_ENCODE_RELATIVE_PID_KP,
                    PITCH_ENCODE_RELATIVE_PID_KI,
                    PITCH_ENCODE_RELATIVE_PID_KD,
                    PITCH_ENCODE_RELATIVE_PID_MAX_OUT,
                    PITCH_ENCODE_RELATIVE_PID_MAX_IOUT);

    control->gimbal_yaw_motor.max_relative_angle = YAW_MAX_RELATIVE_ANGLE;
    control->gimbal_yaw_motor.min_relative_angle = YAW_MIN_RELATIVE_ANGLE;
    control->gimbal_pitch_motor.max_relative_angle = PITCH_MAX_RELATIVE_ANGLE;
    control->gimbal_pitch_motor.min_relative_angle = PITCH_MIN_RELATIVE_ANGLE;
    control->gimbal_yaw_motor.inertia_kgm2 = YAW_INERTIA_KGM2;
    control->gimbal_pitch_motor.inertia_kgm2 = PITCH_INERTIA_KGM2;

    Motor_MIT_MODE(&hfdcan2, DM_YAW_CAN_ID);
    Motor_MIT_MODE(&hfdcan2, DM_PIT_CAN_ID);
    Motor_ENABLE(&hfdcan2, DM_YAW_CAN_ID);
    Motor_ENABLE(&hfdcan2, DM_PIT_CAN_ID);
    vTaskDelay(GIMBAL_MIT_FEEDBACK_INIT_DELAY);

    gimbal_total_pid_clear(control);
    gimbal_feedback_update(control);

    control->gimbal_yaw_motor.absolute_angle_set = control->gimbal_yaw_motor.absolute_angle;
    control->gimbal_yaw_motor.relative_angle_set = control->gimbal_yaw_motor.relative_angle;
    control->gimbal_yaw_motor.gyro_set = control->gimbal_yaw_motor.gyro;

    control->gimbal_pitch_motor.absolute_angle_set = control->gimbal_pitch_motor.absolute_angle;
    control->gimbal_pitch_motor.relative_angle_set = control->gimbal_pitch_motor.relative_angle;
    control->gimbal_pitch_motor.gyro_set = control->gimbal_pitch_motor.gyro;

    gravity_comp_param.mass_kg = PITCH_GRAVITY_COMP_MASS_KG;
    gravity_comp_param.com_forward_m = PITCH_GRAVITY_COMP_COM_FORWARD_M;
    gravity_comp_param.com_up_m = PITCH_GRAVITY_COMP_COM_UP_M;
    gravity_comp_param.gravity_mps2 = GRAVITY_COMP_DEFAULT_GRAVITY;
    control->gimbal_pitch_gravity_comp.output_scale = OUTPUT_SCALE;
    control->gimbal_pitch_gravity_comp.output_limit = PITCH_GRAVITY_COMP_OUTPUT_LIMIT;
    gravity_comp_init(&control->gimbal_pitch_gravity_comp, &gravity_comp_param);
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
    float chassis_yaw = 0.0f;
    float yaw_relative = 0.0f;
    float pitch_motor_pos = 0.0f;
    float yaw_gyro_last = 0.0f;
    float pitch_gyro_last = 0.0f;
    float relative_speed_cmd = 0.0f;
    float relative_speed_alpha = 0.0f;
    const float *imu_gyro = hwt906_get_gimbal_gyro_point();
    const float control_dt = (float)GIMBAL_CONTROL_TIME * 0.001f;

    if (control == 0)
    {
        return;
    }

    if (control->gimbal_INT_angle_point != 0)
    {
        control->gimbal_yaw_motor.absolute_angle =
            control->gimbal_INT_angle_point[INS_YAW_ADDRESS_OFFSET];
        pitch_motor_pos = MIT_MOTOR_MEASURE[GIMBAL_PITCH_MIT_INDEX].fdb.pos;

        chassis_yaw = hwt101_get_yaw_total_rad();

        if (control->gimbal_yaw_motor.angle_offset_init == 0u)
        {
            control->gimbal_yaw_motor.angle_offset =
                control->gimbal_yaw_motor.absolute_angle - chassis_yaw;

            control->gimbal_yaw_motor.relative_angle = 0.0f;
            control->gimbal_yaw_motor.relative_angle_set = 0.0f;
            control->gimbal_yaw_motor.absolute_angle_set =
                control->gimbal_yaw_motor.absolute_angle;
            control->gimbal_yaw_motor.angle_offset_init = 1u;
        }
        else
        {
            yaw_relative =
                control->gimbal_yaw_motor.absolute_angle -
                chassis_yaw -
                control->gimbal_yaw_motor.angle_offset;

            control->gimbal_yaw_motor.relative_angle =
                yaw_pitch_direct_wrap_angle(yaw_relative);
        }

        if (control->gimbal_pitch_motor.angle_offset_init == 0u)
        {
            control->gimbal_pitch_motor.angle_offset =
                pitch_motor_pos;

            control->gimbal_pitch_motor.absolute_angle = 0.0f;
            control->gimbal_pitch_motor.relative_angle = 0.0f;
            control->gimbal_pitch_motor.relative_angle_set = 0.0f;
            control->gimbal_pitch_motor.relative_angle_last = 0.0f;
            control->gimbal_pitch_motor.relative_speed = 0.0f;
            control->gimbal_pitch_motor.relative_speed_update_init = 1u;
            control->gimbal_pitch_motor.absolute_angle_set =
                control->gimbal_pitch_motor.absolute_angle;
            control->gimbal_pitch_motor.angle_offset_init = 1u;
        }
        else
        {
            /* MIT pitch 电机正方向与机械 pitch 正方向相同：
             * 上电机械零位为 0，按右手系 pitch 正方向计角。
             */
            control->gimbal_pitch_motor.absolute_angle =
                pitch_motor_pos -
                control->gimbal_pitch_motor.angle_offset;
            control->gimbal_pitch_motor.relative_angle =
                control->gimbal_pitch_motor.absolute_angle;

            if (control->gimbal_pitch_motor.relative_speed_update_init == 0u)
            {
                control->gimbal_pitch_motor.relative_angle_last =
                    control->gimbal_pitch_motor.relative_angle;
                control->gimbal_pitch_motor.relative_speed = 0.0f;
                control->gimbal_pitch_motor.relative_speed_update_init = 1u;
            }
            else
            {
                relative_speed_cmd =
                    (control->gimbal_pitch_motor.relative_angle -
                     control->gimbal_pitch_motor.relative_angle_last) / control_dt;
                relative_speed_alpha =
                    gimbal_mit_clamp(PITCH_RELATIVE_SPEED_FILTER_ALPHA, 0.0f, 1.0f);

                control->gimbal_pitch_motor.relative_speed +=
                    relative_speed_alpha *
                    (relative_speed_cmd - control->gimbal_pitch_motor.relative_speed);
                control->gimbal_pitch_motor.relative_angle_last =
                    control->gimbal_pitch_motor.relative_angle;
            }
        }
    }

    if (control->gimbal_INT_gyro_point != 0)
    {
        yaw_gyro_last = control->gimbal_yaw_motor.gyro;
        pitch_gyro_last = control->gimbal_pitch_motor.gyro;

        control->gimbal_yaw_motor.gyro =
            control->gimbal_INT_gyro_point[INS_GYRO_Z_ADDRESS_OFFSET];
        control->gimbal_pitch_motor.gyro =
            imu_gyro[HWT_AXIS_PITCH];

        if (control->gimbal_yaw_motor.gyro_update_init == 0u)
        {
            control->gimbal_yaw_motor.gyro_last = control->gimbal_yaw_motor.gyro;
            control->gimbal_yaw_motor.gyro_accel = 0.0f;
            control->gimbal_yaw_motor.gyro_update_init = 1u;
        }
        else
        {
            control->gimbal_yaw_motor.gyro_last = yaw_gyro_last;
            control->gimbal_yaw_motor.gyro_accel =
                (control->gimbal_yaw_motor.gyro - yaw_gyro_last) / control_dt;
        }

        if (control->gimbal_pitch_motor.gyro_update_init == 0u)
        {
            control->gimbal_pitch_motor.gyro_last = control->gimbal_pitch_motor.gyro;
            control->gimbal_pitch_motor.gyro_accel = 0.0f;
            control->gimbal_pitch_motor.gyro_update_init = 1u;
        }
        else
        {
            control->gimbal_pitch_motor.gyro_last = pitch_gyro_last;
            control->gimbal_pitch_motor.gyro_accel =
                (control->gimbal_pitch_motor.gyro - pitch_gyro_last) / control_dt;
        }
    }
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
        auto_aim_reset_delta_accum();
        gimbal_feedforward_clear(&control->gimbal_yaw_motor);
        control->gimbal_yaw_motor.raw_cmd = add_yaw;
    }
    else if (control->gimbal_yaw_motor.mode == GIMBAL_MOTOR_GYRO)
    {
        gimbal_yaw_absolute_angle_limit(control, add_yaw);
        gimbal_feedforward_track_target(&control->gimbal_yaw_motor,
                                        control->gimbal_yaw_motor.absolute_angle_set);
    }
    else if (control->gimbal_yaw_motor.mode == GIMBAL_MOTOR_ENCODE)
    {
        control->gimbal_yaw_motor.relative_angle_set += add_yaw;
        gimbal_feedforward_track_target(&control->gimbal_yaw_motor,
                                        control->gimbal_yaw_motor.relative_angle_set);
    }

    if (control->gimbal_pitch_motor.mode == GIMBAL_MOTOR_RAW)
    {
        auto_aim_reset_delta_accum();
        gimbal_feedforward_clear(&control->gimbal_pitch_motor);
        control->gimbal_pitch_motor.raw_cmd = add_pitch;
    }
    else if (control->gimbal_pitch_motor.mode == GIMBAL_MOTOR_GYRO)
    {
        gimbal_absolute_angle_limit(&control->gimbal_pitch_motor, add_pitch);
        gimbal_feedforward_track_target(&control->gimbal_pitch_motor,
                                        control->gimbal_pitch_motor.absolute_angle_set);
    }
    else if (control->gimbal_pitch_motor.mode == GIMBAL_MOTOR_ENCODE)
    {
        gimbal_pitch_relative_angle_limit(control, add_pitch);
        gimbal_feedforward_track_target(&control->gimbal_pitch_motor,
                                        control->gimbal_pitch_motor.relative_angle_set);
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
    float yaw_cmd_torque;
    float pitch_cmd_torque;

    if (control == 0)
    {
        return;
    }

    yaw_can_set_current = control->gimbal_yaw_motor.given_current;
    pitch_can_set_current = control->gimbal_pitch_motor.given_current;

    yaw_cmd_torque = gimbal_output_to_mit_torque(yaw_can_set_current);
    pitch_cmd_torque = gimbal_output_to_mit_torque(pitch_can_set_current);

    CAN_cmd_MIT(&hfdcan2, DM_YAW_CAN_ID, 0.0f, 0.0f, 0.0f, 0.0f, yaw_cmd_torque);
    CAN_cmd_MIT(&hfdcan2, DM_PIT_CAN_ID, 0.0f, 0.0f, 0.0f, 0.0f, pitch_cmd_torque);
}
