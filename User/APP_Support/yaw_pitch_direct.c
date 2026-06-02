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
#ifndef GIMBAL_PITCH_MIT_FDB_TIMEOUT
#define GIMBAL_PITCH_MIT_FDB_TIMEOUT 100U
#endif

float yaw_can_set_current = 0.0f;
float pitch_can_set_current = 0.0f;
int16_t shoot_can_set_current = 0;

typedef enum
{
    GIMBAL_CONTROL_SOURCE_RC = 0,
    GIMBAL_CONTROL_SOURCE_AUTO,
} gimbal_control_source_e;

static float yaw_auto_aim_target_vel = 0.0f;
static float pitch_auto_aim_target_vel = 0.0f;

static uint8_t gimbal_pitch_mit_feedback_ready(void);
static void gimbal_pitch_zero_output(gimbal_motor_t *motor);

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

static uint8_t gimbal_pitch_mit_feedback_ready(void)
{
    const MITMeasure_t *measure = &MIT_MOTOR_MEASURE[GIMBAL_PITCH_MIT_INDEX];
    uint32_t now = HAL_GetTick();

    return (uint8_t)((measure->fdb.last_fdb_time != 0U) &&
                     ((uint32_t)(now - measure->fdb.last_fdb_time) <=
                      (uint32_t)GIMBAL_PITCH_MIT_FDB_TIMEOUT));
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
    motor->rc_ref_vel = 0.0f;
    motor->rc_ref_vel_last = 0.0f;
    motor->rc_ref_accel = 0.0f;
    motor->rc_ref_target_last = 0.0f;
    motor->rc_ref_target_init = 0u;
    motor->auto_ref_vel = 0.0f;
    motor->auto_ref_vel_last = 0.0f;
    motor->auto_ref_accel = 0.0f;
    motor->auto_ref_target_last = 0.0f;
    motor->auto_ref_target_init = 0u;
    motor->ff_torque = 0.0f;
    gimbal_auto_aim_clear_target_vel(motor);
}

static void gimbal_pitch_zero_output(gimbal_motor_t *motor)
{
    if (motor == 0)
    {
        return;
    }

    gimbal_feedforward_clear(motor);
    gimbal_pid_clear(&motor->absolute_angle_pid);
    gimbal_pid_clear(&motor->relative_angle_pid);

    motor->mode = GIMBAL_MOTOR_RAW;
    motor->last_mode = GIMBAL_MOTOR_RAW;
    motor->raw_cmd = 0.0f;
    motor->current_set = 0.0f;
    motor->output = 0.0f;
    motor->given_current = 0.0f;
    motor->pid_torque = 0.0f;
    motor->static_friction_comp = 0.0f;
    motor->rc_control_enable = 0u;
    motor->auto_control_enable = 0u;
    motor->absolute_angle_set = motor->absolute_angle;
    motor->relative_angle_set = motor->relative_angle;
    motor->rc_absolute_angle_set = motor->absolute_angle;
    motor->rc_relative_angle_set = motor->relative_angle;
    motor->auto_absolute_angle_set = motor->absolute_angle;
    motor->auto_relative_angle_set = motor->relative_angle;
    motor->gyro_set = motor->gyro;
}

static void gimbal_feedforward_clear_source(gimbal_motor_t *motor,
                                            gimbal_control_source_e source)
{
    if (motor == 0)
    {
        return;
    }

    if (source == GIMBAL_CONTROL_SOURCE_AUTO)
    {
        motor->auto_ref_vel = 0.0f;
        motor->auto_ref_vel_last = 0.0f;
        motor->auto_ref_accel = 0.0f;
        motor->auto_ref_target_last = 0.0f;
        motor->auto_ref_target_init = 0u;
        gimbal_auto_aim_clear_target_vel(motor);
        return;
    }

    motor->rc_ref_vel = 0.0f;
    motor->rc_ref_vel_last = 0.0f;
    motor->rc_ref_accel = 0.0f;
    motor->rc_ref_target_last = 0.0f;
    motor->rc_ref_target_init = 0u;
}

static void gimbal_feedforward_track_target(gimbal_motor_t *motor,
                                            float target_angle,
                                            gimbal_control_source_e source)
{
    float ref_vel_cmd;
    float ref_accel_cmd;
    float alpha;
    float accel_limit;
    float *ref_vel;
    float *ref_vel_last;
    float *ref_accel;
    float *target_last;
    uint8_t *target_init;
    const float control_dt = (float)GIMBAL_CONTROL_TIME * 0.001f;

    if (motor == 0 || control_dt <= 0.0f)
    {
        return;
    }

    if (source == GIMBAL_CONTROL_SOURCE_AUTO)
    {
        ref_vel = &motor->auto_ref_vel;
        ref_vel_last = &motor->auto_ref_vel_last;
        ref_accel = &motor->auto_ref_accel;
        target_last = &motor->auto_ref_target_last;
        target_init = &motor->auto_ref_target_init;
    }
    else
    {
        ref_vel = &motor->rc_ref_vel;
        ref_vel_last = &motor->rc_ref_vel_last;
        ref_accel = &motor->rc_ref_accel;
        target_last = &motor->rc_ref_target_last;
        target_init = &motor->rc_ref_target_init;
    }

    if (*target_init == 0u)
    {
        *target_last = target_angle;
        *target_init = 1u;
        *ref_vel_last = 0.0f;
        *ref_vel = 0.0f;
        *ref_accel = 0.0f;
        return;
    }

    alpha = gimbal_mit_clamp(YAW_REF_VEL_FILTER_ALPHA, 0.0f, 1.0f);
    ref_vel_cmd = (target_angle - *target_last) / control_dt;

    *ref_vel_last = *ref_vel;
    *ref_vel += alpha * (ref_vel_cmd - *ref_vel);

    ref_accel_cmd = (*ref_vel - *ref_vel_last) / control_dt;
    accel_limit = YAW_REF_ACCEL_LIMIT;
    if (accel_limit > 0.0f)
    {
        ref_accel_cmd = gimbal_mit_clamp(ref_accel_cmd,
                                         -accel_limit,
                                         accel_limit);
    }

    *ref_accel = ref_accel_cmd;
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

static void gimbal_yaw_absolute_angle_limit(gimbal_control_t *control,
                                            float add,
                                            uint8_t rc_enable,
                                            uint8_t auto_enable)
{
    gimbal_motor_t *yaw_motor;
    float chassis_yaw;
    float rc_relative_angle_set;
    float auto_relative_angle_set;
    float desired_relative_angle;
    const float control_dt = (float)GIMBAL_CONTROL_TIME * 0.001f;

    if (control == 0)
    {
        return;
    }

    yaw_motor = &control->gimbal_yaw_motor;
    chassis_yaw = hwt101_get_yaw_total_rad();
    rc_relative_angle_set =
        yaw_motor->rc_absolute_angle_set -
        chassis_yaw -
        yaw_motor->angle_offset;

    if (rc_enable != 0u)
    {
        rc_relative_angle_set += add;
    }
    else
    {
        rc_relative_angle_set = yaw_motor->relative_angle;
        gimbal_feedforward_clear_source(yaw_motor, GIMBAL_CONTROL_SOURCE_RC);
    }

    rc_relative_angle_set =
        gimbal_mit_clamp(rc_relative_angle_set,
                         yaw_motor->min_relative_angle,
                         yaw_motor->max_relative_angle);
    yaw_motor->rc_relative_angle_set = rc_relative_angle_set;
    yaw_motor->rc_absolute_angle_set =
        chassis_yaw + yaw_motor->angle_offset + rc_relative_angle_set;

    if (auto_enable != 0u)
    {
        desired_relative_angle =
            yaw_motor->relative_angle +
            auto_aim_get_yaw_err_rad();
        desired_relative_angle =
            gimbal_mit_clamp(desired_relative_angle,
                             yaw_motor->min_relative_angle,
                             yaw_motor->max_relative_angle);
        auto_relative_angle_set =
            gimbal_auto_aim_plan_target(yaw_motor->auto_relative_angle_set,
                                        desired_relative_angle,
                                        &yaw_auto_aim_target_vel,
                                        GIMBAL_AUTO_AIM_YAW_KP,
                                        GIMBAL_AUTO_AIM_YAW_MAX_SPEED,
                                        GIMBAL_AUTO_AIM_YAW_MAX_ACCEL,
                                        control_dt);
        auto_relative_angle_set =
            gimbal_mit_clamp(auto_relative_angle_set,
                             yaw_motor->min_relative_angle,
                             yaw_motor->max_relative_angle);
    }
    else
    {
        auto_relative_angle_set = rc_relative_angle_set;
    }

    yaw_motor->auto_relative_angle_set = auto_relative_angle_set;
    yaw_motor->auto_absolute_angle_set =
        chassis_yaw + yaw_motor->angle_offset + auto_relative_angle_set;

    yaw_motor->relative_angle_set =
        rc_relative_angle_set +
        ((auto_enable != 0u) ? (auto_relative_angle_set - yaw_motor->relative_angle) : 0.0f);
    yaw_motor->absolute_angle_set =
        chassis_yaw + yaw_motor->angle_offset + yaw_motor->relative_angle_set;
}

static void gimbal_pitch_relative_angle_limit(gimbal_control_t *control,
                                              float add,
                                              uint8_t rc_enable,
                                              uint8_t auto_enable)
{
    gimbal_motor_t *pitch_motor;
    float rc_relative_angle_set;
    float desired_relative_angle;
    float auto_relative_angle_set;
    const float control_dt = (float)GIMBAL_CONTROL_TIME * 0.001f;

    if (control == 0)
    {
        return;
    }

    pitch_motor = &control->gimbal_pitch_motor;

    if (rc_enable != 0u)
    {
        rc_relative_angle_set = pitch_motor->rc_relative_angle_set + add;
    }
    else
    {
        rc_relative_angle_set = pitch_motor->relative_angle;
        gimbal_feedforward_clear_source(pitch_motor, GIMBAL_CONTROL_SOURCE_RC);
    }

    pitch_motor->rc_relative_angle_set =
        gimbal_mit_clamp(rc_relative_angle_set,
                         pitch_motor->min_relative_angle,
                         pitch_motor->max_relative_angle);
    pitch_motor->rc_absolute_angle_set = pitch_motor->rc_relative_angle_set;

    if (auto_enable != 0u)
    {
        desired_relative_angle =
            pitch_motor->relative_angle +
            auto_aim_get_pitch_err_rad();
        desired_relative_angle =
            gimbal_mit_clamp(desired_relative_angle,
                             pitch_motor->min_relative_angle,
                             pitch_motor->max_relative_angle);
        auto_relative_angle_set =
            gimbal_auto_aim_plan_target(pitch_motor->auto_relative_angle_set,
                                        desired_relative_angle,
                                        &pitch_auto_aim_target_vel,
                                        GIMBAL_AUTO_AIM_PITCH_KP,
                                        GIMBAL_AUTO_AIM_PITCH_MAX_SPEED,
                                        GIMBAL_AUTO_AIM_PITCH_MAX_ACCEL,
                                        control_dt);
        pitch_motor->auto_relative_angle_set =
            gimbal_mit_clamp(auto_relative_angle_set,
                             pitch_motor->min_relative_angle,
                             pitch_motor->max_relative_angle);
    }
    else
    {
        pitch_motor->auto_relative_angle_set = pitch_motor->rc_relative_angle_set;
    }

    pitch_motor->auto_absolute_angle_set = pitch_motor->auto_relative_angle_set;
    pitch_motor->relative_angle_set =
        pitch_motor->rc_relative_angle_set +
        ((auto_enable != 0u) ?
             (pitch_motor->auto_relative_angle_set - pitch_motor->relative_angle) :
             0.0f);
    pitch_motor->absolute_angle_set = pitch_motor->relative_angle_set;
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
		
//		Motor_MIT_MODE(&hfdcan1, DM_YAW_CAN_ID);
    Motor_ENABLE(&hfdcan1, DM_YAW_CAN_ID);
//		Motor_MIT_MODE(&hfdcan2, DM_PIT_CAN_ID);
    Motor_ENABLE(&hfdcan2, DM_PIT_CAN_ID);

    gimbal_total_pid_clear(control);
    gimbal_feedback_update(control);

    control->gimbal_yaw_motor.absolute_angle_set = control->gimbal_yaw_motor.absolute_angle;
    control->gimbal_yaw_motor.relative_angle_set = control->gimbal_yaw_motor.relative_angle;
    control->gimbal_yaw_motor.rc_absolute_angle_set = control->gimbal_yaw_motor.absolute_angle;
    control->gimbal_yaw_motor.rc_relative_angle_set = control->gimbal_yaw_motor.relative_angle;
    control->gimbal_yaw_motor.auto_absolute_angle_set = control->gimbal_yaw_motor.absolute_angle;
    control->gimbal_yaw_motor.auto_relative_angle_set = control->gimbal_yaw_motor.relative_angle;
    control->gimbal_yaw_motor.gyro_set = control->gimbal_yaw_motor.gyro;

    control->gimbal_pitch_motor.absolute_angle_set = control->gimbal_pitch_motor.absolute_angle;
    control->gimbal_pitch_motor.relative_angle_set = control->gimbal_pitch_motor.relative_angle;
    control->gimbal_pitch_motor.rc_absolute_angle_set = control->gimbal_pitch_motor.absolute_angle;
    control->gimbal_pitch_motor.rc_relative_angle_set = control->gimbal_pitch_motor.relative_angle;
    control->gimbal_pitch_motor.auto_absolute_angle_set = control->gimbal_pitch_motor.absolute_angle;
    control->gimbal_pitch_motor.auto_relative_angle_set = control->gimbal_pitch_motor.relative_angle;
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
    if (gimbal_pitch_mit_feedback_ready() == 0u)
    {
        gimbal_pitch_zero_output(&control->gimbal_pitch_motor);
    }
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
    uint8_t pitch_feedback_ready = gimbal_pitch_mit_feedback_ready();
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
        if (pitch_feedback_ready != 0u)
        {
            pitch_motor_pos = MIT_MOTOR_MEASURE[GIMBAL_PITCH_MIT_INDEX].fdb.pos;
        }

        chassis_yaw = hwt101_get_yaw_total_rad();

        if (control->gimbal_yaw_motor.angle_offset_init == 0u)
        {
            control->gimbal_yaw_motor.angle_offset = 0.0f;

            control->gimbal_yaw_motor.relative_angle = 0.0f;
            control->gimbal_yaw_motor.relative_angle_set = 0.0f;
            control->gimbal_yaw_motor.rc_relative_angle_set = 0.0f;
            control->gimbal_yaw_motor.auto_relative_angle_set = 0.0f;
            control->gimbal_yaw_motor.absolute_angle_set =
                control->gimbal_yaw_motor.absolute_angle;
            control->gimbal_yaw_motor.rc_absolute_angle_set =
                control->gimbal_yaw_motor.absolute_angle;
            control->gimbal_yaw_motor.auto_absolute_angle_set =
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

        if (pitch_feedback_ready == 0u)
        {
            control->gimbal_pitch_motor.relative_speed = 0.0f;
            control->gimbal_pitch_motor.relative_speed_update_init = 0u;
        }
        else if (control->gimbal_pitch_motor.angle_offset_init == 0u)
        {
            control->gimbal_pitch_motor.angle_offset =
                pitch_motor_pos;

            control->gimbal_pitch_motor.absolute_angle = 0.0f;
            control->gimbal_pitch_motor.relative_angle = 0.0f;
            control->gimbal_pitch_motor.relative_angle_set = 0.0f;
            control->gimbal_pitch_motor.rc_relative_angle_set = 0.0f;
            control->gimbal_pitch_motor.auto_relative_angle_set = 0.0f;
            control->gimbal_pitch_motor.relative_angle_last = 0.0f;
            control->gimbal_pitch_motor.relative_speed = 0.0f;
            control->gimbal_pitch_motor.relative_speed_update_init = 1u;
            control->gimbal_pitch_motor.absolute_angle_set =
                control->gimbal_pitch_motor.absolute_angle;
            control->gimbal_pitch_motor.rc_absolute_angle_set =
                control->gimbal_pitch_motor.absolute_angle;
            control->gimbal_pitch_motor.auto_absolute_angle_set =
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
        control->gimbal_yaw_motor.rc_absolute_angle_set = control->gimbal_yaw_motor.absolute_angle;
        control->gimbal_yaw_motor.auto_absolute_angle_set = control->gimbal_yaw_motor.absolute_angle;
        control->gimbal_yaw_motor.gyro_set = control->gimbal_yaw_motor.gyro;
    }
    else if (control->gimbal_yaw_motor.last_mode != GIMBAL_MOTOR_ENCODE && control->gimbal_yaw_motor.mode == GIMBAL_MOTOR_ENCODE)
    {
        control->gimbal_yaw_motor.relative_angle_set = control->gimbal_yaw_motor.relative_angle;
        control->gimbal_yaw_motor.rc_relative_angle_set = control->gimbal_yaw_motor.relative_angle;
        control->gimbal_yaw_motor.auto_relative_angle_set = control->gimbal_yaw_motor.relative_angle;
        control->gimbal_yaw_motor.gyro_set = control->gimbal_yaw_motor.gyro;
    }
    control->gimbal_yaw_motor.last_mode = control->gimbal_yaw_motor.mode;

    if (gimbal_pitch_mit_feedback_ready() == 0u)
    {
        gimbal_pitch_zero_output(&control->gimbal_pitch_motor);
        return;
    }

    if (control->gimbal_pitch_motor.last_mode != GIMBAL_MOTOR_RAW && control->gimbal_pitch_motor.mode == GIMBAL_MOTOR_RAW)
    {
        control->gimbal_pitch_motor.raw_cmd = control->gimbal_pitch_motor.current_set = control->gimbal_pitch_motor.given_current;
    }
    else if (control->gimbal_pitch_motor.last_mode != GIMBAL_MOTOR_GYRO && control->gimbal_pitch_motor.mode == GIMBAL_MOTOR_GYRO)
    {
        control->gimbal_pitch_motor.absolute_angle_set = control->gimbal_pitch_motor.absolute_angle;
        control->gimbal_pitch_motor.rc_absolute_angle_set = control->gimbal_pitch_motor.absolute_angle;
        control->gimbal_pitch_motor.auto_absolute_angle_set = control->gimbal_pitch_motor.absolute_angle;
        control->gimbal_pitch_motor.gyro_set = control->gimbal_pitch_motor.gyro;
    }
    else if (control->gimbal_pitch_motor.last_mode != GIMBAL_MOTOR_ENCODE && control->gimbal_pitch_motor.mode == GIMBAL_MOTOR_ENCODE)
    {
        control->gimbal_pitch_motor.relative_angle_set = control->gimbal_pitch_motor.relative_angle;
        control->gimbal_pitch_motor.rc_relative_angle_set = control->gimbal_pitch_motor.relative_angle;
        control->gimbal_pitch_motor.auto_relative_angle_set = control->gimbal_pitch_motor.relative_angle;
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
    uint8_t auto_enable;
    uint8_t yaw_rc_enable;
    uint8_t pitch_rc_enable;
    gimbal_motor_t *yaw_motor;
    gimbal_motor_t *pitch_motor;

    if (control == 0)
    {
        return;
    }

    gimbal_behaviour_control_set(&add_yaw, &add_pitch, control);
    auto_enable = auto_aim_is_active();
    yaw_rc_enable =
        (uint8_t)((auto_enable == 0u) || (add_yaw != 0.0f));
    pitch_rc_enable =
        (uint8_t)((auto_enable == 0u) || (add_pitch != 0.0f));
    yaw_motor = &control->gimbal_yaw_motor;
    pitch_motor = &control->gimbal_pitch_motor;

    if (yaw_motor->mode == GIMBAL_MOTOR_RAW)
    {
        auto_aim_reset_delta_accum();
        gimbal_feedforward_clear(yaw_motor);
        yaw_motor->rc_control_enable = 0u;
        yaw_motor->auto_control_enable = 0u;
        yaw_motor->raw_cmd = add_yaw;
    }
    else if (yaw_motor->mode == GIMBAL_MOTOR_GYRO)
    {
        yaw_motor->rc_control_enable = yaw_rc_enable;
        yaw_motor->auto_control_enable = auto_enable;
        gimbal_yaw_absolute_angle_limit(control,
                                        add_yaw,
                                        yaw_rc_enable,
                                        auto_enable);
        if (yaw_rc_enable != 0u)
        {
            gimbal_feedforward_track_target(yaw_motor,
                                            yaw_motor->rc_absolute_angle_set,
                                            GIMBAL_CONTROL_SOURCE_RC);
        }
        if (auto_enable != 0u)
        {
            gimbal_feedforward_track_target(yaw_motor,
                                            yaw_motor->auto_absolute_angle_set,
                                            GIMBAL_CONTROL_SOURCE_AUTO);
        }
        else
        {
            gimbal_feedforward_track_target(yaw_motor,
                                            yaw_motor->auto_absolute_angle_set,
                                            GIMBAL_CONTROL_SOURCE_AUTO);
        }
    }
    else if (yaw_motor->mode == GIMBAL_MOTOR_ENCODE)
    {
        yaw_motor->rc_control_enable = yaw_rc_enable;
        yaw_motor->auto_control_enable = auto_enable;
        if (yaw_rc_enable != 0u)
        {
            yaw_motor->rc_relative_angle_set += add_yaw;
        }
        else
        {
            yaw_motor->rc_relative_angle_set = yaw_motor->relative_angle;
            gimbal_feedforward_clear_source(yaw_motor, GIMBAL_CONTROL_SOURCE_RC);
        }
        yaw_motor->rc_relative_angle_set =
            gimbal_mit_clamp(yaw_motor->rc_relative_angle_set,
                             yaw_motor->min_relative_angle,
                             yaw_motor->max_relative_angle);
        if (auto_enable != 0u)
        {
            yaw_motor->auto_relative_angle_set =
                gimbal_auto_aim_plan_target(yaw_motor->auto_relative_angle_set,
                                            yaw_motor->relative_angle + auto_aim_get_yaw_err_rad(),
                                            &yaw_auto_aim_target_vel,
                                            GIMBAL_AUTO_AIM_YAW_KP,
                                            GIMBAL_AUTO_AIM_YAW_MAX_SPEED,
                                            GIMBAL_AUTO_AIM_YAW_MAX_ACCEL,
                                            (float)GIMBAL_CONTROL_TIME * 0.001f);
            yaw_motor->auto_relative_angle_set =
                gimbal_mit_clamp(yaw_motor->auto_relative_angle_set,
                                 yaw_motor->min_relative_angle,
                                 yaw_motor->max_relative_angle);
        }
        else
        {
            yaw_motor->auto_relative_angle_set = yaw_motor->rc_relative_angle_set;
        }
        yaw_motor->relative_angle_set =
            yaw_motor->rc_relative_angle_set +
            ((auto_enable != 0u) ?
                 (yaw_motor->auto_relative_angle_set - yaw_motor->relative_angle) :
                 0.0f);
        if (yaw_rc_enable != 0u)
        {
            gimbal_feedforward_track_target(yaw_motor,
                                            yaw_motor->rc_relative_angle_set,
                                            GIMBAL_CONTROL_SOURCE_RC);
        }
        if (auto_enable != 0u)
        {
            gimbal_feedforward_track_target(yaw_motor,
                                            yaw_motor->auto_relative_angle_set,
                                            GIMBAL_CONTROL_SOURCE_AUTO);
        }
        else
        {
            gimbal_feedforward_track_target(yaw_motor,
                                            yaw_motor->auto_relative_angle_set,
                                            GIMBAL_CONTROL_SOURCE_AUTO);
        }
    }

    if (gimbal_pitch_mit_feedback_ready() == 0u)
    {
        gimbal_pitch_zero_output(pitch_motor);
        return;
    }

    if (pitch_motor->mode == GIMBAL_MOTOR_RAW)
    {
        auto_aim_reset_delta_accum();
        gimbal_feedforward_clear(pitch_motor);
        pitch_motor->rc_control_enable = 0u;
        pitch_motor->auto_control_enable = 0u;
        pitch_motor->raw_cmd = add_pitch;
    }
    else if (pitch_motor->mode == GIMBAL_MOTOR_GYRO)
    {
        pitch_motor->rc_control_enable = pitch_rc_enable;
        pitch_motor->auto_control_enable = auto_enable;
        if (pitch_rc_enable != 0u)
        {
            pitch_motor->rc_absolute_angle_set =
                gimbal_mit_clamp(pitch_motor->rc_absolute_angle_set + add_pitch,
                                 pitch_motor->min_relative_angle,
                                 pitch_motor->max_relative_angle);
        }
        else
        {
            pitch_motor->rc_absolute_angle_set = pitch_motor->absolute_angle;
            gimbal_feedforward_clear_source(pitch_motor, GIMBAL_CONTROL_SOURCE_RC);
        }
        if (auto_enable != 0u)
        {
            pitch_motor->auto_absolute_angle_set =
                gimbal_auto_aim_plan_target(pitch_motor->auto_absolute_angle_set,
                                            pitch_motor->absolute_angle + auto_aim_get_pitch_err_rad(),
                                            &pitch_auto_aim_target_vel,
                                            GIMBAL_AUTO_AIM_PITCH_KP,
                                            GIMBAL_AUTO_AIM_PITCH_MAX_SPEED,
                                            GIMBAL_AUTO_AIM_PITCH_MAX_ACCEL,
                                            (float)GIMBAL_CONTROL_TIME * 0.001f);
            pitch_motor->auto_absolute_angle_set =
                gimbal_mit_clamp(pitch_motor->auto_absolute_angle_set,
                                 pitch_motor->min_relative_angle,
                                 pitch_motor->max_relative_angle);
        }
        else
        {
            pitch_motor->auto_absolute_angle_set = pitch_motor->rc_absolute_angle_set;
        }
        pitch_motor->absolute_angle_set =
            pitch_motor->rc_absolute_angle_set +
            ((auto_enable != 0u) ?
                 (pitch_motor->auto_absolute_angle_set - pitch_motor->absolute_angle) :
                 0.0f);
        if (pitch_rc_enable != 0u)
        {
            gimbal_feedforward_track_target(pitch_motor,
                                            pitch_motor->rc_absolute_angle_set,
                                            GIMBAL_CONTROL_SOURCE_RC);
        }
        if (auto_enable != 0u)
        {
            gimbal_feedforward_track_target(pitch_motor,
                                            pitch_motor->auto_absolute_angle_set,
                                            GIMBAL_CONTROL_SOURCE_AUTO);
        }
        else
        {
            gimbal_feedforward_track_target(pitch_motor,
                                            pitch_motor->auto_absolute_angle_set,
                                            GIMBAL_CONTROL_SOURCE_AUTO);
        }
    }
    else if (pitch_motor->mode == GIMBAL_MOTOR_ENCODE)
    {
        pitch_motor->rc_control_enable = pitch_rc_enable;
        pitch_motor->auto_control_enable = auto_enable;
        gimbal_pitch_relative_angle_limit(control,
                                          add_pitch,
                                          pitch_rc_enable,
                                          auto_enable);
        if (pitch_rc_enable != 0u)
        {
            gimbal_feedforward_track_target(pitch_motor,
                                            pitch_motor->rc_relative_angle_set,
                                            GIMBAL_CONTROL_SOURCE_RC);
        }
        if (auto_enable != 0u)
        {
            gimbal_feedforward_track_target(pitch_motor,
                                            pitch_motor->auto_relative_angle_set,
                                            GIMBAL_CONTROL_SOURCE_AUTO);
        }
        else
        {
            gimbal_feedforward_track_target(pitch_motor,
                                            pitch_motor->auto_relative_angle_set,
                                            GIMBAL_CONTROL_SOURCE_AUTO);
        }
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

    if (gimbal_pitch_mit_feedback_ready() == 0u)
    {
        gimbal_pitch_zero_output(&control->gimbal_pitch_motor);
        return;
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
    if (gimbal_pitch_mit_feedback_ready() == 0u)
    {
        pitch_can_set_current = 0.0f;
    }
    else
    {
        pitch_can_set_current = control->gimbal_pitch_motor.given_current;
    }

    yaw_cmd_torque = gimbal_output_to_mit_torque(yaw_can_set_current);
    pitch_cmd_torque = gimbal_output_to_mit_torque(pitch_can_set_current);

    CAN_cmd_MIT(&hfdcan1, DM_YAW_CAN_ID, 0.0f, 0.0f, 0.0f, 0.0f, yaw_cmd_torque);
    CAN_cmd_MIT(&hfdcan2, DM_PIT_CAN_ID, 0.0f, 0.0f, 0.0f, 0.0f, pitch_cmd_torque);
}
