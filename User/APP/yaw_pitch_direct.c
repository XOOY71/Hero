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
#include "cmsis_os.h"
#include <math.h>
#include <string.h>

#define YAW_PITCH_DIRECT_PI 3.14159265358979323846f

#ifndef GIMBAL_MIT_FEEDBACK_INIT_DELAY
#define GIMBAL_MIT_FEEDBACK_INIT_DELAY 100U
#endif

float yaw_can_set_current = 0.0f;
float pitch_can_set_current = 0.0f;
int16_t shoot_can_set_current = 0;

static float yaw_ref_target_last = 0.0f;
static uint8_t yaw_ref_target_init = 0u;

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

static float gimbal_take_auto_aim_bias(gimbal_motor_t *motor)
{
    float bias = 0.0f;

    if (motor == &gimbal_control.gimbal_yaw_motor)
    {
        bias = aim.receive.yaw;
        aim.receive.yaw = 0.0f;
    }
    else if (motor == &gimbal_control.gimbal_pitch_motor)
    {
        bias = aim.receive.pitch;
        aim.receive.pitch = 0.0f;
    }

    return bias;
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

    if (motor == &gimbal_control.gimbal_yaw_motor)
    {
        yaw_ref_target_last = 0.0f;
        yaw_ref_target_init = 0u;
    }
}

static float gimbal_pitch_soft_limit_add(gimbal_motor_t *motor, float angle_add)
{
    float angle_set;
    float distance_to_limit;
    float scale;

    if (motor != &gimbal_control.gimbal_pitch_motor || PITCH_SOFT_LIMIT_BUFFER_ANGLE <= 0.0f)
    {
        return angle_add;
    }

    angle_set = (motor->mode == GIMBAL_MOTOR_GYRO) ? motor->absolute_angle_set : motor->relative_angle_set;

    if (angle_add > 0.0f)
    {
        distance_to_limit = motor->max_relative_angle - fmaxf(motor->relative_angle, angle_set);
    }
    else if (angle_add < 0.0f)
    {
        distance_to_limit = fminf(motor->relative_angle, angle_set) - motor->min_relative_angle;
    }
    else
    {
        return 0.0f;
    }

    if (distance_to_limit <= 0.0f)
    {
        return 0.0f;
    }

    scale = gimbal_mit_clamp(distance_to_limit / PITCH_SOFT_LIMIT_BUFFER_ANGLE, 0.0f, 1.0f);
    return angle_add * scale;
}

static float gimbal_feedforward_update(gimbal_motor_t *motor, float angle_add)
{
    float ref_vel_cmd;
    float ref_accel_cmd;
    float max_ref_accel;
    float ramp_time;
    const float control_dt = (float)GIMBAL_CONTROL_TIME * 0.001f;

    if (motor == 0 || control_dt <= 0.0f)
    {
        return 0.0f;
    }

    ramp_time = (motor == &gimbal_control.gimbal_yaw_motor) ? YAW_REF_ACCEL_RAMP_TIME : PITCH_REF_ACCEL_RAMP_TIME;
    if (ramp_time <= 0.0f)
    {
        ramp_time = control_dt;
    }

    angle_add = gimbal_pitch_soft_limit_add(motor, angle_add);
    ref_vel_cmd = angle_add / control_dt;
    max_ref_accel = fabsf(ref_vel_cmd - motor->ref_vel) / ramp_time;
    ref_accel_cmd = (ref_vel_cmd - motor->ref_vel) / control_dt;
    ref_accel_cmd = gimbal_mit_clamp(ref_accel_cmd, -max_ref_accel, max_ref_accel);

    motor->ref_vel_last = motor->ref_vel;
    motor->ref_vel += ref_accel_cmd * control_dt;
    motor->ref_accel = ref_accel_cmd;

    return motor->ref_vel * control_dt;
}

static void gimbal_feedforward_track_target(gimbal_motor_t *motor, float target_angle)
{
    float ref_vel_cmd;
    float ref_accel_cmd;
    float alpha;
    const float control_dt = (float)GIMBAL_CONTROL_TIME * 0.001f;

    if (motor == 0 || control_dt <= 0.0f)
    {
        return;
    }

    if (motor != &gimbal_control.gimbal_yaw_motor)
    {
        return;
    }

    if (yaw_ref_target_init == 0u)
    {
        yaw_ref_target_last = target_angle;
        yaw_ref_target_init = 1u;
        motor->ref_vel_last = 0.0f;
        motor->ref_vel = 0.0f;
        motor->ref_accel = 0.0f;
        return;
    }

    alpha = gimbal_mit_clamp(YAW_REF_VEL_FILTER_ALPHA, 0.0f, 1.0f);
    ref_vel_cmd = (target_angle - yaw_ref_target_last) / control_dt;

    motor->ref_vel_last = motor->ref_vel;
    motor->ref_vel += alpha * (ref_vel_cmd - motor->ref_vel);

    ref_accel_cmd = (motor->ref_vel - motor->ref_vel_last) / control_dt;
    if (YAW_REF_ACCEL_LIMIT > 0.0f)
    {
        ref_accel_cmd = gimbal_mit_clamp(ref_accel_cmd,
                                         -YAW_REF_ACCEL_LIMIT,
                                         YAW_REF_ACCEL_LIMIT);
    }

    motor->ref_accel = ref_accel_cmd;
    yaw_ref_target_last = target_angle;
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
        }
    }

    if (control->gimbal_INT_gyro_point != 0)
    {
        yaw_gyro_last = control->gimbal_yaw_motor.gyro;
        pitch_gyro_last = control->gimbal_pitch_motor.gyro;

        control->gimbal_yaw_motor.gyro =
            control->gimbal_INT_gyro_point[INS_GYRO_Z_ADDRESS_OFFSET];
        control->gimbal_pitch_motor.gyro =
            control->gimbal_INT_gyro_point[INS_GYRO_Y_ADDRESS_OFFSET];

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
        gimbal_feedforward_clear(&control->gimbal_yaw_motor);
        control->gimbal_yaw_motor.raw_cmd = add_yaw;
    }
    else if (control->gimbal_yaw_motor.mode == GIMBAL_MOTOR_GYRO)
    {
        gimbal_absolute_angle_limit(&control->gimbal_yaw_motor, add_yaw);
        gimbal_feedforward_track_target(&control->gimbal_yaw_motor,
                                        control->gimbal_yaw_motor.absolute_angle_set);
    }
    else if (control->gimbal_yaw_motor.mode == GIMBAL_MOTOR_ENCODE)
    {
        control->gimbal_yaw_motor.relative_angle_set +=
            add_yaw + gimbal_take_auto_aim_bias(&control->gimbal_yaw_motor);
        gimbal_feedforward_track_target(&control->gimbal_yaw_motor,
                                        control->gimbal_yaw_motor.relative_angle_set);
    }

    if (control->gimbal_pitch_motor.mode == GIMBAL_MOTOR_RAW)
    {
        gimbal_feedforward_clear(&control->gimbal_pitch_motor);
        control->gimbal_pitch_motor.raw_cmd = add_pitch;
    }
    else if (control->gimbal_pitch_motor.mode == GIMBAL_MOTOR_GYRO)
    {
        add_pitch = gimbal_feedforward_update(&control->gimbal_pitch_motor, add_pitch);
        gimbal_absolute_angle_limit(&control->gimbal_pitch_motor, add_pitch);
    }
    else if (control->gimbal_pitch_motor.mode == GIMBAL_MOTOR_ENCODE)
    {
        add_pitch = gimbal_feedforward_update(&control->gimbal_pitch_motor, add_pitch);
        control->gimbal_pitch_motor.relative_angle_set =
            gimbal_mit_clamp(control->gimbal_pitch_motor.relative_angle_set +
                                 add_pitch +
                                 gimbal_take_auto_aim_bias(&control->gimbal_pitch_motor),
                             control->gimbal_pitch_motor.min_relative_angle,
                             control->gimbal_pitch_motor.max_relative_angle);
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
