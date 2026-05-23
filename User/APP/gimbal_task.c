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
#include "hwt_imu.h"

gimbal_control_t gimbal_control;

#ifndef GIMBAL_MIT_FEEDBACK_INIT_DELAY
#define GIMBAL_MIT_FEEDBACK_INIT_DELAY 100U
#endif

#ifndef GIMBAL_YAW_MIT_INDEX
#define GIMBAL_YAW_MIT_INDEX 0U
#endif

#ifndef GIMBAL_STATIC_FRICTION_COMP
#define GIMBAL_STATIC_FRICTION_COMP 0.28f
#endif
#ifndef GIMBAL_STATIC_FRICTION_DEADBAND
#define GIMBAL_STATIC_FRICTION_DEADBAND 0.002f
#endif
#ifndef GIMBAL_STATIC_FRICTION_FULLBAND
#define GIMBAL_STATIC_FRICTION_FULLBAND 0.006f
#endif
#ifndef GIMBAL_STATIC_FRICTION_FILTER_ALPHA
#define GIMBAL_STATIC_FRICTION_FILTER_ALPHA 0.2f
#endif

static osThreadId gimbalTaskHandle = NULL;

static void gimbal_task(void const *pvParameters);
#define GIMBAL_PI PI
static float gimbal_wrap_angle(float angle);
static float gimbal_take_auto_aim_bias(gimbal_motor_t *motor);
static float gimbal_clamp(float value, float min_value, float max_value);
static float gimbal_calc_static_friction_comp_raw(float angle_error);
static float gimbal_update_static_friction_comp(float *friction_comp, float angle_error);
static float gimbal_float_to_torque_cmd(float output);
static float gimbal_calc_feedforward(gimbal_motor_t *motor, float ref_vel, float ref_accel);
static float gimbal_calc_feedback_torque(gimbal_motor_t *motor, gimbal_pid_t *angle_pid, float angle_get, float angle_set, float *friction_comp);
static float gimbal_calc_angle_speed_torque(gimbal_motor_t *motor, gimbal_pid_t *pid, float angle_error, float ref_vel, float *friction_comp);
static void gimbal_commit_motor_output(gimbal_motor_t *motor);
void gimbal_vofa_send_fric(void);
void gimbal_vofa_send_yaw(void);
void gimbal_vofa_send_pitch(void);
void gimbal_vofa_send_yaw_pitch_half(void);
void gimbal_vofa_send_strum(void);

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

        gimbal_vofa_send_strum();

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

    if (motor == 0)
    {
        return;
    }

    motor->rc_pid_torque = 0.0f;
    motor->auto_pid_torque = 0.0f;
    motor->rc_ff_torque = 0.0f;
    motor->auto_ff_torque = 0.0f;
    motor->rc_current_set = 0.0f;
    motor->auto_current_set = 0.0f;

    if (motor == &gimbal_control.gimbal_yaw_motor)
    {
        angle_get = motor->absolute_angle;
        if (motor->rc_control_enable != 0u)
        {
            motor->rc_pid_torque =
                gimbal_calc_angle_speed_torque(motor,
                                               &motor->rc_absolute_angle_pid,
                                               gimbal_wrap_angle(motor->rc_absolute_angle_set - angle_get),
                                               motor->rc_ref_vel,
                                               &motor->rc_static_friction_comp);
            motor->rc_ff_torque =
                gimbal_calc_feedforward(motor, motor->rc_ref_vel, motor->rc_ref_accel);
            motor->rc_current_set = motor->rc_pid_torque + motor->rc_ff_torque;
        }
        else
        {
            (void)gimbal_calc_angle_speed_torque(motor,
                                                 &motor->rc_absolute_angle_pid,
                                                 gimbal_wrap_angle(motor->rc_absolute_angle_set - angle_get),
                                                 motor->rc_ref_vel,
                                                 &motor->rc_static_friction_comp);
        }

        if (motor->auto_control_enable != 0u)
        {
            motor->auto_pid_torque =
                gimbal_calc_angle_speed_torque(motor,
                                               &motor->auto_absolute_angle_pid,
                                               gimbal_wrap_angle(motor->auto_absolute_angle_set - angle_get),
                                               motor->auto_ref_vel,
                                               &motor->auto_static_friction_comp);
            motor->auto_ff_torque =
                gimbal_calc_feedforward(motor, motor->auto_ref_vel, motor->auto_ref_accel);
            motor->auto_current_set = motor->auto_pid_torque + motor->auto_ff_torque;
        }
        else
        {
            (void)gimbal_calc_angle_speed_torque(motor,
                                                 &motor->auto_absolute_angle_pid,
                                                 gimbal_wrap_angle(motor->auto_absolute_angle_set - angle_get),
                                                 motor->auto_ref_vel,
                                                 &motor->auto_static_friction_comp);
        }
    }
    else
    {
        angle_get = gimbal_wrap_angle(motor->absolute_angle);
        if (motor->rc_control_enable != 0u)
        {
            motor->rc_pid_torque =
                gimbal_calc_feedback_torque(motor,
                                            &motor->rc_absolute_angle_pid,
                                            angle_get,
                                            gimbal_wrap_angle(motor->rc_absolute_angle_set),
                                            &motor->rc_static_friction_comp);
            motor->rc_ff_torque =
                gimbal_calc_feedforward(motor, motor->rc_ref_vel, motor->rc_ref_accel);
            motor->rc_current_set = motor->rc_pid_torque + motor->rc_ff_torque;
        }
        else
        {
            (void)gimbal_calc_feedback_torque(motor,
                                              &motor->rc_absolute_angle_pid,
                                              angle_get,
                                              gimbal_wrap_angle(motor->rc_absolute_angle_set),
                                              &motor->rc_static_friction_comp);
        }

        if (motor->auto_control_enable != 0u)
        {
            motor->auto_pid_torque =
                gimbal_calc_feedback_torque(motor,
                                            &motor->auto_absolute_angle_pid,
                                            angle_get,
                                            gimbal_wrap_angle(motor->auto_absolute_angle_set),
                                            &motor->auto_static_friction_comp);
            motor->auto_ff_torque =
                gimbal_calc_feedforward(motor, motor->auto_ref_vel, motor->auto_ref_accel);
            motor->auto_current_set = motor->auto_pid_torque + motor->auto_ff_torque;
        }
        else
        {
            (void)gimbal_calc_feedback_torque(motor,
                                              &motor->auto_absolute_angle_pid,
                                              angle_get,
                                              gimbal_wrap_angle(motor->auto_absolute_angle_set),
                                              &motor->auto_static_friction_comp);
        }
    }

    gimbal_commit_motor_output(motor);
}

void gimbal_motor_relative_angle_control(gimbal_motor_t *motor)
{
    if (motor == 0)
    {
        return;
    }

    motor->rc_pid_torque = 0.0f;
    motor->auto_pid_torque = 0.0f;
    motor->rc_ff_torque = 0.0f;
    motor->auto_ff_torque = 0.0f;
    motor->rc_current_set = 0.0f;
    motor->auto_current_set = 0.0f;

    if (motor == &gimbal_control.gimbal_yaw_motor)
    {
        if (motor->rc_control_enable != 0u)
        {
            motor->rc_pid_torque =
                gimbal_calc_feedback_torque(motor,
                                            &motor->rc_relative_angle_pid,
                                            0.0f,
                                            gimbal_wrap_angle(motor->rc_relative_angle_set - motor->relative_angle),
                                            &motor->rc_static_friction_comp);
            motor->rc_ff_torque =
                gimbal_calc_feedforward(motor, motor->rc_ref_vel, motor->rc_ref_accel);
            motor->rc_current_set = motor->rc_pid_torque + motor->rc_ff_torque;
        }
        else
        {
            (void)gimbal_calc_feedback_torque(motor,
                                              &motor->rc_relative_angle_pid,
                                              0.0f,
                                              gimbal_wrap_angle(motor->rc_relative_angle_set - motor->relative_angle),
                                              &motor->rc_static_friction_comp);
        }

        if (motor->auto_control_enable != 0u)
        {
            motor->auto_pid_torque =
                gimbal_calc_feedback_torque(motor,
                                            &motor->auto_relative_angle_pid,
                                            0.0f,
                                            gimbal_wrap_angle(motor->auto_relative_angle_set - motor->relative_angle),
                                            &motor->auto_static_friction_comp);
            motor->auto_ff_torque =
                gimbal_calc_feedforward(motor, motor->auto_ref_vel, motor->auto_ref_accel);
            motor->auto_current_set = motor->auto_pid_torque + motor->auto_ff_torque;
        }
        else
        {
            (void)gimbal_calc_feedback_torque(motor,
                                              &motor->auto_relative_angle_pid,
                                              0.0f,
                                              gimbal_wrap_angle(motor->auto_relative_angle_set - motor->relative_angle),
                                              &motor->auto_static_friction_comp);
        }
    }
    else
    {
        if (motor->rc_control_enable != 0u)
        {
            motor->rc_pid_torque =
                gimbal_calc_angle_speed_torque(motor,
                                               &motor->rc_relative_angle_pid,
                                               motor->rc_relative_angle_set - motor->relative_angle,
                                               motor->rc_ref_vel,
                                               &motor->rc_static_friction_comp);
            motor->rc_ff_torque =
                gimbal_calc_feedforward(motor, motor->rc_ref_vel, motor->rc_ref_accel);
            motor->rc_current_set = motor->rc_pid_torque + motor->rc_ff_torque;
        }
        else
        {
            (void)gimbal_calc_angle_speed_torque(motor,
                                                 &motor->rc_relative_angle_pid,
                                                 motor->rc_relative_angle_set - motor->relative_angle,
                                                 motor->rc_ref_vel,
                                                 &motor->rc_static_friction_comp);
        }

        if (motor->auto_control_enable != 0u)
        {
            motor->auto_pid_torque =
                gimbal_calc_angle_speed_torque(motor,
                                               &motor->auto_relative_angle_pid,
                                               motor->auto_relative_angle_set - motor->relative_angle,
                                               motor->auto_ref_vel,
                                               &motor->auto_static_friction_comp);
            motor->auto_ff_torque =
                gimbal_calc_feedforward(motor, motor->auto_ref_vel, motor->auto_ref_accel);
            motor->auto_current_set = motor->auto_pid_torque + motor->auto_ff_torque;
        }
        else
        {
            (void)gimbal_calc_angle_speed_torque(motor,
                                                 &motor->auto_relative_angle_pid,
                                                 motor->auto_relative_angle_set - motor->relative_angle,
                                                 motor->auto_ref_vel,
                                                 &motor->auto_static_friction_comp);
        }
    }

    gimbal_commit_motor_output(motor);
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
    motor->static_friction_comp = 0.0f;
    motor->rc_pid_torque = 0.0f;
    motor->auto_pid_torque = 0.0f;
    motor->rc_ff_torque = 0.0f;
    motor->auto_ff_torque = 0.0f;
    motor->rc_current_set = 0.0f;
    motor->auto_current_set = 0.0f;
    motor->rc_static_friction_comp = 0.0f;
    motor->auto_static_friction_comp = 0.0f;
    motor->rc_control_enable = 0u;
    motor->auto_control_enable = 0u;
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

static float gimbal_calc_static_friction_comp_raw(float angle_error)
{
    float abs_error;
    float comp_scale;

    abs_error = fabsf(angle_error);
    if (abs_error <= GIMBAL_STATIC_FRICTION_DEADBAND)
    {
        return 0.0f;
    }

    if (GIMBAL_STATIC_FRICTION_FULLBAND <= GIMBAL_STATIC_FRICTION_DEADBAND ||
        abs_error >= GIMBAL_STATIC_FRICTION_FULLBAND)
    {
        comp_scale = 1.0f;
    }
    else
    {
        comp_scale =
            (abs_error - GIMBAL_STATIC_FRICTION_DEADBAND) /
            (GIMBAL_STATIC_FRICTION_FULLBAND - GIMBAL_STATIC_FRICTION_DEADBAND);
    }

    if (angle_error > 0.0f)
    {
        return GIMBAL_STATIC_FRICTION_COMP * comp_scale;
    }
    if (angle_error < 0.0f)
    {
        return -GIMBAL_STATIC_FRICTION_COMP * comp_scale;
    }
    return 0.0f;
}

static float gimbal_update_static_friction_comp(float *friction_comp, float angle_error)
{
    float alpha;
    float comp_cmd;

    comp_cmd = gimbal_calc_static_friction_comp_raw(angle_error);
    if (friction_comp == 0)
    {
        return comp_cmd;
    }

    alpha = gimbal_clamp(GIMBAL_STATIC_FRICTION_FILTER_ALPHA, 0.0f, 1.0f);
    *friction_comp += alpha * (comp_cmd - *friction_comp);

    return *friction_comp;
}

static float gimbal_float_to_torque_cmd(float output)
{
    return gimbal_clamp(output, T_MIN, T_MAX);
}

static float gimbal_calc_feedforward(gimbal_motor_t *motor, float ref_vel, float ref_accel)
{
    float velocity_torque = 0.0f;

    if (motor == 0)
    {
        return 0.0f;
    }

    if (motor == &gimbal_control.gimbal_pitch_motor)
    {
        velocity_torque = PITCH_VELOCITY_FF_GAIN * ref_vel;
    }

    return velocity_torque + motor->inertia_kgm2 * ref_accel;
}

static float gimbal_calc_feedback_torque(gimbal_motor_t *motor, gimbal_pid_t *angle_pid, float angle_get, float angle_set, float *friction_comp)
{
    float angle_torque;
    float angle_error;

    if (motor == 0 || angle_pid == 0)
    {
        return 0.0f;
    }

    motor->gyro_set = motor->ref_vel;
    angle_error = angle_set - angle_get;
    angle_torque = gimbal_pid_calc(angle_pid, angle_get, angle_set, 0.0f);
    return gimbal_clamp(angle_torque + gimbal_update_static_friction_comp(friction_comp, angle_error),
                        -angle_pid->max_out,
                        angle_pid->max_out);
}

static float gimbal_calc_angle_speed_torque(gimbal_motor_t *motor, gimbal_pid_t *pid, float angle_error, float ref_vel, float *friction_comp)
{
    float speed_error;
    float output;

    if (motor == 0 || pid == 0)
    {
        return 0.0f;
    }

    speed_error = ref_vel - motor->gyro;

    pid->set = angle_error;
    pid->fdb = 0.0f;
    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->error[0] = angle_error;
    pid->Pout = pid->Kp * angle_error;
    pid->Iout = 0.0f;
    pid->Dout = pid->Kd * speed_error;

    output = pid->Pout + pid->Dout + gimbal_update_static_friction_comp(friction_comp, angle_error);
    output = gimbal_clamp(output, -pid->max_out, pid->max_out);
    pid->out = output;

    return output;
}

static void gimbal_commit_motor_output(gimbal_motor_t *motor)
{
    if (motor == 0)
    {
        return;
    }

    motor->pid_torque = motor->rc_pid_torque + motor->auto_pid_torque;
    motor->ff_torque = motor->rc_ff_torque + motor->auto_ff_torque;
    motor->static_friction_comp =
        motor->rc_static_friction_comp + motor->auto_static_friction_comp;
    motor->ref_vel =
        ((motor->rc_control_enable != 0u) ? motor->rc_ref_vel : 0.0f) +
        ((motor->auto_control_enable != 0u) ? motor->auto_ref_vel : 0.0f);
    motor->ref_accel =
        ((motor->rc_control_enable != 0u) ? motor->rc_ref_accel : 0.0f) +
        ((motor->auto_control_enable != 0u) ? motor->auto_ref_accel : 0.0f);
    motor->gyro_set = motor->ref_vel;
    motor->current_set = motor->rc_current_set + motor->auto_current_set;
    motor->output = motor->current_set;
    motor->given_current = gimbal_float_to_torque_cmd(motor->output);
}

void gimbal_vofa_send_fric(void)
{
    float current_avg;

    current_avg = (shoot_task_control.fric1.give_current_a +
                   shoot_task_control.fric2.give_current_a +
                   shoot_task_control.fric3.give_current_a) / 3.0f;

    VOFA_Send6(shoot_task_control.fric1.speed_rpm,
               shoot_task_control.fric2.speed_rpm,
               shoot_task_control.fric3.speed_rpm,
               current_avg,
               shoot_task_control.bullet_speed_min_avg_rpm,
               shoot_task_control.estimated_bullet_speed_mps);
}

void gimbal_vofa_send_yaw(void)
{
    const gimbal_motor_t *yaw = &gimbal_control.gimbal_yaw_motor;

    VOFA_Send6(yaw->relative_angle_set,
               yaw->relative_angle,
               yaw->absolute_angle_set,
               yaw->absolute_angle,
               yaw->gyro,
               yaw->current_set);
}

void gimbal_vofa_send_pitch(void)
{
    const gimbal_motor_t *pitch = &gimbal_control.gimbal_pitch_motor;

    VOFA_Send6(pitch->relative_angle_set,
               pitch->relative_angle,
               pitch->absolute_angle_set,
               pitch->absolute_angle,
               pitch->gyro,
               pitch->current_set);
}

void gimbal_vofa_send_yaw_pitch_half(void)
{
    const gimbal_motor_t *yaw = &gimbal_control.gimbal_yaw_motor;
    const gimbal_motor_t *pitch = &gimbal_control.gimbal_pitch_motor;

    VOFA_Send6(yaw->relative_angle_set,
               yaw->relative_angle,
               yaw->static_friction_comp,
               pitch->relative_angle_set,
               pitch->relative_angle,
               pitch->static_friction_comp);
}

void gimbal_vofa_send_strum(void)
{
    const MITMeasure_t *strum = &MIT_MOTOR_MEASURE[SHOOT_STRUM_MIT_INDEX];

    VOFA_Send6(strum->fdb.pos,
               strum->set.POS,
               strum->fdb.vel,
               strum->fdb.tor,
               strum->set.TOR,
               strum->fdb.t_motor);
}
