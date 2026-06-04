#ifndef CHASSIS_TASK_H
#define CHASSIS_TASK_H

#include "bsp_fdcan.h"
#include "gimbal_task.h"
#include "hwt_imu.h"
#include "pid.h"
#include "project_config.h"
#include "remote_control.h"
#include "struct_typedef.h"
#include "user_lib.h"

#define rc_deadband_limit(input, output, dealine)       \
{                                                       \
    if ((input) > (dealine) || (input) < -(dealine))    \
    {                                                   \
        (output) = (input);                             \
    }                                                   \
    else                                                \
    {                                                   \
        (output) = 0;                                   \
    }                                                   \
}

typedef enum
{
    CHASSIS_VECTOR_NO_MOVE = 0,
    CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW,
    CHASSIS_VECTOR_YAW_HOLD,
    CHASSIS_VECTOR_SPIN,
    CHASSIS_VECTOR_RETURN,
} chassis_mode_e;

typedef struct
{
    const motor_measure_t *chassis_motor_measure;
    fp32 accel;
    fp32 speed;
    fp32 speed_rad_s;
    fp32 speed_set;
    fp32 angle;
    fp32 angle_set;
    int16_t give_current;
    fp32 given_current_a;
} chassis_motor_t;

typedef struct
{
    fp32 now[CHASSIS_MODULE_NUM];
    fp32 last[CHASSIS_MODULE_NUM];
    fp32 initial[CHASSIS_MODULE_NUM];
} wheel_angle_offset_t;

typedef struct
{
    const RC_ctrl_t *chassis_RC;
    const gimbal_motor_t *chassis_yaw_motor;
    const gimbal_motor_t *chassis_pitch_motor;
    const fp32 *chassis_INS_angle;

    chassis_mode_e chassis_mode;
    chassis_mode_e last_chassis_mode;

    chassis_motor_t chassis_3508[CHASSIS_MODULE_NUM];
    wheel_angle_offset_t wheel_angle_offset;

    fp32 model_3508_out[CHASSIS_MODULE_NUM];
    fp32 model_accel[CHASSIS_MODULE_NUM];
    fp32 model_last_speed_set[CHASSIS_MODULE_NUM];
    fp32 speed_pi_iout[CHASSIS_MODULE_NUM];
    fp32 speed_pid_last_error[CHASSIS_MODULE_NUM];
    uint8_t stop_brake_active[CHASSIS_MODULE_NUM];
    fp32 motor_current_limit_a[CHASSIS_MODULE_NUM];
    fp32 last_motor_current_limit_a[CHASSIS_MODULE_NUM];

    pid_type_def chassis_angle_pid;
    pid_type_def chas_return_pid;

    uint8_t chassis_return_flag;
    chassis_mode_e chassis_return_record;

    first_order_filter_type_t chassis_cmd_slow_set_vx;
    first_order_filter_type_t chassis_cmd_slow_set_vy;

    fp32 vx;
    fp32 vy;
    fp32 wz;
    fp32 vx_set;
    fp32 vy_set;
    fp32 wz_set;
    fp32 last_vx_set;
    fp32 last_vy_set;
    fp32 last_wz_set;
    fp32 vx_plan;
    fp32 vy_plan;
    fp32 wz_plan;
    fp32 vx_plan_accel;
    fp32 vy_plan_accel;
    fp32 wz_plan_accel;
    fp32 return_wz_set;
    fp32 chassis_relative_angle;
    fp32 chassis_relative_angle_vel;
    fp32 chassis_relative_angle_target;
    fp32 chassis_relative_angle_set;
    fp32 chassis_relative_angle_set_vel;
    fp32 chassis_yaw_target;
    fp32 chassis_yaw_set;
    fp32 chassis_yaw_set_vel;
    fp32 last_vx_plan_ff;
    fp32 last_vy_plan_ff;
    fp32 last_wz_plan_ff;
    fp32 body_ff_ax;
    fp32 body_ff_ay;
    fp32 body_ff_alpha;
    fp32 body_ff_current[CHASSIS_MODULE_NUM];
    uint8_t body_ff_init;
    fp32 gimbal_radian_of_ecd;
    fp32 chassis_yaw_rate;

    uint8_t gimbal_behaviour;
    uint8_t gimbal_shoot_mode;

    fp32 vx_max_speed;
    fp32 vx_min_speed;
    fp32 vy_max_speed;
    fp32 vy_min_speed;
    fp32 chassis_yaw;
    fp32 chassis_pitch;
    fp32 chassis_roll;
} chassis_move_t;

extern void chassis_task(void const *pvParameters);
void chassis_init(chassis_move_t *chassis_move_init);
void chassis_set_mode(chassis_move_t *chassis_move_mode);
void chassis_mode_change_control_transit(chassis_move_t *chassis_move_transit);
void chassis_feedback_update(chassis_move_t *chassis_move_update);
void chassis_set_contorl(chassis_move_t *chassis_move_control);
void chassis_control_loop(chassis_move_t *chassis_move_control_loop);
void chassis_send_cmd(chassis_move_t *chassis_move_send);

extern chassis_move_t chassis_move;
extern fp32 yaw_set;

#endif
