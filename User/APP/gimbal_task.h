/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       gimbal_task.c/h
  * @brief      minimal gimbal control framework
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#ifndef GIMBAL_TASK_H
#define GIMBAL_TASK_H

#include <stdint.h>
#include <stdbool.h>

#include "project_config.h"
#include "remote_control.h"
#include "pid.h"
#include "gravity_comp.h"

typedef enum
{
    GIMBAL_MOTOR_RAW = 0,
    GIMBAL_MOTOR_GYRO,
    GIMBAL_MOTOR_ENCODE,
} gimbal_motor_mode_e;

#ifndef GIMBAL_MOTOR_ENCONDE
#define GIMBAL_MOTOR_ENCONDE GIMBAL_MOTOR_ENCODE
#endif

typedef pid_type_def gimbal_pid_t;

typedef struct
{
    gimbal_motor_mode_e mode;        // 当前电机控制模式
    gimbal_motor_mode_e last_mode;   // 上一次电机控制模式，用于模式切换过渡

    float relative_angle;            // 相对角度反馈值
    float relative_angle_set;        // 相对角度目标值
    float relative_angle_last;       // 上一次相对角度反馈值
    float relative_speed;            // 相对角编码器差分速度
    uint8_t relative_speed_update_init; // 相对角速度差分初始化标志

    float absolute_angle;            // 绝对角度反馈值
    float absolute_angle_set;        // 绝对角度目标值

    float angle_offset;              // 上电/初始化时记录的角度零偏
    uint8_t angle_offset_init;       // 角度零偏是否已初始化

    float gyro;                      // 角速度反馈值
    float gyro_set;                  // 角速度目标值
    float gyro_last;                 // 上一次角速度反馈值
    float gyro_accel;                // 角加速度反馈值，由角速度差分得到
    uint8_t gyro_update_init;        // 角速度差分初始化标志

    float ref_vel;                   // 前馈轨迹参考速度
    float ref_vel_last;              // 上一次前馈轨迹参考速度
    float ref_accel;                 // 前馈轨迹参考加速度
    float rc_ref_vel;
    float rc_ref_vel_last;
    float rc_ref_accel;
    float rc_ref_target_last;
    uint8_t rc_ref_target_init;
    float auto_ref_vel;
    float auto_ref_vel_last;
    float auto_ref_accel;
    float auto_ref_target_last;
    uint8_t auto_ref_target_init;
    float pid_torque;                // PID 反馈输出力矩
    float rc_pid_torque;
    float auto_pid_torque;
    float ff_torque;                 // 惯量前馈输出力矩
    float rc_ff_torque;
    float auto_ff_torque;
    float static_friction_comp;      // 静摩擦补偿输出力矩
    float rc_static_friction_comp;
    float auto_static_friction_comp;
    float inertia_kgm2;              // 转动惯量参数 J，单位 kg*m^2

    float raw_cmd;                   // RAW 模式下直接输出命令
    float output;                    // 控制器最终输出值
    float current_set;               // 目标电流/目标力矩命令
    float rc_current_set;
    float auto_current_set;
    float given_current;             // 实际发送给电机的电流/力矩命令

    float max_relative_angle;        // 相对角度上限
    float min_relative_angle;        // 相对角度下限
    uint16_t offset_ecd;             // 编码器零点偏移

    float rc_relative_angle_set;
    float rc_absolute_angle_set;
    float auto_relative_angle_set;
    float auto_absolute_angle_set;
    uint8_t rc_control_enable;
    uint8_t auto_control_enable;
    gimbal_pid_t rc_absolute_angle_pid;
    gimbal_pid_t rc_relative_angle_pid;
    gimbal_pid_t auto_absolute_angle_pid;
    gimbal_pid_t auto_relative_angle_pid;
} gimbal_motor_t;

typedef struct
{
    float max_yaw;
    float min_yaw;
    float max_pitch;
    float min_pitch;
    uint16_t max_yaw_ecd;
    uint16_t min_yaw_ecd;
    uint16_t max_pitch_ecd;
    uint16_t min_pitch_ecd;
    uint8_t step;
} gimbal_step_cali_t;

typedef struct gimbal_control_t
{
    const RC_ctrl_t *gimbal_rc_ctrl;
    const float *gimbal_INT_angle_point;
    const float *gimbal_INT_gyro_point;
    const float *gimbal_INT_accel_point;
    gimbal_motor_t gimbal_yaw_motor;
    gimbal_motor_t gimbal_pitch_motor;
    gimbal_step_cali_t gimbal_cali;
    gravity_comp_t gimbal_pitch_gravity_comp;
} gimbal_control_t;

extern gimbal_control_t gimbal_control;
extern float yaw_can_set_current;
extern float pitch_can_set_current;
extern int16_t shoot_can_set_current;

void GimbalTask_Init(void);

/**
  * @brief          返回yaw 电机数据指针
  * @param[in]      none
  * @retval         yaw电机指针
  */
const gimbal_motor_t *get_yaw_motor_point(void);

/**
  * @brief          返回pitch 电机数据指针
  * @param[in]      none
  * @retval         pitch电机指针
  */
const gimbal_motor_t *get_pitch_motor_point(void);

/**
  * @brief          云台控制模式:GIMBAL_MOTOR_GYRO，更新绝对角目标
  * @param[out]     motor:yaw电机或者pitch电机
  * @param[in]      add:角度增量
  * @retval         none
  */
void gimbal_absolute_angle_limit(gimbal_motor_t *motor, float add);

/**
  * @brief          云台控制模式:GIMBAL_MOTOR_GYRO
  * @param[out]     motor:yaw电机或者pitch电机
  * @retval         none
  */
void gimbal_motor_absolute_angle_control(gimbal_motor_t *motor);

/**
  * @brief          云台控制模式:GIMBAL_MOTOR_ENCODE
  * @param[out]     motor:yaw电机或者pitch电机
  * @retval         none
  */
void gimbal_motor_relative_angle_control(gimbal_motor_t *motor);

/**
  * @brief          云台控制模式:GIMBAL_MOTOR_RAW
  * @param[out]     motor:yaw电机或者pitch电机
  * @retval         none
  */
void gimbal_motor_raw_angle_control(gimbal_motor_t *motor);

/**
  * @brief          简化PID初始化
  * @param[out]     pid:PID结构体指针
  * @param[in]      kp,ki,kd,max_out,max_iout: PID参数
  * @retval         none
  */
void gimbal_pid_init(gimbal_pid_t *pid, float kp, float ki, float kd, float max_out, float max_iout);

/**
  * @brief          简化PID清零
  * @param[out]     pid:PID结构体指针
  * @retval         none
  */
void gimbal_pid_clear(gimbal_pid_t *pid);

/**
  * @brief          简化PID计算接口
  * @param[out]     pid:PID结构体指针
  * @param[in]      get,set,error_delta
  * @retval         PID输出
  */
float gimbal_pid_calc(gimbal_pid_t *pid, float get, float set, float error_delta);

/* platform hooks */
void gimbal_init(gimbal_control_t *control);
void gimbal_set_mode(gimbal_control_t *control);
void gimbal_feedback_update(gimbal_control_t *control);
void gimbal_mode_change_control_transit(gimbal_control_t *control);
void gimbal_set_control(gimbal_control_t *control);
void gimbal_control_loop(gimbal_control_t *control);
void gimbal_send_cmd(gimbal_control_t *control);
#endif
