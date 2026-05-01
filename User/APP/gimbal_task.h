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
    gimbal_motor_mode_e mode;
    gimbal_motor_mode_e last_mode;

    float relative_angle;
    float relative_angle_set;

    float absolute_angle;
    float absolute_angle_set;

    float angle_offset;
    uint8_t angle_offset_init;

    float gyro;
    float gyro_set;

    float raw_cmd;
    float output;
    float current_set;
    int16_t given_current;

    float max_relative_angle;
    float min_relative_angle;
    uint16_t offset_ecd;

    gimbal_pid_t absolute_angle_pid;
    gimbal_pid_t relative_angle_pid;
    gimbal_pid_t gyro_pid;
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

typedef struct
{
    const RC_ctrl_t *gimbal_rc_ctrl;
    const float *gimbal_INT_angle_point;
    const float *gimbal_INT_gyro_point;
    gimbal_motor_t gimbal_yaw_motor;
    gimbal_motor_t gimbal_pitch_motor;
    gimbal_step_cali_t gimbal_cali;
} gimbal_control_t;

extern gimbal_control_t gimbal_control;
extern int16_t yaw_can_set_current;
extern int16_t pitch_can_set_current;
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
  * @brief          云台控制模式:GIMBAL_MOTOR_ENCODE，更新相对角目标
  * @param[out]     motor:yaw电机或者pitch电机
  * @param[in]      add:角度增量
  * @retval         none
  */
void gimbal_relative_angle_limit(gimbal_motor_t *motor, float add);

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
  * @param[in]      kp,ki,kd: PID参数
  * @retval         none
  */
void gimbal_pid_init(gimbal_pid_t *pid, float kp, float ki, float kd);

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
void gimbal_test(void);
#endif
