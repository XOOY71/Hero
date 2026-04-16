/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       gimbal_behaviour.c/h
  * @brief      minimal gimbal behaviour framework
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#ifndef GIMBAL_BEHAVIOUR_H
#define GIMBAL_BEHAVIOUR_H

#include <stdbool.h>
#include "gimbal_task.h"

typedef enum
{
    GIMBAL_ZERO_FORCE = 0,
    GIMBAL_INIT,
    GIMBAL_CALI,
    GIMBAL_ABSOLUTE_ANGLE,
    GIMBAL_RELATIVE_ANGLE,
    GIMBAL_SPIN,
    GIMBAL_MOTIONLESS,
} gimbal_behaviour_e;

extern volatile gimbal_behaviour_e gimbal_behaviour;


#ifndef GIMBAL_SPIN_KEYBOARD
#define GIMBAL_SPIN_KEYBOARD KEY_PRESSED_OFFSET_SHIFT
#endif

#ifndef GIMBAL_ZERO_KEYBOARD
#define GIMBAL_ZERO_KEYBOARD KEY_PRESSED_OFFSET_X
#endif

#ifndef GIMBAL_RELATIVE_KEYBOARD
#define GIMBAL_RELATIVE_KEYBOARD KEY_PRESSED_OFFSET_C
#endif


/**
  * @brief          云台行为状态机以及电机状态机设置
  * @param[out]     control: 云台数据指针
  * @retval         none
  */
void gimbal_behaviour_mode_set(gimbal_control_t *control);

/**
  * @brief          云台行为控制，根据不同行为采用不同控制函数
  * @param[out]     add_yaw: yaw角度增加值
  * @param[out]     add_pitch: pitch角度增加值
  * @param[in]      control: 云台数据指针
  * @retval         none
  */
void gimbal_behaviour_control_set(float *add_yaw, float *add_pitch, gimbal_control_t *control);

/**
  * @brief          云台在某些行为下，需要底盘不动
  * @param[in]      none
  * @retval         true:no move false:normal
  */
bool gimbal_cmd_to_chassis_stop(void);

/**
  * @brief          云台在某些行为下，需要射击停止
  * @param[in]      none
  * @retval         true:no move false:normal
  */
bool gimbal_cmd_to_shoot_stop(void);

/**
  * @brief          云台行为状态机设置
  * @param[in]      control: 云台数据指针
  * @retval         none
  */
void gimbal_behavour_set(gimbal_control_t *control);

void gimbal_zero_force_control(float *yaw, float *pitch, gimbal_control_t *control);
void gimbal_init_control(float *yaw, float *pitch, gimbal_control_t *control);
void gimbal_cali_control(float *yaw, float *pitch, gimbal_control_t *control);
void gimbal_absolute_angle_control(float *yaw, float *pitch, gimbal_control_t *control);
void gimbal_relative_angle_control(float *yaw, float *pitch, gimbal_control_t *control);
void gimbal_motionless_control(float *yaw, float *pitch, gimbal_control_t *control);
void gimbal_spin_control(float *yaw, float *pitch, gimbal_control_t *control);

#endif
