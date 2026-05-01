/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       yaw_pitch_direct.c/h
  * @brief      minimal yaw-pitch direct framework
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#ifndef YAW_PITCH_DIRECT_H
#define YAW_PITCH_DIRECT_H

#include "gimbal_task.h"
#include "gimbal_behaviour.h"

/**
  * @brief          初始化gimbal_control变量
  * @param[out]     control: 云台控制结构体指针
  * @retval         none
  */
void gimbal_init(gimbal_control_t *control);

/**
  * @brief          设置云台控制模式
  * @param[out]     control: 云台控制结构体指针
  * @retval         none
  */
void gimbal_set_mode(gimbal_control_t *control);

/**
  * @brief          云台反馈更新
  * @param[out]     control: 云台控制结构体指针
  * @retval         none
  */
void gimbal_feedback_update(gimbal_control_t *control);

/**
  * @brief          控制模式切换时的过渡处理
  * @param[out]     control: 云台控制结构体指针
  * @retval         none
  */
void gimbal_mode_change_control_transit(gimbal_control_t *control);

/**
  * @brief          设置云台控制设定值
  * @param[out]     control: 云台控制结构体指针
  * @retval         none
  */
void gimbal_set_control(gimbal_control_t *control);

/**
  * @brief          云台控制环
  * @param[out]     control: 云台控制结构体指针
  * @retval         none
  */
void gimbal_control_loop(gimbal_control_t *control);

/**
  * @brief          发送控制命令
  * @param[out]     control: 云台控制结构体指针
  * @retval         none
  */
void gimbal_send_cmd(gimbal_control_t *control);


void gimbal_test(void);
#endif
