/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       gimbal_task.c
  * @brief      gimbal task entry and weak mechanism hooks
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "gimbal_task.h"
#include "gravity_comp.h"
#include "shoot_task.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

gimbal_control_t gimbal_control;

static osThreadId gimbalTaskHandle = NULL;

static void gimbal_task(void const *pvParameters);

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
    shoot_init();
    last_wake_time = xTaskGetTickCount();

    while (1)
    {
        gimbal_set_mode(&gimbal_control);
        gimbal_mode_change_control_transit(&gimbal_control);
        gimbal_feedback_update(&gimbal_control);
        gimbal_set_control(&gimbal_control);
        gimbal_control_loop(&gimbal_control);
        gravity_comp_execute(&gimbal_control);
        shoot_control_loop();
        gimbal_send_cmd(&gimbal_control);

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

__weak void gimbal_init(gimbal_control_t *control)
{
    (void)control;
}

__weak void gimbal_set_mode(gimbal_control_t *control)
{
    (void)control;
}

__weak void gimbal_feedback_update(gimbal_control_t *control)
{
    (void)control;
}

__weak void gimbal_mode_change_control_transit(gimbal_control_t *control)
{
    (void)control;
}

__weak void gimbal_set_control(gimbal_control_t *control)
{
    (void)control;
}

__weak void gimbal_control_loop(gimbal_control_t *control)
{
    (void)control;
}

__weak void gimbal_send_cmd(gimbal_control_t *control)
{
    (void)control;
}
