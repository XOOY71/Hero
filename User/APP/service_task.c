/**
  ****************************(C) COPYRIGHT 2026****************************
  * @file       service_task.c
  * @brief      service task, used for some ordinary jobs
  ****************************(C) COPYRIGHT 2026****************************
  */

#include "service_task.h"
#include "cmsis_os.h"
#include "safewarning.h"
#include "hwt_imu.h"
#include "vofa.h"
service_control_t service_control;

/**/
static osThreadId serviceTaskHandle = NULL;
static void service_task(void const *pvParameters);
void ServiceTask_Init(void)
{
    osThreadDef(serviceTask, service_task, osPriorityLow, 0, 256);
    serviceTaskHandle = osThreadCreate(osThread(serviceTask), NULL);
}
/**/


/**
  * @brief          模块私有任务主流程
  * @param[in]      none
  * @retval         none
  */
static void service_task(void const *pvParameters)
{
    (void)pvParameters;

    vTaskDelay(SERVICE_TASK_INIT_TIME);
		Beep_Init();
		hwt_imu_init();
		Beep_Play(BEEP_POWER_ON);
    service_control.service_time = 0;

    while (1)
    {
			
      service_control.service_time += SERVICE_CONTROL_TIME;
			ws2812_task();
			Beep_Task();
			VOFA_ServiceSend();

			vTaskDelay(SERVICE_CONTROL_TIME);
    }
}
