#include "chassis_task.h"
#include "cmsis_os.h"

#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t chassis_high_water;
#endif

chassis_move_t chassis_move;

void chassis_task(void const *pvParameters)
{
	vTaskDelay(CHASSIS_TASK_INIT_TIME);
	chassis_init(&chassis_move);

	while (1)
	{
		chassis_set_mode(&chassis_move);
		chassis_mode_change_control_transit(&chassis_move);
		chassis_feedback_update(&chassis_move);
		chassis_set_contorl(&chassis_move);
		chassis_control_loop(&chassis_move);
		chassis_send_cmd(&chassis_move);

		osDelay(CHASSIS_CONTROL_TIME_MS);

#if INCLUDE_uxTaskGetStackHighWaterMark
		chassis_high_water = uxTaskGetStackHighWaterMark(NULL);
#endif
	}
}

__weak void chassis_init(chassis_move_t *chassis_move_init)
{
	(void)chassis_move_init;
}

__weak void chassis_set_mode(chassis_move_t *chassis_move_mode)
{
	(void)chassis_move_mode;
}

__weak void chassis_mode_change_control_transit(chassis_move_t *chassis_move_transit)
{
	(void)chassis_move_transit;
}

__weak void chassis_feedback_update(chassis_move_t *chassis_move_update)
{
	(void)chassis_move_update;
}

__weak void chassis_set_contorl(chassis_move_t *chassis_move_control)
{
	(void)chassis_move_control;
}

__weak void chassis_control_loop(chassis_move_t *chassis_move_control_loop)
{
	(void)chassis_move_control_loop;
}

__weak void chassis_send_cmd(chassis_move_t *chassis_move_send)
{
	(void)chassis_move_send;
}
