#include "shoot_task.h"

__weak void shoot_init(void)
{
}

__weak void shoot_control_loop(void)
{
}

void shoot_task_init(void)
{
    shoot_init();
}

void shoot_task_loop(void)
{
    shoot_control_loop();
}
