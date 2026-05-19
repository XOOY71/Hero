#include "detect_task.h"

#include "cmsis_os.h"
#include "remote_control.h"

static error_t error_list[ERROR_LIST_LENGHT + 1];
static uint8_t detect_inited = 0U;

static void detect_init(uint32_t time)
{
    static const uint16_t set_item[ERROR_LIST_LENGHT][3] =
    {
        {30, 0, 17},
        {10, 0, 12},
        {10, 0, 11},
        {10, 0, 10},
        {10, 0, 9},
        {2, 3, 16},
        {2, 3, 15},
        {2, 3, 14},
        {2, 3, 13},
        {10, 10, 8},
        {2, 3, 4},
        {2, 3, 3},
        {2, 3, 7},
        {5, 5, 7},
        {40, 200, 7},
        {100, 100, 5},
        {10, 10, 7},
        {100, 100, 1},
    };

    for (uint8_t i = 0U; i < ERROR_LIST_LENGHT; i++)
    {
        error_list[i].set_offline_time = set_item[i][0];
        error_list[i].set_online_time = set_item[i][1];
        error_list[i].priority = set_item[i][2];
        error_list[i].enable = 1U;
        error_list[i].error_exist = 1U;
        error_list[i].is_lost = 1U;
        error_list[i].data_is_error = 0U;
        error_list[i].frequency = 0.0f;
        error_list[i].new_time = time;
        error_list[i].last_time = time;
        error_list[i].lost_time = time;
        error_list[i].work_time = time;
    }

    error_list[ERROR_LIST_LENGHT].enable = 0U;
    detect_inited = 1U;
}

void detect_task(void const *pvParameters)
{
    (void)pvParameters;

    detect_init(xTaskGetTickCount());
    vTaskDelay(DETECT_TASK_INIT_TIME);

    while (1)
    {
        uint32_t now = xTaskGetTickCount();

        for (uint8_t i = 0U; i < ERROR_LIST_LENGHT; i++)
        {
            if (error_list[i].enable == 0U)
            {
                continue;
            }

            if ((i == DBUS_TOE) && (rc_ctrl.last_fdb != 0U))
            {
                error_list[i].new_time = rc_ctrl.last_fdb;
            }

            if ((now - error_list[i].new_time) > error_list[i].set_offline_time)
            {
                error_list[i].is_lost = 1U;
                error_list[i].error_exist = 1U;
                error_list[i].lost_time = now;
            }
            else if ((now - error_list[i].work_time) >= error_list[i].set_online_time)
            {
                error_list[i].is_lost = 0U;
                error_list[i].error_exist = 0U;
            }
        }

        vTaskDelay(DETECT_CONTROL_TIME);
    }
}

bool_t toe_is_error(uint8_t err)
{
    if (!detect_inited)
    {
        detect_init(xTaskGetTickCount());
    }

    if (err >= ERROR_LIST_LENGHT)
    {
        return 0U;
    }

    return (bool_t)(error_list[err].error_exist == 1U);
}

void detect_hook(uint8_t toe)
{
    if (!detect_inited)
    {
        detect_init(xTaskGetTickCount());
    }

    if (toe >= ERROR_LIST_LENGHT)
    {
        return;
    }

    error_list[toe].last_time = error_list[toe].new_time;
    error_list[toe].new_time = xTaskGetTickCount();
    error_list[toe].is_lost = 0U;
    error_list[toe].error_exist = 0U;

    if (error_list[toe].new_time > error_list[toe].last_time)
    {
        error_list[toe].frequency = configTICK_RATE_HZ /
                                    (fp32)(error_list[toe].new_time - error_list[toe].last_time);
    }
}

const error_t *get_error_list_point(void)
{
    return error_list;
}
