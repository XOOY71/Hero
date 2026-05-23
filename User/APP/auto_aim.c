#include "auto_aim.h"

#include "cmsis_os.h"
#include "project_config.h"
#include "stm32h7xx_hal.h"

#include <stdbool.h>
#include <stddef.h>

#define AUTO_AIM_UDEG_TO_RAD (PI / 180000000.0f)

typedef struct
{
    float yaw_err_rad;
    float pitch_err_rad;
} auto_aim_error_t;

auto_aim_t aim;

static auto_aim_error_t s_auto_aim_error = {0};

static void auto_aim_init(auto_aim_t *aim_obj);
static void auto_aim_set(auto_aim_t *aim_obj);
static void auto_aim_feedback_update(auto_aim_t *aim_obj);

static void auto_aim_clear_error(void)
{
    taskENTER_CRITICAL();
    s_auto_aim_error.yaw_err_rad = 0.0f;
    s_auto_aim_error.pitch_err_rad = 0.0f;
    taskEXIT_CRITICAL();
}

float auto_aim_get_yaw_err_rad(void)
{
    float err;

    taskENTER_CRITICAL();
    err = s_auto_aim_error.yaw_err_rad;
    taskEXIT_CRITICAL();

    return err;
}

float auto_aim_get_pitch_err_rad(void)
{
    float err;

    taskENTER_CRITICAL();
    err = s_auto_aim_error.pitch_err_rad;
    taskEXIT_CRITICAL();

    return err;
}

uint8_t auto_aim_is_active(void)
{
    return (uint8_t)((aim.auto_aim_flag == AIM_ON) && (aim.online != 0U));
}

void auto_aim_reset_delta_accum(void)
{
    auto_aim_clear_error();
}

void auto_aim_task(void const *pvParameters)
{
    (void)pvParameters;

    osDelay(AIM_INIT_TIME);
    auto_aim_init(&aim);

    while (1)
    {
        auto_aim_set(&aim);
        auto_aim_feedback_update(&aim);
        osDelay(AUTO_AIM_TIME);
    }
}

void auto_aim_apply_delta_udeg(int32_t dyaw_udeg,
                               int32_t dpitch_udeg,
                               uint16_t status,
                               uint64_t ts_us)
{
    const float yaw_err_rad = (float)dyaw_udeg * AUTO_AIM_UDEG_TO_RAD;
    const float pitch_err_rad = (float)dpitch_udeg * AUTO_AIM_UDEG_TO_RAD;

    (void)status;
    (void)ts_us;

    taskENTER_CRITICAL();
    s_auto_aim_error.yaw_err_rad = yaw_err_rad;
    s_auto_aim_error.pitch_err_rad = pitch_err_rad;
    taskEXIT_CRITICAL();

    aim.online = 1U;
    aim.last_fdb = HAL_GetTick();
}

static void auto_aim_init(auto_aim_t *aim_obj)
{
    if (aim_obj == NULL)
    {
        return;
    }

    aim_obj->online = 1U;
    aim_obj->auto_aim_flag = (AUTO_AIM_SOFT_ENABLE != 0) ? AIM_ON : AIM_OFF;
    aim_obj->last_fdb = 0U;
    aim_obj->aim_rc = get_remote_control_point();

    auto_aim_clear_error();
}

static void auto_aim_set(auto_aim_t *aim_obj)
{
    static bool last_press_r = false;
    bool press_r;

    if (aim_obj == NULL)
    {
        return;
    }

#if ROBOT_MODE == release
    if (HAL_GetTick() - aim_obj->last_fdb > AUTO_AIM_TIMEOUT)
    {
        aim_obj->online = 0U;
        aim_obj->auto_aim_flag = AIM_OFF;
        auto_aim_clear_error();
        return;
    }
#endif

    press_r = (aim_obj->aim_rc != NULL) &&
              ((aim_obj->aim_rc->key.v & KEY_PRESSED_OFFSET_R) != 0U);

    if (press_r && !last_press_r)
    {
        aim_obj->auto_aim_flag =
            (aim_obj->auto_aim_flag == AIM_OFF) ? AIM_ON : AIM_OFF;
    }
    last_press_r = press_r;
}

static void auto_aim_feedback_update(auto_aim_t *aim_obj)
{
    if (aim_obj == NULL)
    {
        return;
    }

    if (aim_obj->auto_aim_flag == AIM_OFF)
    {
        auto_aim_clear_error();
    }
}
