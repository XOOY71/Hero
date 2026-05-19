#include "light_task.h"

#include <string.h>

#include "bsp_usart.h"
#include "chassis_behaviour.h"
#include "chassis_power_control.h"
#include "cmsis_os.h"
#include "detect_task.h"
#include "gimbal_behaviour.h"

#define LIGHT_FRAME_HEAD0 0xAAU
#define LIGHT_FRAME_HEAD1 0x55U
#define LIGHT_FRAME_TAIL0 0x55U
#define LIGHT_FRAME_TAIL1 0xAAU

#define LIGHT_DIM 12U
#define LIGHT_LOW 24U
#define LIGHT_MID 48U
#define LIGHT_HIGH 96U

extern super_cap_mode_e super_cap_mode;

static osThreadId lightTaskHandle = NULL;
static volatile light_mode_t light_mode = LIGHT_MODE_AUTO;
static light_rgb_t light_leds[LIGHT_LED_COUNT];
static uint8_t light_frame[LIGHT_UART_FRAME_BYTES];

static void light_pack_frame(void)
{
    uint8_t frame_index = 0U;

    light_frame[frame_index++] = LIGHT_FRAME_HEAD0;
    light_frame[frame_index++] = LIGHT_FRAME_HEAD1;

    for (uint8_t i = 0U; i < LIGHT_LED_COUNT; i++)
    {
        light_frame[frame_index++] = light_leds[i].r;
        light_frame[frame_index++] = light_leds[i].g;
        light_frame[frame_index++] = light_leds[i].b;
    }

    light_frame[frame_index++] = LIGHT_FRAME_TAIL0;
    light_frame[frame_index] = LIGHT_FRAME_TAIL1;
}

static void light_send_frame(void)
{
    light_pack_frame();
    USART7_Transmit(light_frame, (uint16_t)sizeof(light_frame));
}

static void light_fill(uint8_t first, uint8_t last, uint8_t r, uint8_t g, uint8_t b)
{
    if (first >= LIGHT_LED_COUNT)
    {
        return;
    }

    if (last >= LIGHT_LED_COUNT)
    {
        last = LIGHT_LED_COUNT - 1U;
    }

    for (uint8_t i = first; i <= last; i++)
    {
        light_leds[i].r = r;
        light_leds[i].g = g;
        light_leds[i].b = b;
    }
}

static bool light_any_chassis_motor_lost(void)
{
    for (uint8_t toe = CHASSIS_MOTOR1_TOE; toe <= CHASSIS_MOTOR8_TOE; toe++)
    {
        if (toe_is_error(toe))
        {
            return true;
        }
    }

    return false;
}

static void light_render_chassis_status(uint8_t tick)
{
    switch (chassis_behaviour_mode)
    {
        case CHASSIS_NO_MOVE:
            light_fill(3U, 5U, 0U, 0U, LIGHT_DIM);
            break;

        case CHASSIS_FOLLOW_GIMBAL_YAW:
            light_fill(3U, 5U, 0U, LIGHT_MID, LIGHT_MID);
            break;

        case CHASSIS_SPIN:
            light_fill(3U, 5U, LIGHT_LOW, 0U, LIGHT_MID);
            light_set_pixel((uint8_t)(3U + (tick % 3U)), LIGHT_HIGH, 0U, LIGHT_HIGH);
            break;

        case CHASSIS_RETURN:
        default:
            light_fill(3U, 5U, LIGHT_MID, LIGHT_MID, 0U);
            break;
    }
}

static void light_render_gimbal_status(void)
{
    switch (gimbal_behaviour)
    {
        case GIMBAL_ZERO_FORCE:
        case GIMBAL_MOTIONLESS:
            light_set_pixel(6U, LIGHT_LOW, 0U, 0U);
            break;

        case GIMBAL_INIT:
        case GIMBAL_CALI:
            light_set_pixel(6U, LIGHT_MID, LIGHT_MID, 0U);
            break;

        case GIMBAL_SPIN:
            light_set_pixel(6U, LIGHT_LOW, 0U, LIGHT_MID);
            break;

        case GIMBAL_ABSOLUTE_ANGLE:
        case GIMBAL_RELATIVE_ANGLE:
        default:
            light_set_pixel(6U, 0U, LIGHT_MID, 0U);
            break;
    }
}

static void light_render_cap_status(void)
{
    switch (super_cap_mode)
    {
        case SUPER_CAP_USING:
            light_set_pixel(7U, 0U, 0U, LIGHT_HIGH);
            break;

        case SUPER_CAP_PREPARED:
            light_set_pixel(7U, 0U, LIGHT_MID, 0U);
            break;

        case SUPER_CAP_CHARGING:
        default:
            light_set_pixel(7U, LIGHT_MID, LIGHT_LOW, 0U);
            break;
    }
}

static void light_render_heartbeat(uint8_t tick)
{
    if ((tick & 0x01U) == 0U)
    {
        light_set_pixel(8U, LIGHT_LOW, LIGHT_LOW, LIGHT_LOW);
        light_set_pixel(9U, 0U, 0U, 0U);
    }
    else
    {
        light_set_pixel(8U, 0U, 0U, 0U);
        light_set_pixel(9U, LIGHT_LOW, LIGHT_LOW, LIGHT_LOW);
    }
}

static void light_render_auto(void)
{
    static uint8_t tick = 0U;
    const bool dbus_lost = (toe_is_error(DBUS_TOE) != 0U);
    const bool chassis_lost = light_any_chassis_motor_lost();
    const bool blink_on = ((tick & 0x01U) == 0U);

    light_clear();

    if (dbus_lost)
    {
        if (blink_on)
        {
            light_fill(0U, LIGHT_LED_COUNT - 1U, LIGHT_HIGH, 0U, 0U);
        }
        tick++;
        return;
    }

    if (chassis_lost)
    {
        light_fill(0U, 2U, LIGHT_MID, LIGHT_LOW, 0U);
    }
    else
    {
        light_fill(0U, 2U, 0U, LIGHT_MID, 0U);
    }

    light_render_chassis_status(tick);
    light_render_gimbal_status();
    light_render_cap_status();
    light_render_heartbeat(tick);

    tick++;
}

void LightTask_Init(void)
{
    osThreadDef(lightTask, light_task, osPriorityLow, 0, 256);
    lightTaskHandle = osThreadCreate(osThread(lightTask), NULL);
}

void light_task(void const *pvParameters)
{
    (void)pvParameters;
    (void)lightTaskHandle;

    osDelay(LIGHT_TASK_INIT_TIME_MS);
    light_clear();
    light_send_frame();

    while (1)
    {
        if (light_mode == LIGHT_MODE_AUTO)
        {
            light_render_auto();
        }

        light_send_frame();
        osDelay(LIGHT_TASK_PERIOD_MS);
    }
}

void light_set_auto_mode(void)
{
    light_mode = LIGHT_MODE_AUTO;
}

void light_set_manual_mode(void)
{
    light_mode = LIGHT_MODE_MANUAL;
}

void light_set_all(uint8_t r, uint8_t g, uint8_t b)
{
    light_set_manual_mode();
    light_fill(0U, LIGHT_LED_COUNT - 1U, r, g, b);
}

void light_set_pixel(uint8_t index, uint8_t r, uint8_t g, uint8_t b)
{
    if (index >= LIGHT_LED_COUNT)
    {
        return;
    }

    light_leds[index].r = r;
    light_leds[index].g = g;
    light_leds[index].b = b;
}

void light_clear(void)
{
    memset(light_leds, 0, sizeof(light_leds));
}

void light_refresh_now(void)
{
    light_send_frame();
}
