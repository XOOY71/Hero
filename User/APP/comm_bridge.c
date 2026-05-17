#include "comm_bridge.h"

#include "auto_aim.h"
#include "cmsis_os.h"
#include "gimbal_task.h"
#include "hwt_imu.h"
#include "stm32h7xx_hal.h"
#include "usbd_core.h"
#include "usbd_cdc_if.h"

#include "../Communication/channel/gimbal/gimbal_channel.h"
#include "../Communication/channel/gimbal/gimbal_config.h"
#include "../Communication/core/comm.h"
#include "../Communication/core/uproto.h"
#include "../Communication/example/shared/protocol_ids.h"

uproto_context_t proto_ctx;
extern USBD_HandleTypeDef hUsbDeviceHS;

static uint8_t s_comm_inited = 0U;
static osThreadId s_comm_task_handle = NULL;
static channel_manager_t s_comm_mgr;
static ch_uproto_bind_t s_comm_bind;
static gimbal_channel_t s_gimbal_channel;

static bool comm_bridge_get_gimbal_state(gimbal_state_t *out, void *user);
static void comm_bridge_task(void const *argument);

#ifndef COMM_BRIDGE_USB_ENUM_TIMEOUT_MS
#define COMM_BRIDGE_USB_ENUM_TIMEOUT_MS 5000U
#endif

static uint32_t comm_bridge_now_ms(void *user)
{
    (void)user;
    return HAL_GetTick();
}

static uint64_t comm_bridge_now_us(void *user)
{
    (void)user;
    return (uint64_t)HAL_GetTick() * 1000ULL;
}

static int32_t comm_bridge_rad_to_udeg(float rad)
{
    const float inv_pi = 0.31830988618379067154f;
    return (int32_t)(rad * 180000000.0f * inv_pi);
}

static bool comm_bridge_get_gimbal_state(gimbal_state_t *out, void *user)
{
    const gimbal_motor_t *yaw;
    const gimbal_motor_t *pitch;
    const float *imu;

    (void)user;

    if (out == NULL)
    {
        return false;
    }

    yaw = get_yaw_motor_point();
    pitch = get_pitch_motor_point();
    if (yaw != NULL && pitch != NULL)
    {
        out->enc_yaw = comm_bridge_rad_to_udeg(yaw->relative_angle);
        out->enc_pitch = comm_bridge_rad_to_udeg(pitch->relative_angle);
    }
    else
    {
        out->enc_yaw = 0;
        out->enc_pitch = 0;
    }

    imu = get_INS_angle_point();
    if (imu != NULL)
    {
        out->yaw_udeg = comm_bridge_rad_to_udeg(imu[0]);
        out->pitch_udeg = comm_bridge_rad_to_udeg(imu[1]);
        out->roll_udeg = comm_bridge_rad_to_udeg(imu[2]);
    }
    else
    {
        out->yaw_udeg = 0;
        out->pitch_udeg = 0;
        out->roll_udeg = 0;
    }

    out->ts_us = comm_bridge_now_us(NULL);
    return true;
}

static void comm_bridge_task(void const *argument)
{
    uint32_t start_ms;

    (void)argument;

    start_ms = HAL_GetTick();
    while ((hUsbDeviceHS.dev_state != USBD_STATE_CONFIGURED) &&
           ((HAL_GetTick() - start_ms) < COMM_BRIDGE_USB_ENUM_TIMEOUT_MS))
    {
        osDelay(1);
    }

    while (1)
    {
        uproto_tick(&proto_ctx);
        ch_uproto_arbiter_tick(&s_comm_bind);
        chmgr_tick(&s_comm_mgr);
        osDelay(1);
    }
}

void CommBridge_Init(void)
{
    uproto_port_ops_t port_ops = {0};
    uproto_time_ops_t time_ops = {0};
    gimbal_source_ops_t gimbal_src = {0};
    gimbal_hooks_t gimbal_hooks = {0};
    gimbal_channel_cfg_t gimbal_cfg = {0};

    if (s_comm_inited != 0U)
    {
        return;
    }

    port_ops.write = usbd_cdc_port_write;
    time_ops.now_ms = comm_bridge_now_ms;
    uproto_init(&proto_ctx, &port_ops, &time_ops, NULL);

    chmgr_init(&s_comm_mgr);
    ch_uproto_bind(&s_comm_bind, &proto_ctx, UPROTO_MSG_MUX, &s_comm_mgr);
    ch_uproto_register_rx(&s_comm_bind);

    gimbal_src.get_state = comm_bridge_get_gimbal_state;
    gimbal_src.user = NULL;
    gimbal_src.period_ms = GIMBAL_PUB_PERIOD_MS;

    gimbal_cfg.ch_id = GIMBAL_CH_ID;
    gimbal_cfg.priority = GIMBAL_PRIORITY;
    gimbal_cfg.period_ms = 0U;
    gimbal_channel_init_ex(&s_gimbal_channel,
                           &s_comm_bind,
                           &s_comm_mgr,
                           &gimbal_cfg,
                           &gimbal_src,
                           &gimbal_hooks,
                           comm_bridge_now_us,
                           NULL);

    if (s_comm_task_handle == NULL)
    {
        osThreadDef(commBridgeTask, comm_bridge_task, osPriorityNormal, 0, 512);
        s_comm_task_handle = osThreadCreate(osThread(commBridgeTask), NULL);
    }

    s_comm_inited = 1U;
}
