#include "comm_bridge.h"

#include "auto_aim.h"
#include "stm32h7xx_hal.h"
#include "usbd_cdc_if.h"

#include "../Communication/channel/gimbal/gimbal_channel.h"
#include "../Communication/channel/gimbal/gimbal_config.h"
#include "../Communication/core/comm.h"
#include "../Communication/core/uproto.h"
#include "../Communication/example/shared/protocol_ids.h"

uproto_context_t proto_ctx;

static uint8_t s_comm_inited = 0U;
static channel_manager_t s_comm_mgr;
static ch_uproto_bind_t s_comm_bind;
static gimbal_channel_t s_gimbal_channel;

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

    s_comm_inited = 1U;
}
