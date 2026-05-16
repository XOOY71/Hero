#ifndef _AUTO_AIM_H
#define _AUTO_AIM_H

#include <stdint.h>

#include "remote_control.h"
#include "struct_typedef.h"

// timing (ms)
#define AIM_INIT_TIME     500
#define AUTO_AIM_TIMEOUT  2000
#define AUTO_AIM_TIME     1

/* uproto tick ownership: prefer comm_app task to tick protocol. */
#ifndef AUTO_AIM_UPROTO_TICK_ENABLE
#define AUTO_AIM_UPROTO_TICK_ENABLE 0
#endif

// simple bounds for received yaw/pitch
#define MAX_VAL   0.001f
#define MIN_VAL  (-MAX_VAL)

#define MAX_YAW   MAX_VAL
#define MIN_YAW   MIN_VAL

#define MAX_PITCH MAX_VAL
#define MIN_PITCH MIN_VAL

typedef enum {
    AIM_OFF = 0x00,
    AIM_ON  = 0x01
} aim_switch;

typedef struct {
    fp32 yaw;
    fp32 pitch;
    fp32 distance;
    fp32 shoot_delay;
} received_data;

typedef struct {
    received_data    receive;

    uint8_t          online;
    uint8_t          auto_aim_flag;   // 0: off, 1: on
    uint16_t         shoot_delay;     // ms

    uint16_t         yaw_delay;
    uint16_t         pitch_delay;

    uint32_t         last_fdb;        // last feedback time (ms)
    const RC_ctrl_t *aim_rc;
} auto_aim_t;

/*
 * Host control chain:
 * USB CDC RX -> uproto/MUX -> gimbal_channel DELTA callback ->
 * auto_aim_apply_delta_udeg() -> auto_aim task shaping ->
 * aim.receive.{yaw,pitch} incremental radians -> gimbal control loop consumes.
 */
extern auto_aim_t aim;

void AutoAimTask_Init(void);
extern void auto_aim_task(void const *pvParameters);

// Bridge: apply host delta in micro-degree (udeg)
void auto_aim_apply_delta_udeg(int32_t dyaw_udeg,
                               int32_t dpitch_udeg,
                               uint16_t status,
                               uint64_t ts_us);

// MCU-side control tick: trajectory shaping + increment generation
void auto_aim_control_tick(auto_aim_t *aim_loop);

#endif
