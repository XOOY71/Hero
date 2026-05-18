#ifndef _AUTO_AIM_H
#define _AUTO_AIM_H

#include <stdint.h>
#include "remote_control.h"
// [SYNC_FROM_H] Header synced from H:\DM-balanceV1\User\APP (adds control tick prototype)

// timing (ms)
#define AIM_INIT_TIME     500
#define AUTO_AIM_TIMEOUT  2000
#define AUTO_AIM_TIME     1
#define PRESS_TIME        500

/* Software default switch: 1 boots auto-aim enabled, 0 boots disabled.
 * The existing RC key toggle still changes aim.auto_aim_flag at runtime. */
#ifndef AUTO_AIM_SOFT_ENABLE
#define AUTO_AIM_SOFT_ENABLE 1
#endif

/* uproto tick ownership: prefer comm_app task to tick protocol. */
/* uproto tick职责：优先由comm_app任务驱动协议栈。 */
#ifndef AUTO_AIM_UPROTO_TICK_ENABLE
#define AUTO_AIM_UPROTO_TICK_ENABLE 0
#endif

// simple bounds for received yaw/pitch
//#define MAX_VAL   0.0025f
#define MAX_VAL   0.001f
#define MIN_VAL  -MAX_VAL

#define MAX_YAW   MAX_VAL
#define MIN_YAW   MIN_VAL

#define MAX_PITCH MAX_VAL
#define MIN_PITCH MIN_VAL

typedef union {
    int16_t  int16;
    uint8_t  bytes[2];
} int16_bytes_t;

typedef union {
    float    fp32;
    uint8_t  bytes[4];
} fp32_bytes_t;

typedef enum {
    AIM_OFF = 0x00,
    AIM_ON  = 0x01
} aim_switch;

typedef struct {
    float yaw;
    float pitch;
    float distance;
    float shoot_delay;
} received_data;

typedef struct {
    float yaw;
    float pitch;
} aim_err_rad_t;

typedef struct {
    float yaw;
    float pitch;
} aim_err_deg_t;

typedef struct {
    received_data    receive;
    aim_err_rad_t    err_rad_lpf;
    aim_err_deg_t    err_deg;

    uint8_t          online;
    uint8_t          auto_aim_flag;   // 0: off, 1: on
    uint16_t         shoot_delay;     // ms

    uint16_t         yaw_delay;
    uint16_t         pitch_delay;

    uint32_t         last_fdb;        // last feedback time (ms)
    const RC_ctrl_t *aim_rc;
} auto_aim_t;

extern auto_aim_t aim;

extern void auto_aim_task(void const *pvParameters);
extern void auto_aim_loop(auto_aim_t *aim_loop);
extern void send_to_minipc(void);

// Bridge: apply host delta in micro-degree (udeg)
void auto_aim_apply_delta_udeg(int32_t dyaw_udeg,
                               int32_t dpitch_udeg,
                               uint16_t status,
                               uint64_t ts_us);

// MCU-side control tick: trajectory shaping + increment generation
void auto_aim_control_tick(auto_aim_t *aim_loop);
float auto_aim_get_yaw_err_rad(void);
float auto_aim_get_pitch_err_rad(void);
uint8_t auto_aim_is_active(void);
float auto_aim_take_yaw_delta(void);
float auto_aim_take_pitch_delta(void);
void auto_aim_reset_delta_accum(void);

#endif
