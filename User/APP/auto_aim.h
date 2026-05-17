#ifndef _AUTO_AIM_H
#define _AUTO_AIM_H

#include <stdint.h>

#include "remote_control.h"

// 时间配置，单位 ms
#define AIM_INIT_TIME     500
#define AUTO_AIM_TIMEOUT  2000
#define AUTO_AIM_TIME     1

/* uproto tick 归属：优先由 comm_app 任务驱动协议栈。 */
#ifndef AUTO_AIM_UPROTO_TICK_ENABLE
#define AUTO_AIM_UPROTO_TICK_ENABLE 0
#endif

/* 编译期软件开关：设为 1 时，无需按 R 键即可开启自瞄。 */
#ifndef AUTO_AIM_SOFTWARE_SWITCH_ENABLE
#define AUTO_AIM_SOFTWARE_SWITCH_ENABLE 1
#endif

// 接收 yaw/pitch 增量限幅
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
    float yaw;
    float pitch;
    float distance;
    float shoot_delay;
} received_data;

typedef struct {
    received_data    receive;

    uint8_t          online;
    uint8_t          auto_aim_flag;   // 0：关闭，1：开启
    uint16_t         shoot_delay;     // 单位 ms

    uint16_t         yaw_delay;
    uint16_t         pitch_delay;

    uint32_t         last_fdb;        // 上次反馈时间，单位 ms
    const RC_ctrl_t *aim_rc;
} auto_aim_t;

/*
 * 主机控制链路：
 * USB CDC 接收 -> uproto/MUX -> gimbal_channel DELTA 回调 ->
 * auto_aim_apply_delta_udeg() -> 自瞄任务整形 ->
 * aim.receive.{yaw,pitch} 弧度增量 -> 云台控制环取用。
 */
extern auto_aim_t aim;

void AutoAimTask_Init(void);
extern void auto_aim_task(void const *pvParameters);

// 桥接函数：接收主机下发的微度单位增量
void auto_aim_apply_delta_udeg(int32_t dyaw_udeg,
                               int32_t dpitch_udeg,
                               uint16_t status,
                               uint64_t ts_us);

// MCU 侧控制节拍：轨迹整形并生成角度增量
void auto_aim_control_tick(auto_aim_t *aim_loop);

#endif
