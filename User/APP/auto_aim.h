#ifndef AUTO_AIM_H
#define AUTO_AIM_H

#include "remote_control.h"

#include <stdint.h>

#define AIM_INIT_TIME     500U
#define AUTO_AIM_TIMEOUT  2000U
#define AUTO_AIM_TIME     1U

#ifndef AUTO_AIM_SOFT_ENABLE
#define AUTO_AIM_SOFT_ENABLE 0
#endif

typedef enum
{
    AIM_OFF = 0x00U,
    AIM_ON  = 0x01U
} aim_switch;

typedef struct
{
    uint8_t online;
    uint8_t auto_aim_flag;
    uint32_t last_fdb;
    const RC_ctrl_t *aim_rc;
} auto_aim_t;

extern auto_aim_t aim;

void auto_aim_task(void const *pvParameters);
void auto_aim_apply_delta_udeg(int32_t dyaw_udeg,
                               int32_t dpitch_udeg,
                               uint16_t status,
                               uint64_t ts_us);

float auto_aim_get_yaw_err_rad(void);
float auto_aim_get_pitch_err_rad(void);
uint8_t auto_aim_is_active(void);
void auto_aim_reset_delta_accum(void);

#endif
