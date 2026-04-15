#ifndef __SINE_TARGET_H__
#define __SINE_TARGET_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* 如果你的工程里没有定义 M_PI，这里手动定义 */
#ifndef SINE_PI
#define SINE_PI 3.14159265358979323846f
#endif

typedef struct
{
    float amplitude;      /* 振幅，单位：rad，内部会限制到 [0, pi] */
    float frequency_hz;   /* 频率，单位：Hz */
    float phase_rad;      /* 初相位，单位：rad */
    float offset_rad;     /* 偏置，单位：rad */
    float time_s;         /* 内部累计时间，单位：s */
    float output_rad;     /* 当前输出，单位：rad */
} SineTarget_t;

/**
 * @brief  初始化正弦目标发生器
 * @param  obj           目标对象指针
 * @param  amplitude_rad 振幅(rad)，会被限制到 [0, pi]
 * @param  frequency_hz  频率(Hz)
 * @param  phase_rad     初相位(rad)
 * @param  offset_rad    偏置(rad)
 */
void SineTarget_Init(SineTarget_t *obj,
                     float amplitude_rad,
                     float frequency_hz,
                     float phase_rad,
                     float offset_rad);

/**
 * @brief  重置时间和输出
 * @param  obj 目标对象指针
 */
void SineTarget_Reset(SineTarget_t *obj);

/**
 * @brief  更新一次正弦输出
 * @param  obj  目标对象指针
 * @param  dt_s 距离上次更新的时间(s)
 * @return 当前目标位置(rad)，已限制在 [-pi, pi]
 */
float SineTarget_Update(SineTarget_t *obj, float dt_s);

/**
 * @brief  按指定绝对时间直接计算输出
 * @param  obj    目标对象指针
 * @param  time_s 绝对时间(s)
 * @return 当前目标位置(rad)，已限制在 [-pi, pi]
 */
float SineTarget_CalcAtTime(const SineTarget_t *obj, float time_s);

/**
 * @brief  设置振幅
 * @param  obj           目标对象指针
 * @param  amplitude_rad 振幅(rad)，内部限制到 [0, pi]
 */
void SineTarget_SetAmplitude(SineTarget_t *obj, float amplitude_rad);

/**
 * @brief  设置频率
 * @param  obj          目标对象指针
 * @param  frequency_hz 频率(Hz)
 */
void SineTarget_SetFrequency(SineTarget_t *obj, float frequency_hz);

/**
 * @brief  设置初相位
 * @param  obj       目标对象指针
 * @param  phase_rad 初相位(rad)
 */
void SineTarget_SetPhase(SineTarget_t *obj, float phase_rad);

/**
 * @brief  设置偏置
 * @param  obj        目标对象指针
 * @param  offset_rad 偏置(rad)
 */
void SineTarget_SetOffset(SineTarget_t *obj, float offset_rad);

/**
 * @brief  获取当前输出
 * @param  obj 目标对象指针
 * @return 当前目标位置(rad)
 */
float SineTarget_GetOutput(const SineTarget_t *obj);

#ifdef __cplusplus
}
#endif

#endif /* __SINE_TARGET_H__ */
