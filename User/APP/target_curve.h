#ifndef TARGET_CURVE_H
#define TARGET_CURVE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifndef TARGET_PI
#define TARGET_PI 3.14159265358979323846f
#endif

typedef enum
{
    TARGET_CURVE_CONST = 0,
    TARGET_CURVE_SINE,
    TARGET_CURVE_COSINE,
    TARGET_CURVE_SQUARE,
    TARGET_CURVE_TRIANGLE,
    TARGET_CURVE_SAW_UP,
    TARGET_CURVE_SAW_DOWN,
    TARGET_CURVE_TRAPEZOID,
    TARGET_CURVE_STEP,
    TARGET_CURVE_RAMP,

    /* 新增：单次点到点 S 型轨迹 */
    TARGET_CURVE_S_CURVE_STEP,

    /* 新增：更贴近真实云台手感的手动输入整形模式
       输入是[-1,1]的手动指令，内部输出平滑 position / velocity / acceleration */
    TARGET_CURVE_MANUAL_S_VEL,
} TargetCurveType_t;

typedef struct
{
    float position;
    float velocity;
    float acceleration;
} TargetCurveState_t;

typedef struct
{
    TargetCurveType_t type;

    /* 通用谐波/周期参数 */
    float amplitude;
    float frequency_hz;
    float phase_rad;
    float offset;

    /* 方波参数 */
    float duty;

    /* 梯形波参数 */
    float rise_ratio;
    float high_ratio;
    float fall_ratio;

    /* step / ramp / s-curve-step 参数 */
    float step_time_s;
    float start_value;
    float end_value;
    float slope;
    float move_time_s;

    /* 手动 S 型速度整形参数 */
    float manual_input;          /* [-1, 1] */
    float manual_max_vel;        /* rad/s */
    float manual_max_acc;        /* rad/s^2 */
    float manual_max_jerk;       /* rad/s^3 */
    float manual_vel_follow_k;   /* 速度跟随增益，建议 8~20 */

    /* 通用运行时间 */
    float time_s;

    /* 输出限幅 */
    uint8_t limit_enable;
    float min_output;
    float max_output;

    /* 当前状态 */
    TargetCurveState_t state;
} TargetCurve_t;


/* 基础接口 */
void TargetCurve_Init(TargetCurve_t *obj);
void TargetCurve_Reset(TargetCurve_t *obj);

void TargetCurve_SetOutputLimit(TargetCurve_t *obj,
                                uint8_t enable,
                                float min_output,
                                float max_output);

void TargetCurve_SetType(TargetCurve_t *obj, TargetCurveType_t type);
void TargetCurve_SetAmplitude(TargetCurve_t *obj, float amplitude);
void TargetCurve_SetFrequency(TargetCurve_t *obj, float frequency_hz);
void TargetCurve_SetPhase(TargetCurve_t *obj, float phase_rad);
void TargetCurve_SetOffset(TargetCurve_t *obj, float offset);

/* 常见曲线 */
void TargetCurve_SetConstant(TargetCurve_t *obj, float value);
void TargetCurve_SetSine(TargetCurve_t *obj,
                         float amplitude,
                         float frequency_hz,
                         float phase_rad,
                         float offset);
void TargetCurve_SetCosine(TargetCurve_t *obj,
                           float amplitude,
                           float frequency_hz,
                           float phase_rad,
                           float offset);
void TargetCurve_SetSquare(TargetCurve_t *obj,
                           float amplitude,
                           float frequency_hz,
                           float phase_rad,
                           float offset,
                           float duty);
void TargetCurve_SetTriangle(TargetCurve_t *obj,
                             float amplitude,
                             float frequency_hz,
                             float phase_rad,
                             float offset);
void TargetCurve_SetSawUp(TargetCurve_t *obj,
                          float amplitude,
                          float frequency_hz,
                          float phase_rad,
                          float offset);
void TargetCurve_SetSawDown(TargetCurve_t *obj,
                            float amplitude,
                            float frequency_hz,
                            float phase_rad,
                            float offset);
void TargetCurve_SetTrapezoid(TargetCurve_t *obj,
                              float amplitude,
                              float frequency_hz,
                              float phase_rad,
                              float offset,
                              float rise_ratio,
                              float high_ratio,
                              float fall_ratio);
void TargetCurve_SetStep(TargetCurve_t *obj,
                         float start_value,
                         float end_value,
                         float step_time_s);
void TargetCurve_SetRamp(TargetCurve_t *obj,
                         float start_value,
                         float slope,
                         float start_time_s);

/* 新增：单次 S 型位置轨迹 */
void TargetCurve_SetSCurveStep(TargetCurve_t *obj,
                               float start_value,
                               float end_value,
                               float start_time_s,
                               float move_time_s);

/* 新增：更贴近真实云台的手动输入整形 */
void TargetCurve_SetManualSVel(TargetCurve_t *obj,
                               float init_position,
                               float max_vel,
                               float max_acc,
                               float max_jerk,
                               float vel_follow_k);
void TargetCurve_SetManualInput(TargetCurve_t *obj, float input_norm);

/* 计算与更新 */
TargetCurveState_t TargetCurve_CalcStateAtTime(const TargetCurve_t *obj, float time_s);
TargetCurveState_t TargetCurve_UpdateState(TargetCurve_t *obj, float dt_s);

float TargetCurve_CalcAtTime(const TargetCurve_t *obj, float time_s);
float TargetCurve_Update(TargetCurve_t *obj, float dt_s);

float TargetCurve_GetOutput(const TargetCurve_t *obj);
float TargetCurve_GetVelocity(const TargetCurve_t *obj);
float TargetCurve_GetAcceleration(const TargetCurve_t *obj);

#ifdef __cplusplus
}
#endif

#endif
