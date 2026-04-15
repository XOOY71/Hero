#include "sine_target.h"
#include <math.h>

static float SineTarget_Clamp(float x, float min_val, float max_val)
{
    if (x < min_val)
    {
        return min_val;
    }
    if (x > max_val)
    {
        return max_val;
    }
    return x;
}

static float SineTarget_LimitAmplitude(float amplitude_rad)
{
    /* 振幅只允许非负，并且最大不超过 pi */
    return SineTarget_Clamp(amplitude_rad, 0.0f, SINE_PI);
}

static float SineTarget_LimitOutput(float output_rad)
{
    /* 输出目标位置限制在 [-pi, pi] */
    return SineTarget_Clamp(output_rad, -SINE_PI, SINE_PI);
}

void SineTarget_Init(SineTarget_t *obj,
                     float amplitude_rad,
                     float frequency_hz,
                     float phase_rad,
                     float offset_rad)
{
    if (obj == 0)
    {
        return;
    }

    obj->amplitude    = SineTarget_LimitAmplitude(amplitude_rad);
    obj->frequency_hz = (frequency_hz < 0.0f) ? 0.0f : frequency_hz;
    obj->phase_rad    = phase_rad;
    obj->offset_rad   = offset_rad;
    obj->time_s       = 0.0f;

    obj->output_rad = SineTarget_LimitOutput(
        obj->offset_rad + obj->amplitude * sinf(obj->phase_rad)
    );
}

void SineTarget_Reset(SineTarget_t *obj)
{
    if (obj == 0)
    {
        return;
    }

    obj->time_s = 0.0f;
    obj->output_rad = SineTarget_LimitOutput(
        obj->offset_rad + obj->amplitude * sinf(obj->phase_rad)
    );
}

float SineTarget_CalcAtTime(const SineTarget_t *obj, float time_s)
{
    float omega = 2.0f * SINE_PI * obj->frequency_hz;
    float output;

    if (obj == 0)
    {
        return 0.0f;
    }

    if (time_s < 0.0f)
    {
        time_s = 0.0f;
    }

    output = obj->offset_rad + obj->amplitude * sinf(omega * time_s + obj->phase_rad);
    output = SineTarget_LimitOutput(output);

    return output;
}

float SineTarget_Update(SineTarget_t *obj, float dt_s)
{
    if (obj == 0)
    {
        return 0.0f;
    }

    if (dt_s < 0.0f)
    {
        dt_s = 0.0f;
    }

    obj->time_s += dt_s;

    obj->output_rad = SineTarget_CalcAtTime(obj, obj->time_s);
    return obj->output_rad;
}

void SineTarget_SetAmplitude(SineTarget_t *obj, float amplitude_rad)
{
    if (obj == 0)
    {
        return;
    }

    obj->amplitude = SineTarget_LimitAmplitude(amplitude_rad);
    obj->output_rad = SineTarget_CalcAtTime(obj, obj->time_s);
}

void SineTarget_SetFrequency(SineTarget_t *obj, float frequency_hz)
{
    if (obj == 0)
    {
        return;
    }

    obj->frequency_hz = (frequency_hz < 0.0f) ? 0.0f : frequency_hz;
    obj->output_rad = SineTarget_CalcAtTime(obj, obj->time_s);
}

void SineTarget_SetPhase(SineTarget_t *obj, float phase_rad)
{
    if (obj == 0)
    {
        return;
    }

    obj->phase_rad = phase_rad;
    obj->output_rad = SineTarget_CalcAtTime(obj, obj->time_s);
}

void SineTarget_SetOffset(SineTarget_t *obj, float offset_rad)
{
    if (obj == 0)
    {
        return;
    }

    obj->offset_rad = offset_rad;
    obj->output_rad = SineTarget_CalcAtTime(obj, obj->time_s);
}

float SineTarget_GetOutput(const SineTarget_t *obj)
{
    if (obj == 0)
    {
        return 0.0f;
    }

    return obj->output_rad;
}