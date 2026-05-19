#ifndef __WS2812_H__
#define __WS2812_H__

#include "board_config.h"

typedef struct
{
    uint8_t g;
    uint8_t r;
    uint8_t b;
} ws2812_color_t;

void WS2812_MapColorCode(uint8_t color_code, uint8_t brightness, ws2812_color_t *color);
void WS2812_Init(void);
void WS2812_Clear(void);
void WS2812_SetPixel(uint8_t index, uint8_t r, uint8_t g, uint8_t b);
void WS2812_SetFromRgbBuffer(const uint8_t *rgb_data, uint8_t byte_count);
void WS2812_CopyRgbBuffer(uint8_t *rgb_data, uint8_t byte_count);
void WS2812_SetByCode(uint8_t index, uint8_t color_code, uint8_t brightness);
void WS2812_Refresh(void);

#endif
