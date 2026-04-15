#include "vofa.h"
#include "usart.h"   // huart1

static VOFA_JustFloatFrame_t s_vofa_frame =
{
    .tail = {0x00, 0x00, 0x80, 0x7F}
};

void VOFA_Send6(float ch0, float ch1, float ch2, float ch3, float ch4, float ch5)
{
    s_vofa_frame.fdata[0] = ch0;
    s_vofa_frame.fdata[1] = ch1;
    s_vofa_frame.fdata[2] = ch2;
    s_vofa_frame.fdata[3] = ch3;
    s_vofa_frame.fdata[4] = ch4;
    s_vofa_frame.fdata[5] = ch5;

    HAL_UART_Transmit_DMA(&huart1, (uint8_t *)&s_vofa_frame, sizeof(s_vofa_frame));
}

void VOFA_SendMotorMap(const moterMapHeader *map)
{
    if (map == NULL)
    {
        return;
    }

    VOFA_Send6(map->j0, map->j1, map->j2, map->j3, map->j4, map->j5);
}