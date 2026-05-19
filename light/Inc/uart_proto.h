#ifndef __UART_PROTO_H__
#define __UART_PROTO_H__

#include "board_config.h"

#define UART_PROTO_HEADER_0           0xAAU
#define UART_PROTO_HEADER_1           0x55U
#define UART_PROTO_CMD_SET_ALL        0x01U
#define UART_PROTO_LED_PAYLOAD_LEN    (WS2812_LED_COUNT * 3U)

typedef struct
{
    uint8_t cmd;
    uint8_t data[UART_PROTO_LED_PAYLOAD_LEN];
    uint8_t data_len;
    uint8_t ready;
} uart_frame_t;

void UART_Proto_Reset(void);
uint8_t UART_Proto_InputByte(uint8_t byte, uart_frame_t *frame);

#endif
