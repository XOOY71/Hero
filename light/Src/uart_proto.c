#include "uart_proto.h"

typedef enum
{
    UART_STATE_HEADER0 = 0,
    UART_STATE_HEADER1,
    UART_STATE_LEN,
    UART_STATE_PAYLOAD,
    UART_STATE_CHECKSUM
} uart_state_t;

static uart_state_t g_state = UART_STATE_HEADER0;
static uint8_t g_len = 0U;
static uint8_t g_index = 0U;
static uint8_t g_checksum = 0U;
static uint8_t g_payload[1U + UART_PROTO_LED_PAYLOAD_LEN];

void UART_Proto_Reset(void)
{
    g_state = UART_STATE_HEADER0;
    g_len = 0U;
    g_index = 0U;
    g_checksum = 0U;
}

uint8_t UART_Proto_InputByte(uint8_t byte, uart_frame_t *frame)
{
    if (frame == 0)
    {
        UART_Proto_Reset();
        return 0U;
    }

    switch (g_state)
    {
        case UART_STATE_HEADER0:
            if (byte == UART_PROTO_HEADER_0)
            {
                g_state = UART_STATE_HEADER1;
            }
            break;

        case UART_STATE_HEADER1:
            if (byte == UART_PROTO_HEADER_1)
            {
                g_state = UART_STATE_LEN;
            }
            else
            {
                g_state = UART_STATE_HEADER0;
            }
            break;

        case UART_STATE_LEN:
            if ((byte == 0U) || (byte > (1U + UART_PROTO_LED_PAYLOAD_LEN)))
            {
                UART_Proto_Reset();
            }
            else
            {
                g_len = byte;
                g_index = 0U;
                g_checksum = byte;
                g_state = UART_STATE_PAYLOAD;
            }
            break;

        case UART_STATE_PAYLOAD:
            g_payload[g_index++] = byte;
            g_checksum = (uint8_t)(g_checksum + byte);
            if (g_index >= g_len)
            {
                g_state = UART_STATE_CHECKSUM;
            }
            break;

        case UART_STATE_CHECKSUM:
            if (byte == g_checksum)
            {
                frame->cmd = g_payload[0];
                frame->data_len = (uint8_t)(g_len - 1U);
                frame->ready = 1U;

                if (frame->data_len > 0U)
                {
                    uint8_t i;
                    for (i = 0U; i < frame->data_len; i++)
                    {
                        frame->data[i] = g_payload[1U + i];
                    }
                }

                UART_Proto_Reset();
                return 1U;
            }

            UART_Proto_Reset();
            break;

        default:
            UART_Proto_Reset();
            break;
    }

    return 0U;
}
