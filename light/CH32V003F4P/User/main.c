/********************************** (C) COPYRIGHT *******************************
 * File Name          : main.c
 * Author             : WCH / Codex
 * Version            : V1.0.0
 * Date               : 2026/04/12
 * Description        : UART receive + SPI WS2812 example.
 *******************************************************************************/

#include "ws2812.h"
#include "ch32v00x_it.h"

typedef enum
{
    UART_RX_WAIT_HEADER0 = 0,
    UART_RX_WAIT_HEADER1,
    UART_RX_PAYLOAD,
    UART_RX_WAIT_TAIL0,
    UART_RX_WAIT_TAIL1
} uart_rx_state_t;

typedef enum
{
    APP_MODE_BOOT_FLOW = 0U,
    APP_MODE_CONTROL = 1U
} app_mode_t;

#define UART_RX_BUFFER_SIZE 128U

static volatile uint8_t g_uart_rx_buf[UART_RX_BUFFER_SIZE];
static volatile uint8_t g_uart_rx_head = 0U;
static volatile uint8_t g_uart_rx_tail = 0U;

static uint32_t g_report_tick_ms = 0U;
static uint32_t g_boot_tick_ms = 0U;
static uint8_t g_boot_step = 0U;
static app_mode_t g_app_mode = APP_MODE_BOOT_FLOW;

static void UART_GPIO_Init(void)
{
    GPIO_InitTypeDef gpio_init = {0};

    RCC_APB2PeriphClockCmd(CTRL_UART_GPIO_CLK | RCC_APB2Periph_AFIO, ENABLE);

#if (CTRL_UART_USE_PARTIAL_REMAP2 == 1U)
    GPIO_PinRemapConfig(GPIO_PartialRemap2_USART1, ENABLE);
#endif

    gpio_init.GPIO_Pin = CTRL_UART_RX_PIN;
    gpio_init.GPIO_Speed = GPIO_Speed_30MHz;
    gpio_init.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(CTRL_UART_RX_PORT, &gpio_init);

    gpio_init.GPIO_Pin = CTRL_UART_TX_PIN;
    gpio_init.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(CTRL_UART_TX_PORT, &gpio_init);
}

static void UART_Init(void)
{
    USART_InitTypeDef uart_init = {0};
    NVIC_InitTypeDef nvic_init = {0};

    RCC_APB2PeriphClockCmd(CTRL_UART_CLK, ENABLE);
    UART_GPIO_Init();

    uart_init.USART_BaudRate = CTRL_UART_BAUDRATE;
    uart_init.USART_WordLength = USART_WordLength_8b;
    uart_init.USART_StopBits = USART_StopBits_1;
    uart_init.USART_Parity = USART_Parity_No;
    uart_init.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    uart_init.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
    USART_Init(CTRL_UART, &uart_init);
    USART_ITConfig(CTRL_UART, USART_IT_RXNE, ENABLE);

    nvic_init.NVIC_IRQChannel = USART1_IRQn;
    nvic_init.NVIC_IRQChannelPreemptionPriority = 1;
    nvic_init.NVIC_IRQChannelSubPriority = 0;
    nvic_init.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic_init);

    USART_Cmd(CTRL_UART, ENABLE);
}

void UART_RxPush(uint8_t byte)
{
    uint8_t next_head = (uint8_t)((g_uart_rx_head + 1U) % UART_RX_BUFFER_SIZE);

    if(next_head != g_uart_rx_tail)
    {
        g_uart_rx_buf[g_uart_rx_head] = byte;
        g_uart_rx_head = next_head;
    }
}

static uint8_t UART_RxPop(uint8_t *byte)
{
    if((byte == 0) || (g_uart_rx_head == g_uart_rx_tail))
    {
        return 0U;
    }

    *byte = g_uart_rx_buf[g_uart_rx_tail];
    g_uart_rx_tail = (uint8_t)((g_uart_rx_tail + 1U) % UART_RX_BUFFER_SIZE);
    return 1U;
}

static void UART_SendByte(uint8_t byte)
{
    while(USART_GetFlagStatus(CTRL_UART, USART_FLAG_TXE) == RESET)
    {
    }
    USART_SendData(CTRL_UART, byte);
}

static void UART_WaitTxComplete(void)
{
    while(USART_GetFlagStatus(CTRL_UART, USART_FLAG_TC) == RESET)
    {
    }
}

static void UART_SendText(const char *text)
{
    while((text != 0) && (*text != '\0'))
    {
        UART_SendByte((uint8_t)(*text));
        text++;
    }
    UART_WaitTxComplete();
}

static void UART_SendBuffer(const uint8_t *data, uint8_t length)
{
    uint8_t index;

    if(data == 0)
    {
        return;
    }

    for(index = 0U; index < length; index++)
    {
        UART_SendByte(data[index]);
    }
}

static void UART_SendFrame(uint8_t cmd, const uint8_t *payload, uint8_t payload_len)
{
    UART_SendByte(WS2812_UART_FRAME_HEADER0);
    UART_SendByte(WS2812_UART_FRAME_HEADER1);
    UART_SendByte(cmd);
    UART_SendByte(payload_len);
    UART_SendBuffer(payload, payload_len);
    UART_SendByte(WS2812_UART_FRAME_TAIL0);
    UART_SendByte(WS2812_UART_FRAME_TAIL1);
    UART_WaitTxComplete();
}

static void UART_SendAckFrame(uint8_t status)
{
    uint8_t payload[WS2812_UART_ACK_PAYLOAD_BYTES];

    payload[0] = status;
    payload[1] = (uint8_t)g_app_mode;
    payload[2] = WS2812_UART_PACKET_BYTES;
    UART_SendFrame(WS2812_UART_TX_ACK_CMD, payload, WS2812_UART_ACK_PAYLOAD_BYTES);
}

static void UART_SendRunFrame(void)
{
    uint8_t payload[WS2812_UART_RUN_PAYLOAD_BYTES];
    uint32_t clock_hz = SystemCoreClock;

    payload[0] = (uint8_t)g_app_mode;
    payload[1] = WS2812_LED_COUNT;
    payload[2] = (uint8_t)(clock_hz & 0xFFU);
    payload[3] = (uint8_t)((clock_hz >> 8) & 0xFFU);
    payload[4] = (uint8_t)((clock_hz >> 16) & 0xFFU);
    payload[5] = (uint8_t)((clock_hz >> 24) & 0xFFU);
    WS2812_CopyRgbBuffer(&payload[6], WS2812_UART_PACKET_BYTES);
    UART_SendFrame(WS2812_UART_TX_RUN_CMD, payload, WS2812_UART_RUN_PAYLOAD_BYTES);
}

static void Process_Packet(const uint8_t *packet)
{
    if(packet == 0)
    {
        return;
    }

    g_app_mode = APP_MODE_CONTROL;
    WS2812_SetFromRgbBuffer(packet, WS2812_UART_PACKET_BYTES);
    WS2812_Refresh();
    UART_SendText("RX_OK\r\n");
    UART_SendAckFrame(0U);
    UART_SendRunFrame();
}

static void WS2812_PlayBootFlow(void)
{
    static const ws2812_color_t palette[] =
    {
        {0U, 48U, 0U},
        {48U, 48U, 0U},
        {0U, 48U, 0U},
        {0U, 48U, 48U},
        {0U, 0U, 48U},
        {48U, 0U, 48U}
    };
    uint8_t i;
    uint8_t palette_len = (uint8_t)(sizeof(palette) / sizeof(palette[0]));

    for(i = 0U; i < WS2812_LED_COUNT; i++)
    {
        WS2812_SetPixel(i,
                        palette[(uint8_t)((i + g_boot_step) % palette_len)].r,
                        palette[(uint8_t)((i + g_boot_step) % palette_len)].g,
                        palette[(uint8_t)((i + g_boot_step) % palette_len)].b);
    }

    WS2812_Refresh();
    g_boot_step++;
}

int main(void)
{
    uint8_t packet[WS2812_UART_PACKET_BYTES] = {0};
    uint8_t packet_index = 0U;
    uint8_t rx;
    uart_rx_state_t rx_state = UART_RX_WAIT_HEADER0;

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
    SystemCoreClockUpdate();
    Delay_Init();

    UART_Init();
    WS2812_Init();

    UART_SendText("light ready\r\n");
    WS2812_PlayBootFlow();
    UART_SendRunFrame();

    while(1)
    {
        while(USART_GetFlagStatus(CTRL_UART, USART_FLAG_RXNE) != RESET)
        {
            rx = (uint8_t)USART_ReceiveData(CTRL_UART);
#if (UART_RX_DEBUG_ECHO == 1U)
            UART_SendByte(rx);
#endif
            UART_RxPush(rx);
        }

        while(UART_RxPop(&rx) != 0U)
        {
            switch(rx_state)
            {
                case UART_RX_WAIT_HEADER0:
                    if(rx == WS2812_UART_FRAME_HEADER0)
                    {
                        rx_state = UART_RX_WAIT_HEADER1;
                    }
                    break;

                case UART_RX_WAIT_HEADER1:
                    if(rx == WS2812_UART_FRAME_HEADER1)
                    {
                        packet_index = 0U;
                        rx_state = UART_RX_PAYLOAD;
                    }
                    else if(rx == WS2812_UART_FRAME_HEADER0)
                    {
                        rx_state = UART_RX_WAIT_HEADER1;
                    }
                    else
                    {
                        rx_state = UART_RX_WAIT_HEADER0;
                    }
                    break;

                case UART_RX_PAYLOAD:
                    packet[packet_index++] = rx;
                    if(packet_index >= WS2812_UART_PACKET_BYTES)
                    {
                        rx_state = UART_RX_WAIT_TAIL0;
                    }
                    break;

                case UART_RX_WAIT_TAIL0:
                    if(rx == WS2812_UART_FRAME_TAIL0)
                    {
                        rx_state = UART_RX_WAIT_TAIL1;
                    }
                    else if(rx == WS2812_UART_FRAME_HEADER0)
                    {
                        rx_state = UART_RX_WAIT_HEADER1;
                        packet_index = 0U;
                    }
                    else
                    {
                        rx_state = UART_RX_WAIT_HEADER0;
                        packet_index = 0U;
                    }
                    break;

                case UART_RX_WAIT_TAIL1:
                    if(rx == WS2812_UART_FRAME_TAIL1)
                    {
                        Process_Packet(packet);
                    }
                    else
                    {
                        UART_SendText("RX_ERR\r\n");
                        UART_SendAckFrame(1U);
                    }
                    packet_index = 0U;
                    rx_state = UART_RX_WAIT_HEADER0;
                    break;

                default:
                    packet_index = 0U;
                    rx_state = UART_RX_WAIT_HEADER0;
                    break;
            }
        }

        Delay_Ms(1U);
        g_report_tick_ms++;
        g_boot_tick_ms++;

        if((g_app_mode == APP_MODE_BOOT_FLOW) && (g_boot_tick_ms >= WS2812_BOOT_FLOW_PERIOD_MS))
        {
            g_boot_tick_ms = 0U;
            WS2812_PlayBootFlow();
        }

        if(g_report_tick_ms >= WS2812_RUN_REPORT_PERIOD_MS)
        {
            g_report_tick_ms = 0U;
            UART_SendRunFrame();
        }
    }
}
