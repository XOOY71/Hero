#include "board_config.h"
#include "uart_proto.h"
#include "ws2812.h"

static void Delay(volatile uint32_t count)
{
    while (count-- != 0U)
    {
    }
}

static void Clock_Init(void)
{
    SystemCoreClockUpdate();
}

static void UART_GPIO_Init(void)
{
    GPIO_InitTypeDef gpio_init = {0};

    RCC_APB2PeriphClockCmd(CTRL_UART_GPIO_CLK, ENABLE);

    gpio_init.GPIO_Pin = CTRL_UART_RX_PIN;
    gpio_init.GPIO_Speed = GPIO_Speed_50MHz;
    gpio_init.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(CTRL_UART_RX_PORT, &gpio_init);

    gpio_init.GPIO_Pin = CTRL_UART_TX_PIN;
    gpio_init.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(CTRL_UART_TX_PORT, &gpio_init);
}

static void UART_Init(void)
{
    USART_InitTypeDef uart_init = {0};

    RCC_APB2PeriphClockCmd(CTRL_UART_CLK, ENABLE);
    UART_GPIO_Init();

    uart_init.USART_BaudRate = CTRL_UART_BAUDRATE;
    uart_init.USART_WordLength = USART_WordLength_8b;
    uart_init.USART_StopBits = USART_StopBits_1;
    uart_init.USART_Parity = USART_Parity_No;
    uart_init.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    uart_init.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
    USART_Init(CTRL_UART, &uart_init);
    USART_Cmd(CTRL_UART, ENABLE);
}

static void UART_SendByte(uint8_t byte)
{
    while (USART_GetFlagStatus(CTRL_UART, USART_FLAG_TXE) == RESET)
    {
    }
    USART_SendData(CTRL_UART, byte);
}

static void UART_SendText(const char *text)
{
    while ((text != 0) && (*text != '\0'))
    {
        UART_SendByte((uint8_t)(*text));
        text++;
    }
}

static void Process_Frame(const uart_frame_t *frame)
{
    if ((frame == 0) || (frame->ready == 0U))
    {
        return;
    }

    if ((frame->cmd == UART_PROTO_CMD_SET_ALL) && (frame->data_len == UART_PROTO_LED_PAYLOAD_LEN))
    {
        WS2812_SetAll(frame->data, frame->data_len);
        WS2812_Refresh();
        UART_SendText("OK\r\n");
    }
    else
    {
        UART_SendText("ERR\r\n");
    }
}

int main(void)
{
    uart_frame_t frame = {0};

    Clock_Init();
    UART_Init();
    WS2812_Init();
    UART_Proto_Reset();

    UART_SendText("light ready\r\n");

    while (1)
    {
        if (USART_GetFlagStatus(CTRL_UART, USART_FLAG_RXNE) != RESET)
        {
            uint8_t rx = (uint8_t)USART_ReceiveData(CTRL_UART);

            frame.ready = 0U;
            if (UART_Proto_InputByte(rx, &frame) != 0U)
            {
                Process_Frame(&frame);
            }
        }

        Delay(1000U);
    }
}
