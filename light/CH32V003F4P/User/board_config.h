#ifndef __BOARD_CONFIG_H__
#define __BOARD_CONFIG_H__

#include "debug.h"

#define WS2812_LED_COUNT              10U
#define WS2812_UART_PACKET_BYTES      (WS2812_LED_COUNT * 3U)
#define WS2812_UART_FRAME_HEADER0     0xAAU
#define WS2812_UART_FRAME_HEADER1     0x55U
#define WS2812_UART_FRAME_TAIL0       0x55U
#define WS2812_UART_FRAME_TAIL1       0xAAU
#define WS2812_UART_TX_ACK_CMD        0x81U
#define WS2812_UART_TX_RUN_CMD        0x82U
#define WS2812_UART_ACK_PAYLOAD_BYTES 3U
#define WS2812_UART_RUN_PAYLOAD_BYTES (1U + 1U + 4U + WS2812_UART_PACKET_BYTES)
#define WS2812_BOOT_FLOW_PERIOD_MS    100U
#define WS2812_RUN_REPORT_PERIOD_MS   20U
#define WS2812_RESET_SLOTS            64U
#define WS2812_BITS_PER_LED           24U
#define WS2812_FRAME_BYTES            (WS2812_LED_COUNT * WS2812_BITS_PER_LED)
#define WS2812_TX_BUFFER_SIZE         (WS2812_RESET_SLOTS + WS2812_FRAME_BYTES + WS2812_RESET_SLOTS)

#define CTRL_UART_BAUDRATE            115200U
/* 0: TX=PD5 RX=PD6, 1: TX=PD6 RX=PD5 */
#define CTRL_UART_USE_PARTIAL_REMAP2  0U

#define CTRL_UART                     USART1
#define CTRL_UART_CLK                 RCC_APB2Periph_USART1
#define CTRL_UART_GPIO_CLK            RCC_APB2Periph_GPIOD
#define CTRL_UART_RX_PORT             GPIOD
#define CTRL_UART_RX_PIN              GPIO_Pin_6
#define CTRL_UART_TX_PORT             GPIOD
#define CTRL_UART_TX_PIN              GPIO_Pin_5

#define WS2812_SPI                    SPI1
#define WS2812_SPI_CLK                RCC_APB2Periph_SPI1
#define WS2812_SPI_GPIO_CLK           RCC_APB2Periph_GPIOC
#define WS2812_SPI_SCK_PORT           GPIOC
#define WS2812_SPI_SCK_PIN            GPIO_Pin_5
#define WS2812_SPI_MOSI_PORT          GPIOC
#define WS2812_SPI_MOSI_PIN           GPIO_Pin_6
#define WS2812_SPI_MISO_PORT          GPIOC
#define WS2812_SPI_MISO_PIN           GPIO_Pin_7

#define WS2812_SPI_PATTERN_0          0xC0U
#define WS2812_SPI_PATTERN_1          0xFCU

#define UART_RX_DEBUG_ECHO            1U

#endif
