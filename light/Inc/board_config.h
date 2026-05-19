#ifndef __BOARD_CONFIG_H__
#define __BOARD_CONFIG_H__

#include "ch32v00x.h"

#define WS2812_LED_COUNT              10U
#define WS2812_RESET_SLOTS            64U
#define WS2812_SPI_BYTES_PER_BIT      1U
#define WS2812_BITS_PER_LED           24U
#define WS2812_FRAME_BYTES            (WS2812_LED_COUNT * WS2812_BITS_PER_LED * WS2812_SPI_BYTES_PER_BIT)
#define WS2812_TX_BUFFER_SIZE         (WS2812_RESET_SLOTS + WS2812_FRAME_BYTES + WS2812_RESET_SLOTS)

#define CTRL_UART_BAUDRATE            115200U

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

/*
 * WS2812 encoding by SPI:
 * bit 0 -> 0b11000000 = 0xC0
 * bit 1 -> 0b11111100 = 0xFC
 *
 * At SPI clock around 6 MHz, one SPI byte lasts about 1.33 us, close to one
 * WS2812 bit cell. High pulse width is shaped by the bit pattern itself.
 */
#define WS2812_SPI_PATTERN_0          0xC0U
#define WS2812_SPI_PATTERN_1          0xFCU

#endif
