#include "ws2812.h"

#define WS2812_GRB_BYTES              (WS2812_LED_COUNT * 3U)

static ws2812_color_t g_led_buffer[WS2812_LED_COUNT];
static uint8_t g_spi_tx_buffer[WS2812_TX_BUFFER_SIZE];

static void WS2812_GPIO_Init(void)
{
    GPIO_InitTypeDef gpio_init = {0};

    RCC_APB2PeriphClockCmd(WS2812_SPI_GPIO_CLK, ENABLE);

    gpio_init.GPIO_Pin = WS2812_SPI_SCK_PIN | WS2812_SPI_MOSI_PIN;
    gpio_init.GPIO_Speed = GPIO_Speed_50MHz;
    gpio_init.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(WS2812_SPI_SCK_PORT, &gpio_init);

    gpio_init.GPIO_Pin = WS2812_SPI_MISO_PIN;
    gpio_init.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(WS2812_SPI_MISO_PORT, &gpio_init);
}

static void WS2812_SPI_InitPeripheral(void)
{
    SPI_InitTypeDef spi_init = {0};

    RCC_APB2PeriphClockCmd(WS2812_SPI_CLK, ENABLE);

    spi_init.SPI_Direction = SPI_Direction_1Line_Tx;
    spi_init.SPI_Mode = SPI_Mode_Master;
    spi_init.SPI_DataSize = SPI_DataSize_8b;
    spi_init.SPI_CPOL = SPI_CPOL_Low;
    spi_init.SPI_CPHA = SPI_CPHA_1Edge;
    spi_init.SPI_NSS = SPI_NSS_Soft;
    spi_init.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;
    spi_init.SPI_FirstBit = SPI_FirstBit_MSB;
    spi_init.SPI_CRCPolynomial = 7;
    SPI_Init(WS2812_SPI, &spi_init);
    SPI_Cmd(WS2812_SPI, ENABLE);
}

static void WS2812_EncodeByte(uint8_t value, uint8_t *dst)
{
    uint8_t bit_index;

    for (bit_index = 0; bit_index < 8U; bit_index++)
    {
        dst[bit_index] = ((value & 0x80U) != 0U) ? WS2812_SPI_PATTERN_1 : WS2812_SPI_PATTERN_0;
        value <<= 1;
    }
}

static void WS2812_BuildFrame(void)
{
    uint16_t offset = 0U;
    uint8_t led_index;

    for (offset = 0U; offset < WS2812_RESET_SLOTS; offset++)
    {
        g_spi_tx_buffer[offset] = 0x00U;
    }

    offset = WS2812_RESET_SLOTS;
    for (led_index = 0U; led_index < WS2812_LED_COUNT; led_index++)
    {
        WS2812_EncodeByte(g_led_buffer[led_index].g, &g_spi_tx_buffer[offset]);
        offset += 8U;
        WS2812_EncodeByte(g_led_buffer[led_index].r, &g_spi_tx_buffer[offset]);
        offset += 8U;
        WS2812_EncodeByte(g_led_buffer[led_index].b, &g_spi_tx_buffer[offset]);
        offset += 8U;
    }

    while (offset < WS2812_TX_BUFFER_SIZE)
    {
        g_spi_tx_buffer[offset] = 0x00U;
        offset++;
    }
}

static void WS2812_SPI_SendBytes(const uint8_t *data, uint16_t length)
{
    uint16_t index;

    for (index = 0U; index < length; index++)
    {
        while (SPI_I2S_GetFlagStatus(WS2812_SPI, SPI_I2S_FLAG_TXE) == RESET)
        {
        }
        SPI_I2S_SendData(WS2812_SPI, data[index]);
    }

    while (SPI_I2S_GetFlagStatus(WS2812_SPI, SPI_I2S_FLAG_BSY) != RESET)
    {
    }
}

void WS2812_Init(void)
{
    WS2812_GPIO_Init();
    WS2812_SPI_InitPeripheral();
    WS2812_Clear();
    WS2812_Refresh();
}

void WS2812_Clear(void)
{
    uint8_t i;

    for (i = 0U; i < WS2812_LED_COUNT; i++)
    {
        g_led_buffer[i].g = 0U;
        g_led_buffer[i].r = 0U;
        g_led_buffer[i].b = 0U;
    }
}

void WS2812_SetPixel(uint8_t index, uint8_t r, uint8_t g, uint8_t b)
{
    if (index >= WS2812_LED_COUNT)
    {
        return;
    }

    g_led_buffer[index].r = r;
    g_led_buffer[index].g = g;
    g_led_buffer[index].b = b;
}

void WS2812_SetAll(const uint8_t *grb_data, uint8_t byte_count)
{
    uint8_t led_index;
    uint8_t data_index = 0U;

    if ((grb_data == 0) || (byte_count < WS2812_GRB_BYTES))
    {
        return;
    }

    for (led_index = 0U; led_index < WS2812_LED_COUNT; led_index++)
    {
        g_led_buffer[led_index].g = grb_data[data_index++];
        g_led_buffer[led_index].r = grb_data[data_index++];
        g_led_buffer[led_index].b = grb_data[data_index++];
    }
}

void WS2812_Refresh(void)
{
    WS2812_BuildFrame();
    WS2812_SPI_SendBytes(g_spi_tx_buffer, WS2812_TX_BUFFER_SIZE);
}
