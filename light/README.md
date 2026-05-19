# light

`CH32V003F4P6` example project scaffold for:

- `USART1` receive external control messages
- convert color payload into `SPI1` waveform
- drive `10` cascaded `WS2812`

## Pin Definition

- `PD1`: `SWIO` debug
- `PD7`: `NRST`
- `PD6`: `USART1_RX`
- `PD5`: `USART1_TX` optional debug
- `PC5`: `SPI1_SCK`
- `PC6`: `SPI1_MOSI` and `WS2812 DIN`
- `PC7`: `SPI1_MISO` unused

## Suggested UART Protocol

Frame format:

```text
0xAA 0x55 [LEN] [CMD] [DATA ...] [CHECKSUM]
```

- `LEN`: byte count of `CMD + DATA`
- `CMD = 0x01`: set all 10 LEDs, payload is `30` bytes in `GRB` order
- `CHECKSUM`: sum of `LEN`, `CMD`, and all data bytes, low 8-bit only

Example:

- header: `AA 55`
- len: `1E + 01 = 1F`
- cmd: `01`
- data: `G1 R1 B1 ... G10 R10 B10`

## Files

- `Inc/board_config.h`: board and pin definition
- `Inc/ws2812.h`: WS2812 driver interface
- `Inc/uart_proto.h`: UART protocol parser interface
- `Src/main.c`: startup example
- `Src/ws2812.c`: SPI-based WS2812 encoder and sender
- `Src/uart_proto.c`: UART frame parser

## Notes

- This scaffold assumes you already have the WCH `ch32v00x` standard peripheral library in your build path.
- `WS2812` is powered by `5V`; this design expects the MCU to also run at `5V`.
- Put a `220R~470R` resistor in series with `PC6 -> DIN`.
- Add a bulk capacitor near LED power input.
