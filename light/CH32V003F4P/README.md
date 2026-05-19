# CH32V003F4P WS2812 Light

## Pinout

- `PD5`: `USART1_TX`
- `PD6`: `USART1_RX`
- `PC6`: `SPI1_MOSI` -> `WS2812 DIN`
- `PC5`: `SPI1_SCK`
- `PD1`: `SWIO`

## Serial Port

- UART: `USART1`
- Baud: `115200`
- Format: `8N1`
- RX pin: `PD6`
- TX pin: `PD5`
- `PD5` only sends data out
- `PD6` only receives data in
- keep `CTRL_UART_USE_PARTIAL_REMAP2 = 0` for `USB-TTL TX -> PD6`

## UART Frame

Important:

- send raw hex bytes, not ASCII text
- `AA 55` means two bytes `0xAA 0x55`
- do not send the characters `A`, `A`, space, `5`, `5`

Receive frame:

```text
AA 55 + 30 bytes RGB + 55 AA
```

- `30` bytes = `10` LEDs x `R G B`
- byte order is `R`, `G`, `B`
- full frame length is `34` bytes

Example:

```text
AA 55 FF 00 00 00 FF 00 00 00 FF FF FF FF 00 00 00 10 10 10 20 00 00 00 20 00 00 00 20 08 08 08 55 AA
```

The above means:

- LED1: red
- LED2: green
- LED3: blue
- LED4: white
- LED5: off
- LED6: dim white
- LED7: weak red
- LED8: weak green
- LED9: weak blue
- LED10: dark white

If the board is alive, you should see:

- startup `RUN` frames first
- after a valid command, one `ACK` frame immediately
- then periodic `RUN` frames at `50Hz`

If you only see `RUN` frames and never `ACK`, the usual causes are:

1. `PD6` is not really connected to the adapter `TX`
2. the sender is typing ASCII instead of raw hex bytes
3. the host tool is not actually transmitting the frame

More one-line examples:

```text
AA 55 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 55 AA
```

```text
AA 55 40 00 00 00 40 00 00 40 00 00 00 40 00 00 00 40 00 00 40 00 00 00 40 00 00 00 40 00 00 55 AA
```

```text
AA 55 FF 00 00 00 FF 00 00 FF 00 00 FF 00 00 FF 00 00 FF 00 00 FF 00 00 FF 00 00 FF 00 00 FF 00 55 AA
```

## LED Control Commands

One LED uses 3 bytes:

- `R`
- `G`
- `B`

Ten LEDs means `30` bytes total.

Examples:

- LED1 red, others off

```text
AA 55 FF 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 55 AA
```

- LED1 green, LED2 blue, others off

```text
AA 55 00 FF 00 00 00 FF 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 55 AA
```

- all LEDs white at half brightness

```text
AA 55 80 80 80 80 80 80 80 80 80 80 80 80 80 80 80 80 80 80 80 80 80 80 80 80 80 80 80 80 80 80 55 AA
```

- all LEDs off

```text
AA 55 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 55 AA
```

Byte order reminder:

- LED1: bytes 1-3
- LED2: bytes 4-6
- ...
- LED10: bytes 28-30

## Return Frames

ACK frame:

```text
AA 55 81 03 [status] [mode] [payload_len] 55 AA
```

- `status = 0`: success
- `status = 1`: frame error
- `mode = 0`: boot flow
- `mode = 1`: control mode
- `payload_len = 30`
- on success, this is sent immediately after a valid command

ACK example:

```text
AA 55 81 03 00 01 1E 55 AA
```

RUN frame:

```text
AA 55 82 24 [mode] [led_count] [sysclk4] [rgb30] 55 AA
```

- sent at `50Hz`
- also sent once after a valid command
- `sysclk4` is little-endian `SystemCoreClock`
- `rgb30` is the current `10` LEDs RGB state

RUN example layout:

```text
AA 55 82 24 01 0A 00 00 00 00 [30 bytes RGB] 55 AA
```

## Boot Behavior

- Power on starts with a flowing rainbow-style boot animation
- After the first valid UART frame arrives, the app switches to control mode
- Control mode stops the boot animation and uses received RGB data

## Troubleshooting

If no serial data is visible:

1. Check `PD5/PD6` wiring and common GND.
2. Confirm UART is `115200, 8N1`.
3. Confirm the sender is adding `AA 55` and `55 AA`.
4. Make sure `USART1` is connected to the correct USB-TTL adapter pins.
5. If the board is powered by `5V`, ensure the adapter UART voltage matches the setup.

## Source Files

- `User/main.c`
- `User/ws2812.c`
- `User/ch32v00x_it.c`
- `User/board_config.h`
