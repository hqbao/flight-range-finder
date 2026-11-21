# xiao_lidar SPEC

## Card
- Purpose: VL53L1X range to bytes on a UART with the least latency from range-ready to the wire.
- Inputs: VL53L1X on I2C (0x29); optional data-ready interrupt on D0.
- Outputs: 8-byte frames on UART0 TX (GPIO43, 921600 8N1), one per reading; one boot text line on USB-Serial-JTAG; the user LED (Status LED).
- Invariants: one task, core 1; frame is written to the UART before anything else after the I2C result read; no per-frame logging; the frame UART carries no text; seq +1 per frame (wraps); an I2C error still sends a frame (status 0xFF, range 0), at most one per 25 ms.
- Code: `main/main.c`; driver `libs/vl53l1x` (ST API, unchanged); host tools `tools/`.

## Pins (XIAO ESP32-S3)
| Signal | Pin | GPIO |
|---|---|---|
| I2C SDA | D4 | 5 |
| I2C SCL | D5 | 6 |
| Frame UART TX | D6 | 43 |
| Frame UART RX (unused) | D7 | 44 |
| VL53L1X GPIO1 (ISR mode only) | D0 | 1 |
| User LED (on board, on when low; Seeed XIAO ESP32-S3 wiki, NOT verified on this board) | - | 21 |

## Status LED (owner 2026-11-21: see at a glance that ranges are coming)
| LED | Meaning |
|---|---|
| steady on | booted, no reading yet (init, or the loop or sensor stuck) |
| slow blink, 2 per second | valid readings (status 0): toggles every 10th valid frame, at 40 frames/s |
| fast flicker, about 20 per second | readings arrive but are not valid (out of range, signal fail, or 0xFF error): toggles on every such frame |
| frozen on or off after blinking | readings stopped |
The LED is set after the frame is written, so it adds nothing to the frame's delay.

Console and logs use USB-Serial-JTAG (`CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG`); bootloader log level none.

## Ranging settings (proven on hardware in flight-optflow `range_finder.c`, kept exactly; constants at the top of `main.c`)
LONG distance mode; timing budget 20000 us; inter-measurement 25 ms; ROI 16x16 (TopLeft 0,15; BotRight 15,0); I2C 400 kHz, internal pull-ups, 7-bit 0x29 (API address 0x52).

## Frame (8 bytes)
| Byte | Field |
|---|---|
| 0,1 | 0xA5, 0x5A |
| 2 | seq u8 |
| 3 | status: RangeStatus as returned (0 valid); 0xFF I2C/driver error |
| 4,5 | range_mm u16 little-endian |
| 6,7 | ck_a, ck_b: for b in bytes 2..5: ck_a += b; ck_b += ck_a (mod 256) |

## Latency budget (computed, NOT measured)
- Integration: 20 ms budget; one reading per 25 ms.
- Poll mode (default): data-ready is seen after <= 1 ms (vTaskDelay(1), FREERTOS_HZ 1000) plus one status read; then the result read (about 20 bytes at 400 kHz, about 0.5 ms computed); then `uart_write_bytes` copies 8 bytes into the 128-byte FIFO (TX ring buffer 0); serialisation 8 x 10 bit / 921600 = 87 us.
- ISR mode (`LIDAR_INT_GPIO >= 0`): the 1 ms poll wait is replaced by ISR + task-notify wake; the rest is the same. Edge follows the sensor's interrupt polarity read from the device.
- Not measured: ISR wake time, total range-ready to first byte on the wire.

## Build options (`./build.sh`)
| Option | Default | Meaning |
|---|---|---|
| `--int-gpio N` (LIDAR_INT_GPIO) | -1 | GPIO wired to VL53L1X GPIO1; -1 polls |
| `--usb-frames 0\|1` (LIDAR_USB_FRAMES) | 0 | 1 also writes each frame to USB-Serial-JTAG after the UART write (non-blocking, dropped without a host) |

Boot line (USB): `xiao_lidar mac=.. mode=LONG budget_us=.. imp_ms=.. roi=16x16 baud=.. int_gpio=.. usb_frames=..`

## Edge cases
| ID | Case | Test |
|---|---|---|
| EC-L1 | Encoded frames decode to the same seq, status, range | `test_view_lidar.py::test_ec_l1_roundtrip` |
| EC-L2 | Bad checksum is rejected and counted | `test_ec_l2_bad_checksum` |
| EC-L3 | Garbage before and between frames is skipped; decoder resyncs | `test_ec_l3_resync` |
| EC-L4 | seq 255 -> 0 is not a gap | `test_ec_l4_seq_wrap` |
| EC-L5 | A 2-frame gap is counted as 2 | `test_ec_l5_gap` |
| EC-L6 | A frame split across reads decodes | `test_ec_l6_split_feed` |
| EC-L7 | I2C error (sensor unplugged) sends status 0xFF, range 0, at most one per 25 ms (so the LED flickers, not a 1 kHz glow) | NOT tested (needs a board); NOT verified running |
| EC-L8 | the LED follows the Status LED table: steady on at boot, 2 Hz blink on valid readings, fast flicker on invalid ones | NOT tested (needs a board): bench, sensor covered / pointed at the ceiling / unplugged |

## Open
- ESP32-S3 ROM prints boot text on GPIO43 at power-on; the receiver must resync (sync bytes + checksum).
- Ranging settings are untuned for flight.
- Firmware has not been flashed or run: no board attached.
