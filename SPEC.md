# xiao_lidar SPEC

## Card
- Purpose: VL53L1X range to bytes on a UART with the least latency from range-ready to the wire.
- Inputs: VL53L1X on I2C (0x29); optional data-ready interrupt on D0.
- Outputs: 8-byte frames on UART0 TX (GPIO43, 921600 8N1), one per reading; one boot text line, then (default) the same frames, on USB-Serial-JTAG; the user LED (Status LED).
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

## Status LED (owner 2026-11-21: see the laser working; the farther, the brighter)
LEDC PWM on GPIO21 (low-speed timer 0, channel 0, 5 kHz, 13-bit, output inverted for the active-low LED).
| LED | Meaning |
|---|---|
| full, steady | booted, no reading yet |
| brightness follows the range | a valid reading (status 0): duty = 80 + (8191 - 80) x (r / 4000 mm)^2, r clamped to 4000 mm; the square makes the eye see brightness grow about evenly with distance; 80 (1 %) keeps a close reading visibly lit |
| fast flicker, full / off | readings arrive but are not valid (out of range, signal fail, or 0xFF error): switches on every such frame, about 20 per second |
| frozen | readings stopped; a still target looks the same, so move a hand to check |
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
| `--usb-frames 0\|1` (LIDAR_USB_FRAMES) | 1 | 1 also writes each frame to USB-Serial-JTAG after the UART write (non-blocking, dropped without a host); 0 keeps USB for the boot line |
| `--flash [PORT]` | - | builds, then flashes over the XIAO's USB (`idf.py -p PORT build flash`); no PORT: the one `/dev/cu.usbmodem*` or `/dev/ttyACM*`, else refuses naming them |

`build.sh` runs from any folder; without `idf.py` on PATH it sources `$IDF_PATH` (default `~/skydev-research/esp/esp-idf`) `export.sh` with `IDF_PYTHON_ENV_PATH` set to the installed `~/.espressif/python_env/idf*_env` (a login shell's newer `python3` otherwise makes export.sh look for an env that does not exist).

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
| EC-L7 | I2C error (sensor unplugged) sends status 0xFF, range 0, at most one per 25 ms (so the LED flickers, not a 1 kHz glow) | bench only: SEEN on the board (Measured facts 2026-11-21) |
| EC-L8 | the LED follows the Status LED table: full at boot, brighter as a hand moves away, fast flicker on invalid readings | NOT tested (needs a board): bench, a hand at 5 cm then 1 m, pointed at a far ceiling, sensor unplugged |

## Measured facts
| Date | Condition | Fact | Source |
|---|---|---|---|
| 2026-11-21 | XIAO ESP32-S3 MAC 1c:db:d4:75:0b:b4, image db9dc9a `--usb-frames 1`, no VL53L1X fitted, frames read over USB on a Mac | every frame status 0xFF, range 0; 38-39 frames/s, interval p50 26.0 ms p99 26.2 ms, 0 seq gaps, 0 bad checksums; the LED flickers (owner) | `tools/view_lidar.py --port /dev/cu.usbmodem101 --seconds 4` |

## Open
- ESP32-S3 ROM prints boot text on GPIO43 at power-on; the receiver must resync (sync bytes + checksum).
- Ranging settings are untuned for flight.
- No reading from a fitted VL53L1X seen yet: ranging, valid status, the LED brightness and the range-to-wire delay are NOT verified running.
