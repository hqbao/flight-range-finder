# xiao_lidar SPEC

## Card
- Purpose: VL53L1X range to bytes on a UART with the least latency from range-ready to the wire.
- Inputs: VL53L1X on I2C (0x29); optional data-ready interrupt on D0.
- Outputs: 12-byte missionlink DB frames (cmd 0x17) on UART0 TX (GPIO43, 921600 8N1), one per reading; one boot text line, then (default) the same frames, on USB-Serial-JTAG; the user LED (Status LED).
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

## Status LED (owner 2026-11-21: no sensor blinks once a second; a working sensor flashes once per reading, brighter the farther)
LEDC PWM on GPIO21 (low-speed timer 0, channel 0, 5 kHz, 13-bit, output inverted for the active-low LED); a one-shot esp_timer turns each reading's flash off.
| LED | Meaning |
|---|---|
| full, steady | booted, no frame yet |
| one blink per second (full 0.5 s, off 0.5 s) | no sensor: status 0xFF frames (I2C error); toggles when 500 ms have passed since the last toggle |
| one 10 ms flash per reading (40 per second), brightness by range | a valid reading (status 0): duty = 80 + (8191 - 80) x (r / 4000 mm)^2, r clamped to 4000 mm; the square makes the eye see brightness grow about evenly with distance; 80 (1 %) keeps a close reading visibly lit |
| one 10 ms flash per reading at full brightness | the sensor answers but the reading is not valid (out of range, signal fail): nothing within reach reads as farthest |
| off | readings stopped (the last flash timed out) |
The LED is set after the frame is written, so it adds nothing to the frame's delay.

Console and logs use USB-Serial-JTAG (`CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG`); bootloader log level none.

## Ranging settings (proven on hardware in flight-optflow `range_finder.c`, kept exactly; constants at the top of `main.c`)
LONG distance mode; timing budget 20000 us; inter-measurement 25 ms; ROI 16x16 (TopLeft 0,15; BotRight 15,0); I2C 400 kHz, internal pull-ups, 7-bit 0x29 (API address 0x52).

## Frame (12 bytes: a missionlink DB frame, owner 2026-11-21)
The flight controller's frame layout (`'d' 'b' cmd class len16 payload sum16`, flight-controller `modules/missionlink/missionlink_sender.c` and flight-vio `missionlink/check.py` `frame()`), command MISSIONLINK_CMD_LASER_RANGE 0x17 (the next free number; allocated in flight-controller `base/foundation/messages.h` by the h7v3 lidar_uart lane), class 0x00.
| Byte | Field |
|---|---|
| 0,1 | 'd' 'b' (0x64, 0x62) |
| 2 | cmd 0x17 |
| 3 | class 0x00 |
| 4,5 | payload length 4, u16 little-endian |
| 6 | seq u8 |
| 7 | status: RangeStatus as returned (0 valid); 0xFF I2C/driver error |
| 8,9 | range_mm u16 little-endian |
| 10,11 | sum16 little-endian: the 16-bit sum of bytes 2..9 |
Example: seq 1, status 0, range 258 mm = `64 62 17 00 04 00 01 00 02 01 1f 00` (`test_view_lidar.py` EC-L1, equal to flight-vio `frame(0x17, 0, ...)`).

## Latency budget (computed, NOT measured)
- Integration: 20 ms budget; one reading per 25 ms.
- Poll mode (default): data-ready is seen after <= 1 ms (vTaskDelay(1), FREERTOS_HZ 1000) plus one status read; then the result read (about 20 bytes at 400 kHz, about 0.5 ms computed); then `uart_write_bytes` copies 12 bytes into the 128-byte FIFO (TX ring buffer 0); serialisation 12 x 10 bit / 921600 = 130 us.
- ISR mode (`LIDAR_INT_GPIO >= 0`): the 1 ms poll wait is replaced by ISR + task-notify wake; the rest is the same. Edge follows the sensor's interrupt polarity read from the device.
- Not measured: ISR wake time, total range-ready to first byte on the wire.

## Build options (`./build.sh`)
| Option | Default | Meaning |
|---|---|---|
| `--int-gpio N` (LIDAR_INT_GPIO) | -1 | GPIO wired to VL53L1X GPIO1; -1 polls |
| `--usb-frames 0\|1` (LIDAR_USB_FRAMES) | 1 | 1 also writes each frame to USB-Serial-JTAG after the UART write (non-blocking, dropped without a host); 0 keeps USB for the boot line |
| `--build` / `--flash` / `--list` | - (one required; none prints the usage) | build only / build if needed then flash over USB (`idf.py -p PORT build flash`) / list the USB serial devices with vendor, product and USB serial (`tools/list_devices.py`: ioreg on macOS, /dev/serial/by-id on Linux; an ESP32-S3's USB serial is its MAC) |
| `--device PORT` | the one USB serial device | the port `--flash` uses; none plugged: waits 30 s asking for BOOT held while plugging in (a board whose program keeps no USB port); more than one: lists them and asks for `--device` |

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
| EC-L8 | the LED follows the Status LED table: full at boot, 1 blink/s with no sensor, a flash per reading brighter as a hand moves away, full flashes pointed at nothing, off when readings stop | bench by eye: no sensor (owner); with the sensor NOT yet seen |

## Measured facts
| Date | Condition | Fact | Source |
|---|---|---|---|
| 2026-11-21 | same XIAO, VL53L1X fitted, image 8f370c0, board still on the desk (scene not changed between runs) | LONG 20 ms: status 7 (wrap target fail) on 125 of 126 frames at 467-499 mm; a SHORT 20 ms test image on the same scene: status 7 on 211 of 211 frames at 380-389 mm. Two modes give two different flagged distances for one still scene: the sensor's own result is inconsistent (NOT physics; cause not yet confirmed) | `tools/view_lidar.py`, logs `scratch/xiao-lidar/tune/` |
| 2026-11-21 | same, a hand held 5-50 cm over the sensor | status 0 on 25-65 % of frames at 49-182 mm, the rest status 7; then 7 s of status 0xFF (no I2C answer) that recovered without a restart | `scratch/xiao-lidar/hand_test.log` |
| 2026-11-21 | XIAO ESP32-S3 MAC 1c:db:d4:75:0b:b4, image db9dc9a `--usb-frames 1`, no VL53L1X fitted, frames read over USB on a Mac | every frame status 0xFF, range 0; 38-39 frames/s, interval p50 26.0 ms p99 26.2 ms, 0 seq gaps, 0 bad checksums; the LED flickers (owner) | `tools/view_lidar.py --port /dev/cu.usbmodem101 --seconds 4` |

## Open
- ESP32-S3 ROM prints boot text on GPIO43 at power-on; the receiver must resync (sync bytes + checksum).
- Ranging settings are untuned for flight.
- A fitted VL53L1X answers and ranges, but almost every reading is status 7 (Measured facts); valid ranging, the LED brightness and the range-to-wire delay are NOT verified running.
- I2C runs on the ESP32's internal pull-ups only (about 45 kOhm, too weak for 400 kHz unless the sensor board has its own); the 7 s 0xFF dropout is NOT root-caused.
