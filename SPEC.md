# xiao_lidar SPEC

## Card
- Purpose: VL53L1X range to bytes on a UART with the least latency from range-ready to the wire.
- Inputs: VL53L1X on I2C (0x29); optional data-ready interrupt on D0.
- Outputs: 12-byte missionlink DB frames (cmd 0x17) on UART0 TX (GPIO43, 115200 8N1: owner 2026-11-21, every flight controller UART at 115200), one per reading; one boot text line, then (default) the same frames, on USB-Serial-JTAG; the user LED (Status LED).
- Invariants: one task, core 1; the frame is written to the UART right after the 15-byte result read (Result read), before the API's own read; a failed I2C read is retried once at once; no per-frame logging; the frame UART carries no text; seq +1 per frame (wraps); an I2C error, or no reading for 100 ms, still sends a frame (status 0xFF, range 0), at most one per 25 ms; after 500 ms without a good frame the sensor is set up again (Sensor restart).
- Code: `main/main.c`; driver `libs/vl53l1x` (ST API core unchanged; ESP32 port `platform/src/vl53l1_platform.c`, I2C timeout 10 ms); host tools `tools/`.

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
| one 10 ms flash per reading (40 per second), brightness by range | every reading the sensor returns, whatever its status (owner 2026-11-21: the farther, the brighter, full from 1 m): duty = 80 + (8191 - 80) x (r / 1000 mm)^2, r clamped to 1000 mm; the square makes the eye see brightness grow about evenly with distance; 80 (1 %) keeps a close reading visibly lit; a not-valid reading shows its reported range too (a full flash for it made a near status 7 look brighter than a far valid reading) |
| off | readings stopped (the last flash timed out) |
The LED is set after the frame is written, so it adds nothing to the frame's delay.

Console and logs use USB-Serial-JTAG (`CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG`); bootloader log level none.

## Ranging settings (owner 2026-11-21: SHORT 20 ms; constants at the top of `main.c`)
SHORT distance mode (about 1.3 m reach, ST datasheet, NOT measured here; beyond it the sensor reports status 4); timing budget 20000 us; inter-measurement 25 ms; ROI 16x16 (TopLeft 0,15; BotRight 15,0); I2C 400 kHz, internal pull-ups, 7-bit 0x29 (API address 0x52). LONG mode at 20 ms (flight-optflow's setting, which accepts every status except 2) flagged 99 % of readings status 7 on a wall at 0.77 m; ST's API manual allows 20 ms only in SHORT mode (33 ms minimum for LONG); LONG 33 ms worked (96.5 % valid) but costs 25 readings/s and about 13 ms more age (Measured facts).

## Result read (owner 2026-11-21: read only what the frame needs, send it, then let the API do its bookkeeping)
- On data-ready the loop reads 15 bytes from 0x0089 (RESULT__RANGE_STATUS .. RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0) and computes status and range exactly as `VL53L1_GetRangingMeasurementData` does (`vl53l1_api_core.c` `VL53L1_copy_sys_and_core_results_to_range_results`, `vl53l1_api.c` `SetSimpleData`, `ConvertStatusLite`):
  - device status 1, 2, 3, 17 -> 5 (hardware fail); 13 -> 13 (min range fail); 9 (range complete) with stream count 0 -> 6; every other code through ConvertStatusLite; an unknown code -> 255 (the API's NONE, sent like an error).
  - range = int16((raw x standard_ranging_gain_factor + 0x400) / 0x800); a valid reading below 0 becomes status 14 below -30 mm (TUNING_PROXY_MIN), else range 0; the wire carries the int16 as u16.
- Then the frame goes out, then `VL53L1_GetRangingMeasurementData` runs (its result is not used): in LOWPOWER_AUTONOMOUS it sets up the manual calibration after the first range and updates the SPAD selection every range, which the next range uses; then the interrupt is cleared.

## Sensor restart (owner 2026-11-21: a sensor that answered late never started)
`sensor_start()` runs the whole setup (boot wait, `VL53L1_software_reset`, DataInit, StaticInit, the Ranging settings, StartMeasurement) and stops at the first error. The reset is needed because the sensor keeps power, and the calibration it saved while ranging, across an ESP32 reset or restart: set up in another mode on top of that, it gave status 4 on every reading (Measured facts). app_main calls it once and prints its result in the boot line (`sensor_start=0` is ok). The loop sends a 0xFF frame when an I2C read fails or when no reading has been ready for 100 ms (READY_TIMEOUT_US: 4 periods), waits 25 ms after each 0xFF frame, and calls `sensor_start()` again once 500 ms (RESTART_AFTER_US) have passed since the last frame that was not 0xFF, after `i2c_master_bus_reset` (9 clock pulses and a controller reset, which frees a sensor holding SDA low). Every I2C transfer gives up after 10 ms (`I2C_TIMEOUT_MS`; the longest, 134 bytes, takes about 3.1 ms; was 1 s, which stopped frames for a second on a stuck bus), and the ready check and the result read are each retried once at once, so one glitch costs no reading. A sensor that is unplugged, re-powered or answers only after boot therefore starts ranging by itself within about 0.5 s of answering; no reset is needed.

## Frame (12 bytes: a missionlink DB frame, owner 2026-11-21)
The flight controller's frame layout (`'d' 'b' cmd class len16 payload sum16`, flight-controller `modules/missionlink/missionlink_sender.c` and flight-vio `missionlink/check.py` `frame()`), command MISSIONLINK_CMD_LASER_RANGE 0x17 (the next free number; allocated in flight-controller `base/foundation/messages.h` by the h7v3 lidar_uart lane), class 0x00.
| Byte | Field |
|---|---|
| 0,1 | 'd' 'b' (0x64, 0x62) |
| 2 | cmd 0x17 |
| 3 | class 0x00 |
| 4,5 | payload length 4, u16 little-endian |
| 6 | seq u8 |
| 7 | status: RangeStatus as the ST API computes it (Result read; 0 valid); 0xFF I2C error or no result |
| 8,9 | range_mm u16 little-endian |
| 10,11 | sum16 little-endian: the 16-bit sum of bytes 2..9 |
Example: seq 1, status 0, range 258 mm = `64 62 17 00 04 00 01 00 02 01 1f 00` (`test_view_lidar.py` EC-L1, equal to flight-vio `frame(0x17, 0, ...)`).

## Latency budget (computed, NOT measured)
- Integration: 20 ms budget; one reading per 25 ms.
- Poll mode (default): data-ready is seen after <= 1 ms (vTaskDelay(1), FREERTOS_HZ 1000) plus one status read; then the 15-byte result read (about 0.45 ms computed at 400 kHz; the API's 134-byte read, about 3.1 ms, now follows the frame); then `uart_write_bytes` copies 12 bytes into the 128-byte FIFO (TX ring buffer 0); serialisation 12 x 10 bit / 115200 = 1.04 ms.
- ISR mode (`LIDAR_INT_GPIO >= 0`): the 1 ms poll wait is replaced by ISR + task-notify wake; the rest is the same. Edge follows the sensor's interrupt polarity read from the device.
- Not measured: ISR wake time, total range-ready to first byte on the wire.

## Build options (`./build.sh`)
| Option | Default | Meaning |
|---|---|---|
| `--int-gpio N` (LIDAR_INT_GPIO) | -1 | GPIO wired to VL53L1X GPIO1; -1 polls |
| `--usb-frames 0\|1` (LIDAR_USB_FRAMES) | 1 | 1 also writes each frame to USB-Serial-JTAG after the UART write (non-blocking, dropped without a host); 0 keeps USB for the boot line |
| `--build` / `--flash` / `--list` | - (one required; none prints the usage) | build only / build if needed then flash over USB (`idf.py -p PORT build flash`) / list the USB serial devices with vendor, product and USB serial (`tools/list_devices.py`: ioreg on macOS, /dev/serial/by-id on Linux; an ESP32-S3's USB serial is its MAC) |
| `--device PORT` | the one USB serial device | the port `--flash` uses; none plugged: waits 30 s asking for BOOT held while plugging in (a board whose program keeps no USB port); more than one: lists them and asks for `--device`; a port another program holds (lsof) is refused with its name; a failed flash prints esptool's fatal line and the fix for it |

`build.sh` runs from any folder; without `idf.py` on PATH it sources `$IDF_PATH` (default `~/skydev-research/esp/esp-idf`) `export.sh` with `IDF_PYTHON_ENV_PATH` set to the installed `~/.espressif/python_env/idf*_env` (a login shell's newer `python3` otherwise makes export.sh look for an env that does not exist).

Boot line (USB): `xiao_lidar mac=.. mode=SHORT budget_us=.. imp_ms=.. roi=16x16 baud=.. int_gpio=.. usb_frames=..`

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
| EC-L9 | the sensor does not answer at power-on (or is re-powered), then answers: readings start within about 0.5 s with no reset; a sensor that answers but is not ranging gives 0xFF after 100 ms, not silence | bench: the old image's silence (LED full, steady, 0 frames/s) SEEN 2026-11-21; the fixed image's recovery by itself NOT yet seen (it started ranging at its own boot) |
| EC-L10 | the Result read gives the same status and range as `VL53L1_GetRangingMeasurementData` on every reading | bench check image (`scratch/xiao-lidar/i2cdiag`, prints any difference): PASS 2026-11-21 (Measured facts) |
| EC-L11 | an ESP32 reset while the powered sensor was left ranging in another distance mode still gives valid readings | bench: PASS 2026-11-21 (Measured facts; without the reset, status 4 on every reading) |
| EC-L8 | the LED follows the Status LED table: full at boot, 1 blink/s with no sensor, a flash per reading brighter as a hand moves away, full flashes pointed at nothing, off when readings stop | bench by eye: no sensor (owner); with the sensor NOT yet seen |

## Measured facts
| Date | Condition | Fact | Source |
|---|---|---|---|
| 2026-11-21 11:3x | XIAO 34:85:18:ae:a6:d4, wall 0.77 m, sensor powered throughout, images flashed back to back | no reset: SHORT firmware after a LONG run gave status 4 on every reading (range 0); the LONG test image after a SHORT run gave status 4 on 103 of 103; SHORT after SHORT was 100 % valid. With `VL53L1_software_reset` in the setup: SHORT after a LONG run 100 % valid, 772-787 mm | `tools/view_lidar.py`, `scratch/xiao-lidar/i2cdiag/ab_read.py` |
| 2026-11-21 11:3x | XIAO MAC 34:85:18:ae:a6:d4, board fixed facing a wall at 0.77 m, test image switching three settings every 150 frames, 150 s | LONG 20 ms / 25 ms period: 1 % valid (99 % status 7); LONG 33 ms / 40 ms: 96.5 % valid, 771-783 mm; SHORT 20 ms / 25 ms: 100 % valid, 770-779 mm | `scratch/xiao-lidar/i2cdiag/ab_result_b2_wall.txt` |
| 2026-11-21 11:22 | XIAO MAC 34:85:18:ae:a6:d4 with its VL53L1X, check image (Result read then the API read on every reading), 90 s, scene moving | 3652 readings, 0 differences in status or range, 0 API read errors; statuses 0 (567), 2 (2453), 7 (632); Result read 493 us mean, 511 us max; API read 3259 us mean, 3416 us max: the frame leaves about 2.8 ms earlier | `scratch/xiao-lidar/i2cdiag/check_light_b2.log` |
| 2026-11-21 10:05 | same XIAO, restart-fix image, sensor re-wired by the owner (what changed: asked), still scene | status 0 on 157 of 168 frames (the rest 7) at 254-279 mm (mean 264), 24.0 ms apart, 0 gaps, 0 bad checksums | `tools/view_lidar.py --seconds 4`, `scratch/xiao-lidar/tune/restart_img.log` |
| 2026-11-21 10:0x (before 10:05) | same XIAO, image 8f370c0, after the sensor was unplugged and re-powered | 0xFF at 39/s while it did not answer; then 0 frames/s and the LED full and steady once it answered (setup ran only at power-on, so it never started) | `scratch/xiao-lidar/tune/`, owner ("LED solid, no blinking") |
| 2026-11-21 | same XIAO, VL53L1X fitted, image 8f370c0, board still on the desk (scene not changed between runs) | LONG 20 ms: status 7 (wrap target fail) on 125 of 126 frames at 467-499 mm; a SHORT 20 ms test image on the same scene: status 7 on 211 of 211 frames at 380-389 mm. Two modes give two different flagged distances for one still scene: the sensor's own result is inconsistent (NOT physics; cause not yet confirmed) | `tools/view_lidar.py`, logs `scratch/xiao-lidar/tune/` |
| 2026-11-21 | same, a hand held 5-50 cm over the sensor | status 0 on 25-65 % of frames at 49-182 mm, the rest status 7; then 7 s of status 0xFF (no I2C answer) that recovered without a restart | `scratch/xiao-lidar/hand_test.log` |
| 2026-11-21 | XIAO ESP32-S3 MAC 1c:db:d4:75:0b:b4, image db9dc9a `--usb-frames 1`, no VL53L1X fitted, frames read over USB on a Mac | every frame status 0xFF, range 0; 38-39 frames/s, interval p50 26.0 ms p99 26.2 ms, 0 seq gaps, 0 bad checksums; the LED flickers (owner) | `tools/view_lidar.py --port /dev/cu.usbmodem101 --seconds 4` |

## Open
- ESP32-S3 ROM prints boot text on GPIO43 at power-on; the receiver must resync (sync bytes + checksum).
- Ranging settings are untuned for flight.
- The range is NOT checked against a ruler; the SHORT reach (about 1.3 m) and the LED brightness are NOT verified; the range-to-wire delay is NOT measured end to end.
- I2C runs on the ESP32's internal pull-ups only (about 45 kOhm, too weak for 400 kHz unless the sensor board has its own); the 7 s 0xFF dropout is NOT root-caused.
