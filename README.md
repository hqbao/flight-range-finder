# flight-range-finder

A Seeed XIAO ESP32-S3 reads a VL53L1X laser over I2C and sends each reading to the flight controller as a 12-byte missionlink DB frame (command 0x17) on its UART the moment it is read (921600 8N1, about 40 readings/s). The frame, timing, LED and edge cases are in [SPEC.md](SPEC.md).

## Wiring
| From | To |
|---|---|
| VL53L1X VIN | XIAO 3V3 |
| VL53L1X GND | XIAO GND |
| VL53L1X SDA | XIAO D4 |
| VL53L1X SCL | XIAO D5 |
| VL53L1X GPIO1 (optional, build with `--int-gpio 1`) | XIAO D0 |
| XIAO D6 (TX) | flight controller J19 pin 1 (RX1, USART1) |
| XIAO GND | flight controller J19 pin 3 (GND) |
| XIAO power | 5 V on USB-C or the 5V pin (J19 has no power pin) |

## LED (orange, on the XIAO)
| LED | Meaning |
|---|---|
| full, steady | just booted |
| one blink per second | no sensor (not wired, or not answering); the XIAO sets the sensor up again every half second, so a re-plugged sensor starts by itself |
| fast flashing, one flash per reading (40/s), brighter the farther | working: dim up close, full at 4 m |
| fast flashing at full brightness | working, but nothing within reach (pointed at the sky or too far) |
| off | readings stopped |

Details: SPEC.md Status LED.

## Build, flash, watch
One script, from any folder (it loads ESP-IDF itself):
```bash
/Users/bao/skydev/flight-range-finder/build.sh --list
/Users/bao/skydev/flight-range-finder/build.sh --flash
/Users/bao/skydev/flight-range-finder/build.sh --flash --device /dev/cu.usbmodem101
/Users/bao/skydev/flight-range-finder/build.sh --build
```
`--list` shows each USB serial device and what it is (a XIAO shows as Espressif USB JTAG; its serial is the board's MAC). `--flash` builds if needed and flashes the one USB serial device, or the one `--device` names. No option prints the usage.
Watch the ranges:
```bash
python3 /Users/bao/skydev/flight-range-finder/tools/view_lidar.py --port /dev/cu.usbmodem101
```
It runs until Ctrl+C (`--seconds N` stops it after N s). Every reading on its own line (time, seq, range, status and its meaning):
```bash
python3 /Users/bao/skydev/flight-range-finder/tools/view_lidar.py --port /dev/cu.usbmodem101 --each
```
- The viewer keeps waiting through a board restart or a re-plug and carries on. Only one program can use the port at a time: stop the viewer (Ctrl+C) before `--flash`.
- With more than one USB serial device plugged in, `--flash` lists them and asks for `--device`.
- Every frame also goes over USB by default, so the Mac can watch; it costs the UART nothing. `--usb-frames 0` keeps USB for the boot text only.
- ESP-IDF comes from `~/skydev-research/esp/esp-idf` (or `$IDF_PATH`) with its installed Python environment, so a shell whose `python3` is a newer version still builds.
- A new XIAO, or one whose program does not keep USB alive, shows no port: the command then waits 30 s; hold BOOT (B) while plugging the USB in and it flashes. After flashing from BOOT, press RESET (R) if the LED does not start. Still nothing: the cable may be charge-only, or try another USB port.
- `view_lidar.py` without `--each` prints once a second: frames/s, sequence gaps, bad checksums, status counts, range min/mean/max (mm), frame interval p50/p99 (ms).
- Test the decoder: `python3 tools/test_view_lidar.py`.
