# flight-range-finder

An ESP32-S3 SuperMini (or a Seeed XIAO ESP32-S3) reads a VL53L1X laser over I2C and sends each reading to the flight controller as a 12-byte missionlink DB frame (command 0x17) on its UART the moment it is read (115200 8N1, about 40 readings/s, SHORT mode: reaches about 1.3 m). The frame, timing, LED and edge cases are in [SPEC.md](SPEC.md).

## Wiring
| From | SuperMini (`--board mini`, the default) | XIAO (`--board xiao`) |
|---|---|---|
| VL53L1X VIN | 3V3 | 3V3 |
| VL53L1X GND | GND | GND |
| VL53L1X SDA | GP1 | D4 |
| VL53L1X SCL | GP2 | D5 |
| VL53L1X GPIO1 (optional) | GP4, build with `--int-gpio 4` | D0, build with `--int-gpio 1` |
| board TX to flight controller J19 pin 1 (RX1, USART1) | TX | D6 |
| board GND to flight controller J19 pin 3 (GND) | GND | GND |
| board power (J19 has no power pin) | 5 V on USB-C or the 5V pin | 5 V on USB-C or the 5V pin |

## LED (SuperMini: the RGB LED; XIAO: the orange LED)
| LED | Meaning |
|---|---|
| full, steady (SuperMini: green) | just booted |
| one blink per second (SuperMini: red) | no sensor (not wired, or not answering); the board sets the sensor up again every half second, so a re-plugged sensor starts by itself |
| fast flashing, one flash per reading (40/s), brighter the farther (SuperMini: green) | working: dim up close, full from 1 m on (every reading, valid or not, shows its range) |
| off | readings stopped |

Details: SPEC.md Status LED.

## Build, flash, watch
One script, from any folder (it loads ESP-IDF itself):
```bash
/Users/bao/skydev/flight-range-finder/build.sh --list
/Users/bao/skydev/flight-range-finder/build.sh --flash
/Users/bao/skydev/flight-range-finder/build.sh --flash --board xiao
/Users/bao/skydev/flight-range-finder/build.sh --flash --device /dev/cu.usbmodem101
/Users/bao/skydev/flight-range-finder/build.sh --build
```
`--board mini` (the default) is the ESP32-S3 SuperMini, `--board xiao` the Seeed XIAO ESP32-S3; they differ only in the I2C and LED pins.
`--list` shows each USB serial device and what it is (an ESP32-S3 shows as Espressif USB JTAG; its serial is the board's MAC). `--flash` builds if needed and flashes the one USB serial device, or the one `--device` names. No option prints the usage.
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
- `--flash` refuses a port another program holds (it names it), and after a failed flash prints the cause and what to do: for a board that keeps restarting or stops responding, hold BOOT (B), tap RESET (R), release BOOT, and run it again.
- Every frame also goes over USB by default, so the Mac can watch; it costs the UART nothing. `--usb-frames 0` keeps USB for the boot text only.
- ESP-IDF comes from `~/skydev-research/esp/esp-idf` (or `$IDF_PATH`) with its installed Python environment, so a shell whose `python3` is a newer version still builds.
- A new board, or one whose program does not keep USB alive, shows no port: the command then waits 30 s; hold BOOT (B) while plugging the USB in and it flashes. After flashing from BOOT, press RESET (R) if the LED does not start. Still nothing: the cable may be charge-only, or try another USB port.
- `view_lidar.py` without `--each` prints once a second: frames/s, sequence gaps, bad checksums, status counts, range min/mean/max (mm), frame interval p50/p99 (ms).
- Test the decoder: `python3 tools/test_view_lidar.py`.
