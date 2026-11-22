#!/bin/bash
usage() {
    cat <<'EOF'
usage: ./build.sh --build | --flash | --list [--board mini|xiao] [--device PORT] [--usb-frames 0|1] [--int-gpio N] [--clean]
  --build          build only
  --flash          build if needed, then flash over USB
  --list           list the USB serial devices (an ESP32-S3 shows as Espressif USB JTAG, serial = its MAC)
  --board mini|xiao  ESP32-S3 SuperMini (default) or Seeed XIAO ESP32-S3: sets the I2C and LED pins
  --device PORT    the port to flash (default: the one USB serial device; none: waits 30 s for BOOT + plug in)
  --usb-frames 0|1 also send each frame over USB (default 1)
  --int-gpio N     GPIO wired to the VL53L1X GPIO1 (default -1: poll)
  --clean          full clean first
EOF
}
set -e
cd "$(dirname "$0")"
ACTION=""; BOARD=mini; INT=-1; USB=1; PORT=""; CLEAN=0
while [[ $# -gt 0 ]]; do
    case "$1" in
        --build) ACTION=build; shift ;;
        --flash) ACTION=flash; shift ;;
        --list) ACTION=list; shift ;;
        --board) BOARD="$2"; shift 2 ;;
        --device) PORT="$2"; shift 2 ;;
        --int-gpio) INT="$2"; shift 2 ;;
        --usb-frames) USB="$2"; shift 2 ;;
        --clean) CLEAN=1; shift ;;
        -h|--help) usage; exit 0 ;;
        *) echo "Unknown option: $1"; usage; exit 1 ;;
    esac
done
[[ -z "$ACTION" ]] && { usage; exit 1; }
if [[ $ACTION == list ]]; then
    exec python3 tools/list_devices.py
fi
if [[ $ACTION == flash && -z "$PORT" ]]; then
    shopt -s nullglob
    ports=(/dev/cu.usbmodem* /dev/ttyACM*)
    if [[ ${#ports[@]} -eq 0 ]]; then
        echo "no board on USB. Hold BOOT (B) and plug the USB in now (a data cable, not charge-only); waiting 30 s..."
        for _ in $(seq 60); do
            ports=(/dev/cu.usbmodem* /dev/ttyACM*)
            [[ ${#ports[@]} -gt 0 ]] && break
            sleep 0.5
        done
    fi
    if [[ ${#ports[@]} -eq 0 ]]; then
        echo "still no board on USB after 30 s: try another cable (data, not charge-only) or another USB port"; exit 1
    fi
    if [[ ${#ports[@]} -gt 1 ]]; then
        echo "found ${#ports[@]} USB serial ports; pick one with --device:"; python3 tools/list_devices.py; exit 1
    fi
    PORT="${ports[0]}"
fi
if ! command -v idf.py >/dev/null; then
    envs=("$HOME"/.espressif/python_env/idf*_env)
    [[ -z "$IDF_PYTHON_ENV_PATH" && -d "${envs[0]}" ]] && export IDF_PYTHON_ENV_PATH="${envs[0]}"
    source "${IDF_PATH:-$HOME/skydev-research/esp/esp-idf}/export.sh" >/dev/null
fi
[[ $CLEAN == 1 ]] && idf.py fullclean
if [[ $ACTION == flash ]]; then
    holders=$(lsof -t "$PORT" 2>/dev/null | sort -u | paste -sd, -)
    if [[ -n "$holders" ]]; then
        echo "$PORT is in use by:"; ps -o pid=,command= -p "$holders" | cut -c1-120
        echo "close it first (Ctrl+C in the viewer), then run this again"; exit 1
    fi
    echo "flashing $PORT (board $BOARD)"
    log=$(mktemp -t xiao_flash)
    set -o pipefail
    if ! idf.py -DLIDAR_BOARD=$BOARD -DLIDAR_INT_GPIO=$INT -DLIDAR_USB_FRAMES=$USB -p "$PORT" build flash 2>&1 | tee "$log"; then
        echo
        echo "FLASH FAILED: $(grep -m1 -E 'A fatal error occurred|A serial exception error occurred|Error:' "$log")"
        echo "- 'stopped responding', or the board keeps restarting: hold BOOT (B), tap RESET (R), release BOOT, run the same command again"
        echo "- 'multiple access' or 'readiness to read': another program read the port during the flash; close it and run again"
        echo "- 'port is busy or doesn't exist': the board left USB; check --list and the cable"
        exit 1
    fi
else
    idf.py -DLIDAR_BOARD=$BOARD -DLIDAR_INT_GPIO=$INT -DLIDAR_USB_FRAMES=$USB build
fi
