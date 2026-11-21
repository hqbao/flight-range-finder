#!/bin/bash
# usage: ./build.sh [--flash [PORT]] [--usb-frames 0|1] [--int-gpio N] [--clean]
set -e
cd "$(dirname "$0")"
INT=-1; USB=1; FLASH=0; PORT=""; CLEAN=0
while [[ $# -gt 0 ]]; do
    case "$1" in
        --int-gpio) INT="$2"; shift 2 ;;
        --usb-frames) USB="$2"; shift 2 ;;
        --flash) FLASH=1; shift; if [[ $# -gt 0 && "$1" != --* ]]; then PORT="$1"; shift; fi ;;
        --clean) CLEAN=1; shift ;;
        *) echo "Unknown option: $1"; exit 1 ;;
    esac
done
if ! command -v idf.py >/dev/null; then
    envs=("$HOME"/.espressif/python_env/idf*_env)
    [[ -z "$IDF_PYTHON_ENV_PATH" && -d "${envs[0]}" ]] && export IDF_PYTHON_ENV_PATH="${envs[0]}"
    source "${IDF_PATH:-$HOME/skydev-research/esp/esp-idf}/export.sh" >/dev/null
fi
[[ $CLEAN == 1 ]] && idf.py fullclean
if [[ $FLASH == 1 && -z "$PORT" ]]; then
    shopt -s nullglob
    ports=(/dev/cu.usbmodem* /dev/ttyACM*)
    if [[ ${#ports[@]} -eq 0 ]]; then
        echo "no XIAO on USB. Hold BOOT (B) and plug the USB in now (a data cable, not charge-only); waiting 30 s..."
        for _ in $(seq 60); do
            ports=(/dev/cu.usbmodem* /dev/ttyACM*)
            [[ ${#ports[@]} -gt 0 ]] && break
            sleep 0.5
        done
    fi
    if [[ ${#ports[@]} -eq 0 ]]; then
        echo "still no XIAO on USB after 30 s: try another cable (data, not charge-only) or another USB port"; exit 1
    fi
    if [[ ${#ports[@]} -gt 1 ]]; then
        echo "found ${#ports[@]} USB serial ports (${ports[*]}): plug in only the XIAO, or name it: ./build.sh --flash PORT"; exit 1
    fi
    PORT="${ports[0]}"
    echo "flashing the XIAO on $PORT"
fi
if [[ $FLASH == 1 ]]; then
    idf.py -DLIDAR_INT_GPIO=$INT -DLIDAR_USB_FRAMES=$USB -p "$PORT" build flash
else
    idf.py -DLIDAR_INT_GPIO=$INT -DLIDAR_USB_FRAMES=$USB build
fi
