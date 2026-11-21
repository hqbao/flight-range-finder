#!/bin/bash
# usage: ./build.sh [--int-gpio N] [--usb-frames 0|1] [--clean]
set -e
INT=-1; USB=0
while [[ $# -gt 0 ]]; do
    case "$1" in
        --int-gpio) INT="$2"; shift 2 ;;
        --usb-frames) USB="$2"; shift 2 ;;
        --clean) idf.py fullclean; shift ;;
        *) echo "Unknown option: $1"; exit 1 ;;
    esac
done
command -v idf.py >/dev/null || { echo "source ~/skydev-research/esp/esp-idf/export.sh first"; exit 1; }
cd "$(dirname "$0")"
idf.py -DLIDAR_INT_GPIO=$INT -DLIDAR_USB_FRAMES=$USB build
