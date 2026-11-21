# flight-range-finder

ESP-IDF app for a Seeed XIAO ESP32-S3 that reads a VL53L1X laser range finder over I2C and writes each reading as an 8-byte binary frame to a UART the moment it is read. Build with `./build.sh` (ESP-IDF sourced); decode with `tools/view_lidar.py`; the spec is `SPEC.md`.
