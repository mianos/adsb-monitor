#!/usr/bin/env bash
# Build wrapper: ./build.sh [flash|monitor]. Sets the esp32s3 target on first run.
set -e
set -o pipefail
export IDF_PATH=/Users/robertfowler/.espressif/v6.0.1/esp-idf
. "$IDF_PATH/export.sh" >/dev/null 2>&1
cd "$(dirname "$0")"
PORT=${PORT:-/dev/cu.usbmodem142101}
if [ ! -f sdkconfig ] || ! grep -q 'CONFIG_IDF_TARGET="esp32s3"' sdkconfig 2>/dev/null; then
    idf.py set-target esp32s3
fi
idf.py build 2>&1 | tee /tmp/adsb_build.log | tail -n 40
case "$1" in
    flash) idf.py -p "$PORT" flash ;;
    monitor) idf.py -p "$PORT" flash monitor ;;
esac
