#!/bin/bash

# ESP-IDF Flash Script for AlphaPaint Console

echo "========================================="
echo "Flashing AlphaPaint Console Test Program"
echo "========================================="

# Check if IDF_PATH is set
if [ -z "$IDF_PATH" ]; then
    echo "ERROR: IDF_PATH is not set!"
    echo "Please run: . \$HOME/esp/esp-idf/export.sh"
    echo "Or wherever your ESP-IDF installation is located."
    exit 1
fi

# Check if build exists
if [ ! -d "build" ]; then
    echo "ERROR: Build directory not found!"
    echo "Please run ./build.sh first"
    exit 1
fi

# Find the USB serial port
PORT=$(ls /dev/cu.usbserial* 2>/dev/null | head -n 1)

if [ -z "$PORT" ]; then
    echo "ERROR: No USB serial port found!"
    echo "Please connect your ESP32 via USB"
    echo "Looking for /dev/cu.usbserial*"
    exit 1
fi

echo "Found ESP32 on port: $PORT"
echo ""
echo "NOTE: If flashing fails with 'Wrong boot mode detected':"
echo "  1. Hold down the BOOT button on the ESP32"
echo "  2. While holding BOOT, press and release the RESET button"
echo "  3. Release the BOOT button"
echo "  4. Run this script again"
echo ""
echo "Flashing device and starting monitor..."
echo "Press Ctrl+] to exit monitor"
echo ""

idf.py -p $PORT flash 

if [ $? -ne 0 ]; then
    echo ""
    echo "========================================="
    echo "Flash failed!"
    echo "========================================="
    echo "Make sure your ESP32 is in download mode"
    exit 1
fi
