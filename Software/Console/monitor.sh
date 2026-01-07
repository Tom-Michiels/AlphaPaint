#!/bin/bash

# ESP32 Serial Monitor Script

echo "========================================="
echo "ESP32 Console Monitor"
echo "========================================="

# Find the USB serial port
PORT=$(ls /dev/cu.usbserial* 2>/dev/null | head -n 1)

if [ -z "$PORT" ]; then
    echo "ERROR: No USB serial port found!"
    echo "Please connect your ESP32 via USB"
    echo "Looking for /dev/cu.usbserial*"
    exit 1
fi

echo "Connecting to $PORT at 115200 baud"
echo "Press Ctrl+A then K to exit, or Ctrl+C"
echo ""

# Use screen to connect to serial port
screen $PORT 115200
