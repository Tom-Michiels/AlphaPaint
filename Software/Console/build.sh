#!/bin/bash

# ESP-IDF Build Script for AlphaPaint Console

echo "========================================="
echo "Building AlphaPaint Console Test Program"
echo "========================================="

# Check if IDF_PATH is set
if [ -z "$IDF_PATH" ]; then
    echo "ERROR: IDF_PATH is not set!"
    echo "Please run: . \$HOME/esp/esp-idf/export.sh"
    echo "Or wherever your ESP-IDF installation is located."
    exit 1
fi

# Set target to ESP32 only if not already configured
if [ ! -f "build/CMakeCache.txt" ]; then
    echo "Setting target to ESP32..."
    idf.py set-target esp32
else
    echo "Target already configured, skipping set-target..."
fi

# Build the project
echo "Building project..."
idf.py build

if [ $? -eq 0 ]; then
    echo ""
    echo "========================================="
    echo "Build successful!"
    echo "========================================="
    echo "To flash the device, run: ./flash.sh"
else
    echo ""
    echo "========================================="
    echo "Build failed!"
    echo "========================================="
    exit 1
fi
