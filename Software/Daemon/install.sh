#!/bin/bash
#
# AlphaPaint Daemon - Installation Script
# This script installs the AlphaPaint daemon on a Raspberry Pi
#

set -e  # Exit on error

echo "======================================"
echo "AlphaPaint Daemon - Installation"
echo "======================================"
echo ""

# Check if running as root
if [ "$EUID" -eq 0 ]; then
    echo "Error: Do not run this script as root"
    echo "Run as normal user (pi): ./install.sh"
    exit 1
fi

# Get the directory where this script is located
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

# The daemon runs in place from this git checkout - nothing is copied elsewhere.
DAEMON_DIR="$SCRIPT_DIR"

echo "Installing from: $DAEMON_DIR (runs in place)"
echo ""

# Warn about the legacy copy-based install location if present
echo "[1/6] Checking for legacy install..."
if [ -d "/home/$USER/alphapaint" ]; then
    echo "NOTE: legacy copy /home/$USER/alphapaint exists and is no longer used; you can remove it."
fi

# Make daemon executable
echo "[2/6] Making daemon executable..."
chmod +x "$DAEMON_DIR/daemon.py"

# Create virtual environment and install Python dependencies
echo "[3/6] Creating virtual environment and installing dependencies..."
python3 -m venv "$DAEMON_DIR/venv"
"$DAEMON_DIR/venv/bin/pip" install -r "$DAEMON_DIR/requirements.txt"

# Add user to dialout group for serial port access
echo "[4/6] Setting up serial port permissions..."
if ! groups $USER | grep -q dialout; then
    echo "Adding $USER to dialout group..."
    sudo usermod -a -G dialout $USER
    echo "NOTE: You must log out and log back in for group changes to take effect"
fi

# Create log directory
echo "[5/6] Creating log directory..."
sudo mkdir -p /var/log
sudo touch /var/log/alphapaint-daemon.log
sudo chown $USER:$USER /var/log/alphapaint-daemon.log

# Install systemd service
echo "[6/6] Installing systemd service..."

# Update service file with actual user and paths
sudo cp "$DAEMON_DIR/alphapaint-daemon.service" /etc/systemd/system/
sudo sed -i "s|User=pi|User=$USER|g" /etc/systemd/system/alphapaint-daemon.service
sudo sed -i "s|Group=pi|Group=$USER|g" /etc/systemd/system/alphapaint-daemon.service
sudo sed -i "s|/home/pi/AlphaPaint/Software/Daemon|$DAEMON_DIR|g" /etc/systemd/system/alphapaint-daemon.service

# Reload systemd
sudo systemctl daemon-reload

echo ""
echo "======================================"
echo "Installation Complete!"
echo "======================================"
echo ""
echo "Next steps:"
echo ""
echo "1. Review and edit configuration (optional):"
echo "   nano $DAEMON_DIR/config.yaml"
echo ""
echo "2. Enable daemon to start on boot:"
echo "   sudo systemctl enable alphapaint-daemon"
echo ""
echo "3. Start the daemon:"
echo "   sudo systemctl start alphapaint-daemon"
echo ""
echo "4. Check daemon status:"
echo "   sudo systemctl status alphapaint-daemon"
echo ""
echo "5. View logs:"
echo "   sudo journalctl -u alphapaint-daemon -f"
echo ""
echo "NOTE: If you were added to the dialout group,"
echo "      you must log out and log back in!"
echo ""
