#!/bin/bash
#
# UART Setup Script for Jetson Nano
# Configures UART1 (THS1) for hoverboard communication
#

set -e  # Exit on error

echo "=================================="
echo "HoverBot UART Setup Script - Jetson Nano"
echo "=================================="
echo ""

# Check if running as root
if [ "$EUID" -eq 0 ]; then
   echo "Please run as normal user (not sudo)"
   exit 1
fi

echo "This script will:"
echo "  1. Disable serial console on ttyTHS1"
echo "  2. Add user to dialout group"
echo "  3. Configure UART1 permissions"
echo "  4. Set UART baud rate to 115200"
echo ""
echo "NOTE: Jetson Nano uses /dev/ttyTHS1 for UART1 (pins 8/10 on J41 header)"
echo ""
read -p "Continue? (y/n) " -n 1 -r
echo ""
if [[ ! $REPLY =~ ^[Yy]$ ]]; then
    exit 1
fi

# Disable serial console on ttyTHS1
echo ""
echo "[1/4] Disabling serial console on ttyTHS1..."
sudo systemctl stop nvgetty 2>/dev/null || true
sudo systemctl disable nvgetty 2>/dev/null || true
echo "✓ Serial console disabled"

# Add user to dialout group
echo ""
echo "[2/4] Adding $USER to dialout group..."
sudo usermod -aG dialout $USER
echo "✓ Added to dialout group"

# Create udev rule for UART permissions
echo ""
echo "[3/4] Configuring UART permissions..."
echo 'KERNEL=="ttyTHS1", MODE="0666"' | sudo tee /etc/udev/rules.d/99-hoverbot-uart.rules
sudo udevadm control --reload-rules
sudo udevadm trigger
echo "✓ UART permissions configured"

# Set baud rate
echo ""
echo "[4/4] Configuring UART settings..."
if [ -e /dev/ttyTHS1 ]; then
    sudo stty -F /dev/ttyTHS1 115200 raw -echo
    echo "✓ UART configured to 115200 baud"
else
    echo "⚠ /dev/ttyTHS1 not found - will be configured on next boot"
fi

echo ""
echo "=================================="
echo "✅ UART Setup Complete!"
echo "=================================="
echo ""
echo "IMPORTANT:"
echo "  1. You must LOG OUT and back in (for group change)"
echo "  2. Test serial port:"
echo "     ls -l /dev/ttyTHS1"
echo "  3. Verify baud rate:"
echo "     stty -F /dev/ttyTHS1"
echo "  4. Test with: python3 test_serial.py --port /dev/ttyTHS1"
echo ""
echo "JETSON UART PINOUT (J41 40-pin header):"
echo "  Pin  8 (GPIO14 / UART1_TX) → Hoverboard RX"
echo "  Pin 10 (GPIO15 / UART1_RX) → Hoverboard TX"
echo "  Pin  6 (GND)               → Hoverboard GND"
echo ""
read -p "Log out now? (y/n) " -n 1 -r
echo ""
if [[ $REPLY =~ ^[Yy]$ ]]; then
    gnome-session-quit --logout --no-prompt 2>/dev/null || pkill -u $USER
fi
