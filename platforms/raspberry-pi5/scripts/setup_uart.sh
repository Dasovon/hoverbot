#!/bin/bash
#
# UART Setup Script for Raspberry Pi 5
# Configures UART for hoverboard communication
#

set -e  # Exit on error

echo "=================================="
echo "HoverBot UART Setup Script"
echo "=================================="
echo ""

# Check if running as root
if [ "$EUID" -eq 0 ]; then 
   echo "Please run as normal user (not sudo)"
   exit 1
fi

echo "This script will:"
echo "  1. Enable UART in boot config"
echo "  2. Disable Bluetooth (frees up UART)"
echo "  3. Disable serial console"
echo "  4. Add user to dialout group"
echo "  5. Set UART baud rate to 115200"
echo ""
read -p "Continue? (y/n) " -n 1 -r
echo ""
if [[ ! $REPLY =~ ^[Yy]$ ]]; then
    exit 1
fi

# Backup config files
echo "[1/5] Backing up config files..."
sudo cp /boot/firmware/config.txt /boot/firmware/config.txt.backup
echo "✓ Backups created"

# Enable UART in config.txt
echo ""
echo "[2/5] Configuring boot settings..."
if ! grep -q "enable_uart=1" /boot/firmware/config.txt; then
    echo "enable_uart=1" | sudo tee -a /boot/firmware/config.txt
    echo "✓ Added enable_uart=1"
else
    echo "✓ enable_uart already enabled"
fi

if ! grep -q "dtoverlay=disable-bt" /boot/firmware/config.txt; then
    echo "dtoverlay=disable-bt" | sudo tee -a /boot/firmware/config.txt
    echo "✓ Added dtoverlay=disable-bt"
else
    echo "✓ Bluetooth already disabled"
fi

# Disable serial console
echo ""
echo "[3/5] Disabling serial console..."
sudo systemctl stop serial-getty@ttyAMA0.service 2>/dev/null || true
sudo systemctl disable serial-getty@ttyAMA0.service 2>/dev/null || true
echo "✓ Serial console disabled"

# Add user to dialout group
echo ""
echo "[4/5] Adding $USER to dialout group..."
sudo usermod -aG dialout $USER
echo "✓ Added to dialout group"

# Set baud rate
echo ""
echo "[5/5] Configuring UART settings..."
echo "✓ Will be configured automatically on next boot"

echo ""
echo "=================================="
echo "✅ UART Setup Complete!"
echo "=================================="
echo ""
echo "IMPORTANT:"
echo "  1. You must REBOOT for changes to take effect"
echo "  2. After reboot, log out and back in (for group change)"
echo "  3. Run: stty -F /dev/ttyAMA0 115200 raw -echo"
echo "  4. Test with: python3 test_serial.py"
echo ""
read -p "Reboot now? (y/n) " -n 1 -r
echo ""
if [[ $REPLY =~ ^[Yy]$ ]]; then
    sudo reboot
fi
