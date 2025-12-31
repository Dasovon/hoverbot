# udev Rules for HoverBot

Persistent device names that work on any USB port.

## Installation on Raspberry Pi

```bash
sudo cp 99-hoverbot.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
```

**Unplug and replug devices to activate.**

## Persistent Device Names

- **`/dev/rplidar`** → RPLidar A1 (any USB port)
- **`/dev/elp_camera`** → ELP USB camera (any USB port)
- RealSense D435 auto-detected by driver (USB ID 8086:0b07)

## Verification

```bash
ls -la /dev/rplidar      # Should show symlink
ls -la /dev/elp_camera   # Should show symlink
```

## Troubleshooting

**Device not appearing:**
```bash
# Check USB IDs
lsusb

# Test rule
udevadm test /sys/class/tty/ttyUSB0
```

**Permissions incorrect:**
```bash
# Ensure MODE="0666" in rule
ls -la /dev/rplidar  # Should show: crw-rw-rw-
```

---

**Last Updated:** December 31, 2026
