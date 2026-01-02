# HoverBot Platform Setup Guide

**Choose your platform and get started with HoverBot!**

This guide helps you select the right hardware platform and directs you to platform-specific setup instructions.

---

## Supported Platforms

HoverBot supports multiple hardware platforms. Choose the one that best fits your needs:

| Platform | Status | Best For | Difficulty |
|----------|--------|----------|------------|
| [**Raspberry Pi 5**](#raspberry-pi-5) | ✅ **Tested & Recommended** | Best performance, latest software | ⭐ Easy |
| [**Raspberry Pi 4**](#raspberry-pi-4) | ✅ **Fully Supported** | Budget-friendly, proven stability | ⭐ Easy |
| [**Jetson Nano**](#jetson-nano) | ⚠️ **Experimental** | AI/Vision applications | ⭐⭐ Moderate |

---

## Platform Comparison

### Quick Comparison Table

| Feature | Raspberry Pi 4 | Raspberry Pi 5 | Jetson Nano |
|---------|---------------|----------------|-------------|
| **CPU** | Cortex-A72 @ 1.5 GHz | Cortex-A76 @ 2.4 GHz | Cortex-A57 @ 1.43 GHz |
| **RAM** | 4GB/8GB | 4GB/8GB | 4GB |
| **GPU** | None (VideoCore) | VideoCore VII | 128-core Maxwell |
| **Price** | ~$55 | ~$80 | ~$99 |
| **Power** | 5-7W | 8-10W | 5-10W |
| **Ubuntu Version** | 22.04 LTS | 24.04 LTS | 20.04 (JetPack) |
| **ROS 2 Version** | Humble | Jazzy | Humble (source build) |
| **Serial Port** | `/dev/ttyAMA0` | `/dev/ttyAMA0` | `/dev/ttyTHS1` |
| **AI Performance** | ❌ Poor | ⚠️ Moderate | ✅ Excellent |
| **Setup Complexity** | ⭐ Easy | ⭐ Easy | ⭐⭐ Moderate |
| **Community Support** | ✅ Excellent | ✅ Excellent | ⚠️ Good |

---

## Raspberry Pi 5

### Why Choose Pi 5?

✅ **Best overall choice for most users**

**Pros:**
- Best performance (60% faster than Pi 4)
- Latest Ubuntu 24.04 LTS + ROS 2 Jazzy
- Excellent community support
- Lower CPU usage (~40% vs 60% on Pi 4)
- Future-proof (support until 2029)
- Native hardware support

**Cons:**
- Slightly more expensive (~$80)
- Higher power consumption (8-10W)

**Hardware Requirements:**
- Raspberry Pi 5 (8GB recommended, 4GB minimum)
- 32GB+ microSD card
- USB-C power supply (5V 5A)
- Hoverboard with STM32F103 controller
- RPLidar A1 (optional but recommended)

**Get Started:**
👉 **[Raspberry Pi 5 Setup Guide](../platforms/raspberry-pi5/docs/SETUP.md)**

**Quick Links:**
- [Pi5 Configuration](../platforms/raspberry-pi5/config/hoverbot_driver.yaml)
- [Pi5 Scripts](../platforms/raspberry-pi5/scripts/)
- [Pi5 README](../platforms/raspberry-pi5/README.md)

---

## Raspberry Pi 4

### Why Choose Pi 4?

✅ **Best budget option with proven stability**

**Pros:**
- Lower cost (~$55)
- Lower power consumption (5-7W)
- Proven stability (millions deployed)
- Ubuntu 22.04 LTS (support until 2027)
- ROS 2 Humble (LTS until 2027)
- Sufficient performance for navigation/SLAM

**Cons:**
- Slightly slower than Pi 5
- Higher CPU usage (~60%)
- Older software stack

**Hardware Requirements:**
- Raspberry Pi 4 (4GB recommended, 2GB minimum)
- 32GB+ microSD card
- USB-C power supply (5V 3A)
- Hoverboard with STM32F103 controller
- RPLidar A1 (optional but recommended)

**Get Started:**
👉 **[Raspberry Pi 4 Setup Guide](../platforms/raspberry-pi4/docs/SETUP.md)** *(Coming soon - adapt Pi5 guide)*

**Quick Links:**
- [Pi4 Configuration](../platforms/raspberry-pi4/config/hoverbot_driver.yaml)
- [Pi4 Scripts](../platforms/raspberry-pi4/scripts/)
- [Pi4 README](../platforms/raspberry-pi4/README.md)

---

## Jetson Nano

### Why Choose Jetson Nano?

⚠️ **For advanced users planning AI/vision features**

**Pros:**
- NVIDIA GPU (128 CUDA cores)
- Excellent for computer vision / AI
- Can run object detection (YOLO)
- Can run neural networks in real-time
- Good performance per watt
- Future expansion potential

**Cons:**
- More complex setup (ROS 2 from source)
- Different serial port (`/dev/ttyTHS1`)
- Less community support for robotics
- Older OS (Ubuntu 20.04 via JetPack)
- Not yet tested with HoverBot

**Hardware Requirements:**
- Jetson Nano 4GB
- 32GB+ microSD card or NVMe SSD
- Barrel jack power supply (5V 4A)
- Hoverboard with STM32F103 controller
- Camera (if using vision features)
- RPLidar A1 (optional)

**Get Started:**
👉 **[Jetson Nano Setup Guide](../platforms/jetson-nano/docs/SETUP.md)** *(Coming soon)*

**Quick Links:**
- [Jetson Configuration](../platforms/jetson-nano/config/hoverbot_driver.yaml)
- [Jetson Scripts](../platforms/jetson-nano/scripts/)
- [Jetson README](../platforms/jetson-nano/README.md)

---

## Decision Guide

### Choose Your Platform

**Answer these questions:**

1. **Is this your first robotics project?**
   - **Yes** → Choose **Raspberry Pi 5**
   - **No** → Continue to next question

2. **Do you need AI/computer vision?**
   - **Yes** → Choose **Jetson Nano**
   - **No** → Continue to next question

3. **What's your budget?**
   - **Tight budget** → Choose **Raspberry Pi 4**
   - **Can spend more** → Choose **Raspberry Pi 5**

4. **Do you already own hardware?**
   - **I have a Pi 4** → Use it! Pi 4 works great
   - **I have a Pi 5** → Best choice!
   - **I have a Jetson** → Advanced but supported
   - **I need to buy** → Get **Raspberry Pi 5**

---

## Platform-Specific Features

### Serial Port Configuration

| Platform | Device Path | Notes |
|----------|-------------|-------|
| Pi 4 | `/dev/ttyAMA0` | PL011 UART on GPIO 14/15 |
| Pi 5 | `/dev/ttyAMA0` | PL011 UART on GPIO 14/15 (same as Pi4) |
| Jetson Nano | `/dev/ttyTHS1` | Tegra High-Speed UART1 |

**See also:** [Serial Port Configuration Guide](SERIAL_PORT_GUIDE.md)

### Software Stack

| Platform | Ubuntu | ROS 2 | Installation |
|----------|--------|-------|--------------|
| Pi 4 | 22.04 LTS | Humble | apt install |
| Pi 5 | 24.04 LTS | Jazzy | apt install |
| Jetson | 20.04 | Humble | Source build |

---

## Common Setup Steps

All platforms share these core steps:

1. ✅ Flash OS to SD card
2. ✅ Boot and configure network
3. ✅ Run UART setup script
4. ✅ Install ROS 2
5. ✅ Clone HoverBot repository
6. ✅ Build ROS 2 workspace
7. ✅ Flash hoverboard firmware
8. ✅ Configure with platform-specific config file
9. ✅ Test and launch

**Differences:**
- Step 1: OS version varies by platform
- Step 3: UART device path varies
- Step 4: ROS 2 version/method varies
- Step 8: Config file location varies

---

## Migration Between Platforms

### Switching from Pi 4 to Pi 5

✅ **Easy migration:**

1. Flash Ubuntu 24.04 to new SD card
2. Install ROS 2 Jazzy
3. Copy your workspace: `rsync -av ros2_ws/ new_pi:/home/user/hoverbot/ros2_ws/`
4. Use Pi5 config: `platforms/raspberry-pi5/config/hoverbot_driver.yaml`
5. Done! Serial port is the same.

### Switching from Pi to Jetson

⚠️ **Moderate complexity:**

1. Flash JetPack to Jetson
2. Build ROS 2 Humble from source
3. Copy your workspace
4. Update config to use `/dev/ttyTHS1`
5. Test serial communication

---

## Performance Benchmarks

### Validated on Pi 5

| Metric | Target | Actual |
|--------|--------|--------|
| Serial Success | >95% | 99.3% |
| Odometry Rate | 50 Hz | 50 Hz |
| SLAM Update | 0.1 Hz | 0.1 Hz |
| CPU Usage | <50% | ~40% |

### Expected on Pi 4

| Metric | Expected |
|--------|----------|
| Serial Success | >95% |
| Odometry Rate | 50 Hz |
| SLAM Update | 0.1 Hz |
| CPU Usage | ~60% |

### Expected on Jetson

| Metric | Expected |
|--------|----------|
| Serial Success | >95% |
| Odometry Rate | 50 Hz |
| SLAM Update | 0.1 Hz |
| CPU Usage | ~50% |

---

## Getting Help

### Platform-Specific Support

- **Pi 4/Pi 5:** Excellent community support, well-documented
- **Jetson Nano:** NVIDIA forums, robotics community

### Resources

- 📚 [Common Platform Utilities](../platforms/common/README.md)
- 📚 [Serial Port Guide](SERIAL_PORT_GUIDE.md)
- 📚 [Firmware Guide](FIRMWARE.md)
- 📚 [Hardware Wiring](../hardware/README.md)

### Community

- **Issues:** [GitHub Issues](https://github.com/yourusername/hoverbot/issues)
- **Discussions:** [GitHub Discussions](https://github.com/yourusername/hoverbot/discussions)
- **ROS Answers:** [answers.ros.org](https://answers.ros.org)

---

## Quick Start Checklist

- [ ] Choose your platform
- [ ] Purchase required hardware
- [ ] Download platform-specific OS image
- [ ] Flash to SD card
- [ ] Follow platform-specific setup guide
- [ ] Build and test!

---

**Ready to get started?** Pick your platform above and follow the setup guide!
