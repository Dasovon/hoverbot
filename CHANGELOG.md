# Changelog

All notable changes to HoverBot will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

---

## [Unreleased]

### Added
- Multi-platform support (Raspberry Pi 4, Pi 5, Jetson Nano)
- Complete Gazebo simulation with all sensors
- Platform-specific configuration files
- Comprehensive platform documentation
- GitHub community files (CONTRIBUTING.md, SECURITY.md, issue templates)

### Changed
- Restructured repository with `platforms/` directory
- Updated README with platform selector
- Improved .gitignore for development files

### Removed
- Deprecated `raspberry-pi/` directory (migrated to `platforms/raspberry-pi5/`)
- Old `docs/SETUP.md` (moved to platform-specific locations)

---

## [0.2.0] - 2026-01-01

### Added
- **Multi-Platform Support**
  - Raspberry Pi 4 support (Ubuntu 22.04 + ROS 2 Humble)
  - Raspberry Pi 5 support (Ubuntu 24.04 + ROS 2 Jazzy)
  - Jetson Nano support (JetPack + ROS 2 Humble) - Experimental
  - Platform-specific UART setup scripts
  - Platform-specific configuration files

- **Gazebo Simulation**
  - Complete robot URDF model
  - Gazebo plugins for all sensors (LiDAR, IMU, cameras)
  - Differential drive simulation
  - Launch files for simulation

- **Documentation**
  - Platform setup guide with comparison
  - Serial port configuration guide
  - Platform-specific README files
  - Comprehensive troubleshooting guides

- **Repository Structure**
  - `platforms/` directory for platform-specific files
  - `platforms/common/` for shared utilities
  - Session tracking with `.scratchpad/`

### Changed
- Migrated Raspberry Pi 5 files to `platforms/raspberry-pi5/`
- Updated main README with platform selection
- Improved repository organization

### Fixed
- Serial port configuration now properly parameterized
- UART setup scripts validated for each platform

---

## [0.1.0] - 2025-12-XX

### Added
- **Initial HoverBot Implementation**
  - ROS 2 driver for hoverboard serial communication
  - Differential drive kinematics
  - Odometry publishing
  - Transform broadcasting (odom → base_link)
  - Diagnostics publishing (battery, temperature, errors)

- **Hardware Support**
  - STM32F103 hoverboard firmware configuration
  - RPLidar A1 integration
  - BNO055 IMU integration
  - Intel RealSense D435 support
  - ELP USB camera support

- **ROS 2 Packages**
  - `hoverbot_driver` - Serial communication and motor control
  - `hoverbot_description` - Robot URDF model
  - `hoverbot_bringup` - Launch files and configurations

- **Documentation**
  - Firmware configuration guide
  - Hardware wiring documentation
  - Initial setup instructions
  - Development journal

### Technical Details
- Serial protocol: 8-byte packets at 115200 baud
- Update rate: 50 Hz odometry
- Serial success rate: 99.3%
- Supported: Ubuntu 24.04 + ROS 2 Jazzy (Raspberry Pi 5)

---

## Release Notes

### Version 0.2.0 - Multi-Platform Release

This major update restructures the repository to support multiple hardware platforms while maintaining a single, platform-independent ROS 2 codebase.

**Key Highlights:**
- 🎯 Support for 3 platforms (Pi4, Pi5, Jetson Nano)
- 🤖 Complete Gazebo simulation
- 📚 1,750+ lines of new documentation
- 🏗️ Professional GitHub repository structure

**Migration Guide:**
- Raspberry Pi 5 users: No changes needed, files moved to `platforms/raspberry-pi5/`
- Use platform-specific config files for launching

**Known Issues:**
- Jetson Nano support is experimental (not tested on hardware)
- Pi4 complete setup guide pending

---

## Future Roadmap

### Version 0.3.0 (Planned)
- [ ] Complete Pi4 setup documentation
- [ ] Jetson Nano hardware validation
- [ ] Navigation stack integration
- [ ] Autonomous mapping demos
- [ ] Performance benchmarks for all platforms

### Version 0.4.0 (Planned)
- [ ] Computer vision integration (Jetson)
- [ ] Object detection and tracking
- [ ] Web interface for monitoring
- [ ] Cloud connectivity

### Long-term Goals
- Multi-robot coordination
- Advanced path planning algorithms
- Machine learning integration
- Commercial deployment support

---

## Contributing

See [CONTRIBUTING.md](CONTRIBUTING.md) for contribution guidelines.

## Security

See [SECURITY.md](SECURITY.md) for security policy and vulnerability reporting.

---

[Unreleased]: https://github.com/yourusername/hoverbot/compare/v0.2.0...HEAD
[0.2.0]: https://github.com/yourusername/hoverbot/compare/v0.1.0...v0.2.0
[0.1.0]: https://github.com/yourusername/hoverbot/releases/tag/v0.1.0
