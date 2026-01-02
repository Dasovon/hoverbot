# Contributing to HoverBot

Thank you for your interest in contributing to HoverBot! This document provides guidelines and instructions for contributing to the project.

---

## 🎯 How Can I Contribute?

### Reporting Bugs

Before creating a bug report:
- **Check existing issues** to avoid duplicates
- **Verify the bug** on the latest version
- **Collect information**: platform, ROS version, error logs

**Submit a bug report:**
1. Use the bug report template
2. Provide a clear, descriptive title
3. Include reproduction steps
4. Add error messages and logs
5. Specify your platform (Pi4, Pi5, Jetson)

### Suggesting Enhancements

Enhancement suggestions are tracked as GitHub issues:
- Use a clear, descriptive title
- Provide detailed description of the proposed feature
- Explain why this feature would be useful
- Include code examples if applicable

### Pull Requests

We welcome pull requests! Here's the process:

1. **Fork the repository**
2. **Create a feature branch** (`git checkout -b feature/amazing-feature`)
3. **Make your changes** following our coding standards
4. **Test your changes** on your platform
5. **Commit with clear messages** (see commit guidelines below)
6. **Push to your fork** (`git push origin feature/amazing-feature`)
7. **Open a Pull Request** using the PR template

---

## 🏗️ Development Setup

### Prerequisites

- Ubuntu 22.04/24.04 or JetPack (Jetson)
- ROS 2 Humble or Jazzy
- Python 3.10+
- Git

### Clone and Build

```bash
# Clone your fork
git clone https://github.com/YOUR_USERNAME/hoverbot.git
cd hoverbot

# Build ROS 2 workspace
cd ros2_ws
colcon build
source install/setup.bash
```

### Platform-Specific Setup

See platform-specific guides:
- [Raspberry Pi 4](platforms/raspberry-pi4/README.md)
- [Raspberry Pi 5](platforms/raspberry-pi5/README.md)
- [Jetson Nano](platforms/jetson-nano/README.md)

---

## 📝 Coding Standards

### Python Style

- Follow **PEP 8** style guide
- Use **meaningful variable names**
- Add **docstrings** to all functions and classes
- Maximum line length: **100 characters**
- Use **type hints** where appropriate

**Example:**

```python
def calculate_wheel_velocity(linear_vel: float, angular_vel: float,
                             wheelbase: float) -> tuple[float, float]:
    """
    Calculate left and right wheel velocities for differential drive.

    Args:
        linear_vel: Linear velocity in m/s
        angular_vel: Angular velocity in rad/s
        wheelbase: Distance between wheels in meters

    Returns:
        Tuple of (left_wheel_vel, right_wheel_vel) in m/s
    """
    left_vel = linear_vel - (angular_vel * wheelbase / 2)
    right_vel = linear_vel + (angular_vel * wheelbase / 2)
    return left_vel, right_vel
```

### ROS 2 Conventions

- Use **lowercase with underscores** for node names (`hoverbot_driver`)
- Use **CamelCase** for class names (`DifferentialDriveController`)
- Follow **ROS 2 naming conventions** for topics and parameters
- Add proper **logging** (INFO, WARN, ERROR levels)

### Commit Messages

Follow **Conventional Commits** specification:

```
<type>(<scope>): <subject>

<body>

<footer>
```

**Types:**
- `feat:` New feature
- `fix:` Bug fix
- `docs:` Documentation changes
- `style:` Code style changes (formatting, no logic change)
- `refactor:` Code refactoring
- `test:` Adding or updating tests
- `chore:` Maintenance tasks

**Examples:**

```
feat(driver): add support for variable baud rates

Add parameter to configure serial baud rate dynamically.
Previously hardcoded to 115200.

Closes #42
```

```
fix(pi4): correct UART device path in config

Changed /dev/ttyS0 to /dev/ttyAMA0 for Pi4 compatibility.

Fixes #53
```

---

## 🧪 Testing

### Before Submitting PR

- [ ] Code builds without errors (`colcon build`)
- [ ] All existing tests pass
- [ ] New features include tests
- [ ] Documentation updated
- [ ] Tested on target platform

### Running Tests

```bash
cd ros2_ws
colcon test --packages-select hoverbot_driver
colcon test-result --verbose
```

---

## 📚 Documentation

### Code Documentation

- Add **docstrings** to all public functions
- Include **usage examples** for complex features
- Document **parameters** and **return values**
- Explain **edge cases** and **limitations**

### User Documentation

When adding features, update:
- Relevant `README.md` files
- Platform-specific guides in `platforms/{platform}/`
- Main documentation in `docs/`

---

## 🌍 Platform Support

### Adding New Platform Support

To add a new hardware platform:

1. **Create platform directory:**
   ```bash
   mkdir -p platforms/{platform-name}/{config,scripts,docs}
   ```

2. **Add configuration:**
   - Create `config/hoverbot_driver.yaml` with correct serial port
   - Create `scripts/setup_uart.sh` for UART setup
   - Create `docs/SETUP.md` with platform-specific instructions
   - Create `README.md` with platform overview

3. **Update documentation:**
   - Add platform to `docs/PLATFORM_SETUP_GUIDE.md`
   - Update main `README.md` platform table
   - Add serial port info to `docs/SERIAL_PORT_GUIDE.md`

4. **Test thoroughly:**
   - Verify serial communication
   - Test ROS 2 driver
   - Validate SLAM and navigation
   - Document any platform-specific issues

See `platforms/common/README.md` for details.

---

## 🔍 Code Review Process

### What We Look For

- **Functionality**: Does it work as intended?
- **Code Quality**: Is it clean, readable, maintainable?
- **Testing**: Are there adequate tests?
- **Documentation**: Is it well-documented?
- **Platform Compatibility**: Works across platforms?

### Review Timeline

- Initial review within **3-5 days**
- Address feedback and update PR
- Approval requires **1 maintainer review**
- Merge after approval and CI passes

---

## 🎖️ Recognition

Contributors will be:
- Listed in `CONTRIBUTORS.md`
- Credited in release notes
- Mentioned in relevant documentation

---

## ❓ Questions?

- **General questions**: [Discussions](https://github.com/yourusername/hoverbot/discussions)
- **Bug reports**: [Issues](https://github.com/yourusername/hoverbot/issues)
- **Real-time chat**: [Discord](https://discord.gg/hoverbot) *(if applicable)*

---

## 📜 License

By contributing to HoverBot, you agree that your contributions will be licensed under the MIT License.

---

**Thank you for contributing to HoverBot! 🤖**
