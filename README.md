# ROS 2 Workspace - Quick Start

**Complete ROS 2 workspace for HoverBot robot.**

---

## 📁 What's in This Folder

```
ros2_ws/
├── src/
│   ├── hoverbot_description/     # Robot URDF model
│   ├── hoverbot_bringup/          # Launch files
│   ├── hoverbot_driver/           # Serial driver (planned)
│   └── hoverbot_navigation/       # Nav2 configs (planned)
├── docs/
│   ├── ROS2_SETUP.md             # ROS 2 installation
│   ├── PACKAGES.md               # Package documentation
│   └── USAGE.md                  # How to use
└── README.md                     # This file
```

---

## 🚀 Quick Build (5 minutes)

### 1. Install ROS 2 Jazzy

```bash
# Add ROS 2 repository
sudo apt update
sudo apt install software-properties-common -y
sudo add-apt-repository universe

sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | \
  sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2
sudo apt update
sudo apt install ros-jazzy-desktop -y

# Install development tools
sudo apt install python3-colcon-common-extensions python3-rosdep -y

# Initialize rosdep
sudo rosdep init
rosdep update

# Setup environment
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 2. Build Workspace

```bash
cd ros2_ws/

# Install dependencies
rosdep install --from-paths src --ignore-src -r -y

# Build
colcon build --symlink-install

# Source workspace
source install/setup.bash
echo "source ~/hoverbot/ros2_ws/install/setup.bash" >> ~/.bashrc
```

### 3. Test

```bash
# Launch robot state publisher
ros2 launch hoverbot_bringup state_publisher.launch.py

# In another terminal, visualize
rviz2
```

---

## 📦 Packages

### hoverbot_description ✅
**Robot model and visualization**

- URDF with differential drive base
- RPLidar A1 sensor included
- Proper inertial properties
- Ready for simulation

```bash
ros2 launch hoverbot_bringup state_publisher.launch.py
```

### hoverbot_bringup ✅
**Launch file collection**

- State publisher
- Teleoperation
- SLAM mapping (when driver ready)
- Navigation (when driver ready)

### hoverbot_driver 🔄
**Serial communication node** (in progress)

Will provide:
- `/cmd_vel` subscriber
- Odometry publisher
- Transform broadcasting
- Telemetry topics

### hoverbot_navigation 📋
**Navigation configuration** (planned)

Will provide:
- Nav2 parameter files
- Costmap configurations
- Behavior tree setup

---

## 🎮 Usage Examples

### Visualize Robot
```bash
ros2 launch hoverbot_bringup state_publisher.launch.py
rviz2
```

### Keyboard Teleoperation (when driver ready)
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### SLAM Mapping (when driver ready)
```bash
ros2 launch hoverbot_bringup slam.launch.py
```

### Autonomous Navigation (when driver ready)
```bash
ros2 launch hoverbot_bringup navigation.launch.py
```

---

## 🔧 Development

### Create New Package
```bash
cd src/
ros2 pkg create --build-type ament_python my_package
```

### Build Single Package
```bash
colcon build --packages-select hoverbot_description
```

### Clean Build
```bash
rm -rf build/ install/ log/
colcon build --symlink-install
```

---

## 📚 Documentation

- **`docs/ROS2_SETUP.md`** - Detailed ROS 2 installation
- **`docs/PACKAGES.md`** - Package documentation
- **`docs/USAGE.md`** - Usage examples and tutorials

---

## ✅ Status

| Package | Status | Notes |
|---------|--------|-------|
| hoverbot_description | ✅ Complete | URDF ready |
| hoverbot_bringup | ✅ Complete | Launch files ready |
| hoverbot_driver | 🔄 In Progress | Serial bridge needed |
| hoverbot_navigation | 📋 Planned | Nav2 configs |

---

## 🔗 Next Steps

1. **Verify build** - `colcon build`
2. **Test visualization** - Launch state publisher + RViz
3. **Develop driver** - Bridge serial to ROS topics
4. **Integrate sensors** - Add RPLidar driver
5. **Configure SLAM** - slam_toolbox setup
6. **Setup Nav2** - Autonomous navigation

---

**Workspace ready! Build and test visualization before driver development.**
