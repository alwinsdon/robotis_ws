# 🤖 Complete ROBOTIS Workspace Setup Guide for ROS 2 Jazzy

This guide provides step-by-step instructions to replicate a fully functional `robotis_ws` workspace with Open Manipulator packages on a fresh Ubuntu 24.04 system with ROS 2 Jazzy.

## 📋 Table of Contents

1. [System Requirements](#system-requirements)
2. [Prerequisites Installation](#prerequisites-installation)
3. [ROS 2 Jazzy Installation](#ros-2-jazzy-installation)
4. [Workspace Setup](#workspace-setup)
5. [Package Installation](#package-installation)
6. [Configuration](#configuration)
7. [Hardware Setup](#hardware-setup)
8. [Testing & Verification](#testing--verification)
9. [Troubleshooting](#troubleshooting)

## 🖥️ System Requirements

- **Operating System**: Ubuntu 24.04 LTS (Noble Numbat)
- **ROS 2 Version**: Jazzy Jalisco
- **Hardware**: 
  - Open Manipulator-X or Open Manipulator-Y
  - USB2DYNAMIXEL adapter
  - 5 Dynamixel XM430-W350 motors
- **Recommended**: 8GB+ RAM, 20GB+ free disk space

## 🔧 Prerequisites Installation

### 1. Update System

```bash
sudo apt update && sudo apt upgrade -y
```

### 2. Install Essential Tools

```bash
sudo apt install -y \
    curl \
    gnupg2 \
    lsb-release \
    software-properties-common \
    apt-transport-https \
    ca-certificates \
    git \
    python3-pip \
    python3-setuptools \
    python3-dev \
    build-essential \
    cmake \
    pkg-config \
    libudev-dev \
    libusb-1.0-0-dev
```

### 3. Install Python Dependencies

```bash
pip3 install --user \
    setuptools \
    wheel \
    vcstool \
    colcon-common-extensions
```

## 🤖 ROS 2 Jazzy Installation

### 1. Add ROS 2 Repository

```bash
# Add the ROS 2 GPG key
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add the repository
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

### 2. Install ROS 2 Jazzy

```bash
sudo apt update
sudo apt install -y ros-jazzy-desktop-full
```

### 3. Install Additional ROS 2 Packages

```bash
sudo apt install -y \
    ros-jazzy-hardware-interface \
    ros-jazzy-controller-manager \
    ros-jazzy-ros2-controllers \
    ros-jazzy-ros2-control \
    ros-jazzy-gz-ros2-control \
    ros-jazzy-tf-transformations \
    ros-jazzy-xacro \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-joint-state-publisher \
    ros-jazzy-joint-state-publisher-gui \
    ros-jazzy-rviz2 \
    ros-jazzy-rqt* \
    ros-jazzy-gazebo-* \
    ros-jazzy-gz-* \
    ros-jazzy-ros-gz-bridge \
    ros-jazzy-ros-gz-sim \
    ros-jazzy-ros-gz-image \
    ros-jazzy-pal-statistics \
    libboost-all-dev
```

### 4. Install MoveIt 2

```bash
sudo apt install -y ros-jazzy-moveit-* --no-install-recommends
```

### 5. Source ROS 2 Environment

```bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## 🏗️ Workspace Setup

### 1. Create Workspace Directory

```bash
mkdir -p ~/robotis_ws/src
cd ~/robotis_ws
```

### 2. Set Environment Variables

Add these lines to your `~/.bashrc`:

```bash
echo 'export ROS_DOMAIN_ID=30' >> ~/.bashrc
echo 'export ROBOT_MODEL=om_x' >> ~/.bashrc  # Change to om_y if using Open Manipulator-Y
echo 'export WORKSPACE_DIR=~/robotis_ws' >> ~/.bashrc
echo 'source ~/robotis_ws/install/setup.bash' >> ~/.bashrc
source ~/.bashrc
```

## 📦 Package Installation

### 1. Clone Required Repositories

```bash
cd ~/robotis_ws/src

# Clone DynamixelSDK
git clone -b jazzy https://github.com/ROBOTIS-GIT/DynamixelSDK.git

# Clone Dynamixel Hardware Interface
git clone -b jazzy https://github.com/ROBOTIS-GIT/dynamixel_interfaces.git
git clone -b jazzy https://github.com/ROBOTIS-GIT/dynamixel_hardware_interface.git

# Clone Open Manipulator packages
git clone -b jazzy https://github.com/ROBOTIS-GIT/open_manipulator.git
```

### 2. Install Dependencies Using rosdep

```bash
cd ~/robotis_ws
sudo rosdep init  # Only if this is your first ROS 2 installation
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

### 3. Build the Workspace

```bash
cd ~/robotis_ws
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

### 4. Source the Workspace

```bash
source ~/robotis_ws/install/setup.bash
```

## ⚙️ Configuration

### 1. Set Up USB Permissions

Add your user to the dialout group for USB communication:

```bash
sudo usermod -aG dialout $USER
```

**Important**: Log out and log back in for this change to take effect.

### 2. Create udev Rules

```bash
# Create udev rules for the USB2DYNAMIXEL adapter
sudo bash -c 'cat > /etc/udev/rules.d/99-open-manipulator-cdc.rules << EOF
SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6014", MODE="0666", GROUP="dialout", SYMLINK+="ttyUSB_OpenManipulator"
SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6001", MODE="0666", GROUP="dialout"
EOF'

# Reload udev rules
sudo udevadm control --reload-rules
sudo udevadm trigger
```

### 3. Verify USB Device Detection

```bash
# Check if your USB2DYNAMIXEL adapter is detected
lsusb | grep -i "ftdi\|future"

# Check available serial ports
ls -la /dev/ttyUSB*
```

You should see output similar to:
```
Bus 001 Device 005: ID 0403:6014 Future Technology Devices International, Ltd FT232H Single HS USB-UART/FIFO IC
crw-rw---- 1 root dialout 188, 0 Dec  6 10:30 /dev/ttyUSB0
```

## 🔌 Hardware Setup

### 1. Physical Connections

1. **Power Supply**: Connect 12V power supply to the Open Manipulator
2. **USB Connection**: Connect USB2DYNAMIXEL adapter to your PC
3. **Serial Connection**: Connect the USB2DYNAMIXEL to the robot's DYNAMIXEL chain
4. **Motor IDs**: Ensure motors are configured with IDs 11-15 (default for Open Manipulator-X)

### 2. Test Hardware Communication

```bash
# Check if the hardware is communicating properly
ros2 launch open_manipulator_bringup hardware_x.launch.py port_name:=/dev/ttyUSB0
```

## 🧪 Testing & Verification

### 1. Test Simulation (Gazebo)

```bash
# Launch Gazebo simulation
ros2 launch open_manipulator_bringup gazebo.launch.py
```

### 2. Test Visualization (RViz)

```bash
# Launch RViz with robot model
ros2 launch open_manipulator_description model_x.launch.py
```

### 3. Test MoveIt (Simulation)

```bash
# Launch MoveIt with simulation
ros2 launch open_manipulator_moveit_config moveit_core.launch.py
```

### 4. Test Hardware (if connected)

```bash
# Launch hardware interface
ros2 launch open_manipulator_bringup hardware_x.launch.py port_name:=/dev/ttyUSB0
```

### 5. Test Keyboard Control

In a new terminal:

```bash
# For keyboard teleoperation
ros2 run open_manipulator_teleop keyboard_control_x.py
```

### 6. Verify ROS 2 Topics

```bash
# List all active topics
ros2 topic list

# Check joint states
ros2 topic echo /joint_states

# Check robot description
ros2 topic echo /robot_description
```

## 🚀 Launch Commands Reference

### Basic Launch Commands

```bash
# Hardware launch (Open Manipulator-X)
ros2 launch open_manipulator_bringup hardware_x.launch.py port_name:=/dev/ttyUSB0

# Hardware launch (Open Manipulator-Y)
ros2 launch open_manipulator_bringup hardware_y.launch.py port_name:=/dev/ttyUSB0

# Gazebo simulation
ros2 launch open_manipulator_bringup gazebo.launch.py

# MoveIt planning
ros2 launch open_manipulator_moveit_config moveit_core.launch.py

# Robot visualization only
ros2 launch open_manipulator_description model_x.launch.py
```

### Teleoperation Commands

```bash
# Keyboard control (Open Manipulator-X)
ros2 run open_manipulator_teleop keyboard_control_x.py

# Keyboard control (Open Manipulator-Y)
ros2 run open_manipulator_teleop keyboard_control_y.py
```

## 🛠️ Troubleshooting

### Common Issues and Solutions

#### 1. Permission Denied Errors

```bash
# Fix USB permissions
sudo chmod 666 /dev/ttyUSB0
# Or add user to dialout group (permanent solution)
sudo usermod -aG dialout $USER
# Then log out and back in
```

#### 2. Package Not Found Errors

```bash
# Rebuild workspace
cd ~/robotis_ws
colcon build --symlink-install
source install/setup.bash
```

#### 3. Hardware Communication Errors

```bash
# Check hardware connection
dmesg | tail -20

# Check motor power and connections
# Verify baudrate settings (default: 1000000)
# Check motor IDs (should be 11-15 for Open Manipulator-X)
```

#### 4. Gazebo Not Starting

```bash
# Install missing Gazebo components
sudo apt install -y ros-jazzy-gazebo-*
sudo apt install -y ros-jazzy-gz-*
```

#### 5. MoveIt Planning Errors

```bash
# Install missing MoveIt components
sudo apt install -y ros-jazzy-moveit-* --no-install-recommends
```

#### 6. Process Cleanup (if needed)

```bash
# Kill all ROS 2 processes
pkill -f "ros2|controller_manager|robot_state_publisher|gazebo|rviz"

# Check for zombie processes
ps aux | grep -E "(ros2|controller|gazebo|rviz)" | grep -v grep
```

### Diagnostic Commands

```bash
# Check ROS 2 installation
printenv | grep -i ROS

# Check workspace
echo $WORKSPACE_DIR
echo $ROBOT_MODEL

# Check available packages
ros2 pkg list | grep -i manipulator

# Check USB devices
lsusb
ls -la /dev/ttyUSB*

# Check system resources
htop
df -h
```

## 📝 Environment Configuration Summary

Your `~/.bashrc` should contain:

```bash
# ROS 2 Jazzy
source /opt/ros/jazzy/setup.bash

# Workspace
export WORKSPACE_DIR=~/robotis_ws
source ~/robotis_ws/install/setup.bash

# Robot Configuration
export ROBOT_MODEL=om_x  # or om_y
export ROS_DOMAIN_ID=30

# Optional: Add workspace to PATH
export PATH=$PATH:~/robotis_ws/install/bin
```

## ✅ Final Verification Checklist

- [ ] ROS 2 Jazzy installed and sourced
- [ ] All required packages installed via apt
- [ ] Workspace created and built successfully
- [ ] USB permissions configured
- [ ] Hardware detected (if connected)
- [ ] Simulation launches successfully
- [ ] MoveIt launches successfully
- [ ] Environment variables set correctly
- [ ] User added to dialout group

## 🔄 Quick Setup Script

For convenience, here's a one-click setup script. Save this as `setup_robotis_ws.sh`:

```bash
#!/bin/bash
# Quick setup script for ROBOTIS workspace

set -e

echo "Setting up ROBOTIS workspace for ROS 2 Jazzy..."

# Update system
sudo apt update && sudo apt upgrade -y

# Install ROS 2 Jazzy (if not already installed)
if ! command -v ros2 &> /dev/null; then
    echo "Installing ROS 2 Jazzy..."
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
    sudo apt update
    sudo apt install -y ros-jazzy-desktop-full
fi

# Install dependencies
sudo apt install -y \
    ros-jazzy-hardware-interface \
    ros-jazzy-controller-manager \
    ros-jazzy-ros2-controllers \
    ros-jazzy-ros2-control \
    ros-jazzy-gz-ros2-control \
    ros-jazzy-tf-transformations \
    ros-jazzy-xacro \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-rviz2 \
    ros-jazzy-gazebo-* \
    ros-jazzy-gz-* \
    ros-jazzy-pal-statistics \
    libboost-all-dev

sudo apt install -y ros-jazzy-moveit-* --no-install-recommends

# Create workspace
mkdir -p ~/robotis_ws/src
cd ~/robotis_ws/src

# Clone repositories
git clone -b jazzy https://github.com/ROBOTIS-GIT/DynamixelSDK.git
git clone -b jazzy https://github.com/ROBOTIS-GIT/dynamixel_interfaces.git
git clone -b jazzy https://github.com/ROBOTIS-GIT/dynamixel_hardware_interface.git
git clone -b jazzy https://github.com/ROBOTIS-GIT/open_manipulator.git

# Install dependencies using rosdep
cd ~/robotis_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# Build workspace
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# Setup environment
if ! grep -q "source /opt/ros/jazzy/setup.bash" ~/.bashrc; then
    echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
fi
if ! grep -q "source ~/robotis_ws/install/setup.bash" ~/.bashrc; then
    echo "source ~/robotis_ws/install/setup.bash" >> ~/.bashrc
fi
if ! grep -q "export ROBOT_MODEL=om_x" ~/.bashrc; then
    echo "export ROBOT_MODEL=om_x" >> ~/.bashrc
fi
if ! grep -q "export ROS_DOMAIN_ID=30" ~/.bashrc; then
    echo "export ROS_DOMAIN_ID=30" >> ~/.bashrc
fi

# Setup USB permissions
sudo usermod -aG dialout $USER

# Create udev rules
sudo bash -c 'cat > /etc/udev/rules.d/99-open-manipulator-cdc.rules << EOF
SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6014", MODE="0666", GROUP="dialout", SYMLINK+="ttyUSB_OpenManipulator"
SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6001", MODE="0666", GROUP="dialout"
EOF'

sudo udevadm control --reload-rules
sudo udevadm trigger

echo "Setup complete! Please log out and log back in to apply group changes."
echo "Then test with: ros2 launch open_manipulator_bringup gazebo.launch.py"
```

To use this script:

```bash
chmod +x setup_robotis_ws.sh
./setup_robotis_ws.sh
```

---

**Note**: This guide assumes a clean Ubuntu 24.04 installation. If you have existing ROS installations or custom configurations, some steps may need to be adapted. Always backup important data before making system changes.

**Support**: For issues specific to the Open Manipulator packages, refer to the [official ROBOTIS documentation](https://github.com/ROBOTIS-GIT/open_manipulator) or the [ROS 2 documentation](https://docs.ros.org/en/jazzy/). 