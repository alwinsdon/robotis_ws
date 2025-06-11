#!/bin/bash
# Quick setup script for ROBOTIS workspace on ROS 2 Jazzy
# This script automates the complete installation process

set -e

echo "=========================================="
echo "🤖 ROBOTIS Workspace Setup for ROS 2 Jazzy"
echo "=========================================="

# Function to print colored output
print_info() {
    echo -e "\033[1;34m[INFO]\033[0m $1"
}

print_success() {
    echo -e "\033[1;32m[SUCCESS]\033[0m $1"
}

print_warning() {
    echo -e "\033[1;33m[WARNING]\033[0m $1"
}

print_error() {
    echo -e "\033[1;31m[ERROR]\033[0m $1"
}

# Check if running on Ubuntu 24.04
print_info "Checking system compatibility..."
if [[ $(lsb_release -rs) != "24.04" ]]; then
    print_warning "This script is designed for Ubuntu 24.04. Your version: $(lsb_release -rs)"
    read -p "Do you want to continue? (y/N): " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
fi

# Update system
print_info "Updating system packages..."
sudo apt update && sudo apt upgrade -y

# Install essential tools
print_info "Installing essential tools..."
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

# Install Python dependencies
print_info "Installing Python dependencies..."
pip3 install --user \
    setuptools \
    wheel \
    vcstool \
    colcon-common-extensions

# Install ROS 2 Jazzy (if not already installed)
if ! command -v ros2 &> /dev/null; then
    print_info "Installing ROS 2 Jazzy..."
    
    # Add ROS 2 repository
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
    
    sudo apt update
    sudo apt install -y ros-jazzy-desktop-full
    print_success "ROS 2 Jazzy installed successfully!"
else
    print_success "ROS 2 is already installed."
fi

# Install ROS 2 dependencies
print_info "Installing ROS 2 packages and dependencies..."
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

# Install MoveIt 2
print_info "Installing MoveIt 2..."
sudo apt install -y ros-jazzy-moveit-* --no-install-recommends

# Create workspace
print_info "Creating ROBOTIS workspace..."
mkdir -p ~/robotis_ws/src
cd ~/robotis_ws/src

# Clone repositories
print_info "Cloning required repositories..."

# Option 1: Use vcstool if available and repos file exists
if command -v vcs &> /dev/null && [ -f "../robotis_ws.repos" ]; then
    print_info "Using vcstool with repos file..."
    cd ..
    vcs import src < robotis_ws.repos
    cd src
else
    # Option 2: Manual clone
    print_info "Cloning repositories manually..."
    git clone -b jazzy https://github.com/ROBOTIS-GIT/DynamixelSDK.git
    git clone -b jazzy https://github.com/ROBOTIS-GIT/dynamixel_interfaces.git
    git clone -b jazzy https://github.com/ROBOTIS-GIT/dynamixel_hardware_interface.git
    git clone -b jazzy https://github.com/ROBOTIS-GIT/open_manipulator.git
fi

# Install dependencies using rosdep
print_info "Installing workspace dependencies with rosdep..."
cd ~/robotis_ws

# Initialize rosdep if not already done
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    sudo rosdep init
fi

rosdep update
rosdep install --from-paths src --ignore-src -r -y

# Build workspace
print_info "Building workspace... This may take several minutes."
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

if [ $? -eq 0 ]; then
    print_success "Workspace built successfully!"
else
    print_error "Workspace build failed!"
    exit 1
fi

# Setup environment variables
print_info "Setting up environment variables..."

# Backup .bashrc
cp ~/.bashrc ~/.bashrc.backup

# Add ROS 2 sourcing
if ! grep -q "source /opt/ros/jazzy/setup.bash" ~/.bashrc; then
    echo "# ROS 2 Jazzy" >> ~/.bashrc
    echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
fi

# Add workspace sourcing
if ! grep -q "source ~/robotis_ws/install/setup.bash" ~/.bashrc; then
    echo "# ROBOTIS Workspace" >> ~/.bashrc
    echo "export WORKSPACE_DIR=~/robotis_ws" >> ~/.bashrc
    echo "source ~/robotis_ws/install/setup.bash" >> ~/.bashrc
fi

# Add robot configuration
if ! grep -q "export ROBOT_MODEL=om_x" ~/.bashrc; then
    echo "# Robot Configuration" >> ~/.bashrc
    echo "export ROBOT_MODEL=om_x  # Change to om_y if using Open Manipulator-Y" >> ~/.bashrc
    echo "export ROS_DOMAIN_ID=30" >> ~/.bashrc
fi

# Setup USB permissions
print_info "Setting up USB permissions..."
sudo usermod -aG dialout $USER

# Create udev rules
print_info "Creating udev rules for USB2DYNAMIXEL..."
sudo bash -c 'cat > /etc/udev/rules.d/99-open-manipulator-cdc.rules << EOF
SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6014", MODE="0666", GROUP="dialout", SYMLINK+="ttyUSB_OpenManipulator"
SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6001", MODE="0666", GROUP="dialout"
EOF'

sudo udevadm control --reload-rules
sudo udevadm trigger

print_success "udev rules created successfully!"

# Create helpful aliases (optional)
print_info "Creating helpful aliases..."
if ! grep -q "alias om_gazebo" ~/.bashrc; then
    echo "# ROBOTIS Open Manipulator Aliases" >> ~/.bashrc
    echo "alias om_gazebo='ros2 launch open_manipulator_bringup gazebo.launch.py'" >> ~/.bashrc
    echo "alias om_hardware='ros2 launch open_manipulator_bringup hardware_x.launch.py port_name:=/dev/ttyUSB0'" >> ~/.bashrc
    echo "alias om_moveit='ros2 launch open_manipulator_moveit_config moveit_core.launch.py'" >> ~/.bashrc
    echo "alias om_viz='ros2 launch open_manipulator_description model_x.launch.py'" >> ~/.bashrc
    echo "alias om_teleop='ros2 run open_manipulator_teleop keyboard_control_x.py'" >> ~/.bashrc
fi

echo ""
echo "=========================================="
print_success "🎉 Setup Complete!"
echo "=========================================="
echo ""
print_info "What was installed:"
echo "  ✓ ROS 2 Jazzy Desktop Full"
echo "  ✓ All required ROS 2 packages"
echo "  ✓ MoveIt 2"
echo "  ✓ DynamixelSDK"
echo "  ✓ Dynamixel Hardware Interface"
echo "  ✓ Open Manipulator packages"
echo "  ✓ USB permissions and udev rules"
echo "  ✓ Environment configuration"
echo ""
print_info "Available aliases:"
echo "  - om_gazebo   : Launch Gazebo simulation"
echo "  - om_hardware : Launch hardware interface"
echo "  - om_moveit   : Launch MoveIt planning"
echo "  - om_viz      : Launch RViz visualization"
echo "  - om_teleop   : Launch keyboard control"
echo ""
print_warning "IMPORTANT NEXT STEPS:"
echo "1. Log out and log back in (or restart) to apply group changes"
echo "2. Source your environment: source ~/.bashrc"
echo "3. Test installation: om_gazebo"
echo ""
print_info "Hardware setup (if you have the robot):"
echo "1. Connect USB2DYNAMIXEL adapter"
echo "2. Power on the Open Manipulator"
echo "3. Test with: om_hardware"
echo ""
print_info "For troubleshooting, refer to the ROBOTIS_WORKSPACE_SETUP_GUIDE.md"
echo "==========================================" 