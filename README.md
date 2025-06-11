# ROBOTIS Open Manipulator Workspace

[![CI](https://github.com/alwinsdon/robotis_ws/actions/workflows/ci.yml/badge.svg)](https://github.com/alwinsdon/robotis_ws/actions/workflows/ci.yml)
[![ROS 2 Jazzy](https://img.shields.io/badge/ROS%202-Jazzy-blue)](https://docs.ros.org/en/jazzy/)
[![Ubuntu 24.04](https://img.shields.io/badge/Ubuntu-24.04-orange)](https://releases.ubuntu.com/24.04/)

This workspace contains all necessary packages for working with ROBOTIS Open Manipulator robots using ROS 2 Jazzy.

## Quick Setup

### Option 1: Clone this Repository

```bash
git clone https://github.com/USERNAME/robotis_ws.git
cd robotis_ws
./setup_robotis_ws.sh
```

### Option 2: Direct Script Download

For a new PC with Ubuntu 24.04, download and run the setup script directly:

```bash
wget https://raw.githubusercontent.com/alwinsdon/robotis_ws/main/setup_robotis_ws.sh
chmod +x setup_robotis_ws.sh
./setup_robotis_ws.sh
```

### Option 3: Using vcstool

```bash
mkdir -p ~/robotis_ws/src
cd ~/robotis_ws
wget https://raw.githubusercontent.com/alwinsdon/robotis_ws/main/robotis_ws.repos
vcs import src < robotis_ws.repos
# Then follow manual setup instructions
```

## Manual Setup

For detailed step-by-step instructions, see:
- **[ROBOTIS_WORKSPACE_SETUP_GUIDE.md](ROBOTIS_WORKSPACE_SETUP_GUIDE.md)** - Complete setup guide

## What's Included

- **DynamixelSDK** - Low-level Dynamixel motor communication
- **Dynamixel Hardware Interface** - ROS 2 hardware interface for Dynamixel motors
- **Open Manipulator** - Complete Open Manipulator packages including:
  - Hardware interface
  - Bringup scripts
  - Robot description (URDF/XACRO)
  - MoveIt configuration
  - Gazebo simulation
  - Teleoperation tools
  - GUI applications

## Supported Robots

- **Open Manipulator-X** (om_x)
- **Open Manipulator-Y** (om_y)

## Requirements

- Ubuntu 24.04 LTS
- ROS 2 Jazzy
- Hardware: USB2DYNAMIXEL adapter + Dynamixel XM430-W350 motors

## Quick Commands

After setup, use these convenient aliases:

```bash
# Simulation and visualization
om_gazebo     # Launch Gazebo simulation
om_viz        # Launch RViz visualization only
om_moveit     # Launch MoveIt motion planning

# Hardware control
om_hardware   # Launch hardware interface
om_teleop     # Launch keyboard teleoperation

# Manual commands
ros2 launch open_manipulator_bringup gazebo.launch.py
ros2 launch open_manipulator_bringup hardware_x.launch.py port_name:=/dev/ttyUSB0
ros2 launch open_manipulator_moveit_config moveit_core.launch.py
```

## Troubleshooting

1. **USB Permission Issues**: Ensure you're in the `dialout` group
2. **Package Not Found**: Source the workspace: `source install/setup.bash`
3. **Hardware Communication**: Check USB connection and motor power
4. **Build Errors**: Run `rosdep install --from-paths src --ignore-src -r -y`

For detailed troubleshooting, see the complete setup guide.

## Workspace Structure

```
robotis_ws/
├── src/
│   ├── DynamixelSDK/           # Dynamixel SDK
│   ├── dynamixel_interfaces/   # ROS 2 interfaces
│   ├── dynamixel_hardware_interface/  # Hardware interface
│   └── open_manipulator/       # Main robot packages
├── build/                      # Build files
├── install/                    # Installed packages
└── log/                        # Build logs
```

## Documentation

- [Official ROBOTIS Documentation](https://github.com/ROBOTIS-GIT/open_manipulator)
- [ROS 2 Jazzy Documentation](https://docs.ros.org/en/jazzy/)
- [Open Manipulator Manual](https://emanual.robotis.com/docs/en/platform/openmanipulator_x/)

## Contributing

### Development Workflow

1. Fork this repository
2. Create a feature branch: `git checkout -b feature/amazing-feature`
3. Make your changes
4. Test locally: `colcon build && colcon test`
5. Commit your changes: `git commit -m 'Add amazing feature'`
6. Push to the branch: `git push origin feature/amazing-feature`
7. Open a Pull Request

### CI/CD

This repository uses GitHub Actions for continuous integration:
- Builds on Ubuntu 24.04 with ROS 2 Jazzy
- Runs automated tests
- Validates package dependencies

### Issue Reporting

Please use GitHub Issues to report:
- Hardware compatibility problems
- Build failures
- Feature requests
- Documentation improvements

## License

This workspace includes packages under various open-source licenses. See individual package LICENSE files for details.

## Support

- **GitHub Issues**: For bugs and feature requests
- **Discussions**: For questions and community support
- **Wiki**: For additional documentation and tutorials 