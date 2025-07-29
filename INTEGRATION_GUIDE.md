# ROS2 Control and Gazebo Integration Guide

## Summary of Changes Made

### 1. **New Files Created**

#### Simulation Package (`home_custom_base_simulation/`):
- `urdf/home_base_ros2_control.xacro` - ROS2 Control configuration
- `urdf/home_base_gazebo.xacro` - Gazebo-specific properties and sensors
- `urdf/home_base_simulation.xacro` - Complete simulation robot description
- `config/home_base_controllers.yaml` - Controller configuration
- `README.md` - Documentation

#### Description Package (`home_custom_base_description/`):
- `launch/view_robot_standalone.launch.py` - Standalone robot visualization

### 2. **Modified Files**
- `robot.xacro` - Added conditional simulation support
- `home_custom_base_gz_sim.launch.py` - Updated for new architecture
- `package.xml` - Added necessary dependencies
- `CMakeLists.txt` - Added installation of new directories

## Key Features Implemented

### ✅ **ROS2 Control Integration**
- Mecanum drive controller for omnidirectional movement
- Joint state broadcaster for wheel position feedback
- Velocity command interface for each wheel
- Configurable controller parameters

### ✅ **Gazebo Ignition Support**
- Proper physics properties for wheels (friction, damping)
- IMU sensor integration
- Optional camera and lidar sensors
- Material properties for visualization

### ✅ **Modular Architecture**
- Simulation components separated from base robot description
- Conditional inclusion based on `use_sim_time` parameter
- Easy to maintain and extend

### ✅ **Sensor Integration**
- IMU sensor on base_link
- Optional camera sensor with configurable position
- Optional lidar sensor with configurable position
- Proper frame configurations

## Usage Examples

### Launch Simulation
```bash
# Basic simulation
ros2 launch home_custom_base_simulation home_custom_base_gz_sim.launch.py

# Simulation without sensors
ros2 launch home_custom_base_simulation home_custom_base_gz_sim.launch.py add_camera:=false add_lidar:=false

# Simulation with custom prefix
ros2 launch home_custom_base_simulation home_custom_base_gz_sim.launch.py prefix:=robot1
```

### Test Robot Description Only
```bash
# View robot in RViz without simulation
ros2 launch home_custom_base_description view_robot_standalone.launch.py

# View with simulation components included
ros2 launch home_custom_base_description view_robot_standalone.launch.py use_sim_time:=true
```

### Control the Robot
```bash
# Install teleop package if not available
sudo apt install ros-humble-teleop-twist-keyboard

# Control with keyboard
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/home_base_controller/cmd_vel
```

## Next Steps

### 1. **Build and Test**
```bash
cd /home/deivideich/home-custom-base
colcon build --packages-select home_custom_base_description home_custom_base_simulation
source install/setup.bash
```

### 2. **Verify Controllers**
```bash
# Check available controllers
ros2 control list_controllers

# Check hardware interface
ros2 control list_hardware_interfaces
```

### 3. **Monitor Topics**
```bash
# Joint states
ros2 topic echo /joint_states

# Odometry
ros2 topic echo /home_base_controller/odom

# IMU data
ros2 topic echo /home_base/imu
```

## Troubleshooting

### Common Issues:
1. **Controller not loading**: Check that all dependencies are installed
2. **Robot not spawning**: Verify URDF syntax with `xacro --inorder`
3. **Sensors not working**: Ensure Gazebo plugins are properly installed

### Dependencies to Install:
```bash
sudo apt install ros-humble-gz-ros2-control
sudo apt install ros-humble-mecanum-drive-controller
sudo apt install ros-humble-joint-state-broadcaster
sudo apt install ros-humble-controller-manager
```

This modular approach allows you to:
- Use the same base robot description for both simulation and real hardware
- Easily add/remove sensors for different scenarios
- Maintain clean separation between description and simulation components
- Scale to multiple robots with different prefixes
