# Home Custom Base Simulation

This package contains the simulation-specific configurations and launch files for the home custom base robot in Gazebo Ignition.

## Structure

```
home_custom_base_simulation/
├── config/
│   └── home_base_controllers.yaml    # ROS2 Control configuration
├── launch/
│   └── home_custom_base_gz_sim.launch.py  # Main simulation launch file
└── urdf/
    ├── home_base_ros2_control.xacro       # ROS2 Control definitions
    ├── home_base_gazebo.xacro             # Gazebo-specific configurations
    └── home_base_simulation.xacro         # Complete simulation robot description
```

## Features

- **ROS2 Control Integration**: Configured with mecanum drive controller for omnidirectional movement
- **Gazebo Ignition Support**: Optimized for Gazebo Ignition simulation
- **Modular Design**: Simulation components are separated from the base robot description
- **Sensor Support**: Optional camera and lidar sensors
- **IMU Integration**: Built-in IMU sensor for orientation feedback

## Usage

### Launch Simulation

To launch the robot in Gazebo Ignition:

```bash
ros2 launch home_custom_base_simulation home_custom_base_gz_sim.launch.py
```

### Launch Arguments

- `robot_type`: Type of robot (default: "simulation")
- `prefix`: Robot namespace prefix (default: "home_base")
- `use_sim_time`: Use simulation time (default: "true")
- `launch_rviz`: Launch RViz visualization (default: "true")
- `gazebo_gui`: Launch Gazebo GUI (default: "true")
- `world_file`: Path to Gazebo world file (default: empty)
- `add_camera`: Add camera sensor (default: "true")
- `add_lidar`: Add lidar sensor (default: "true")

### Example with Custom Arguments

```bash
ros2 launch home_custom_base_simulation home_custom_base_gz_sim.launch.py \
    prefix:=robot1 \
    launch_rviz:=false \
    add_camera:=false
```

## Control

The robot uses a mecanum drive controller that accepts `geometry_msgs/Twist` commands:

```bash
# Move forward
ros2 topic pub /home_base_controller/cmd_vel geometry_msgs/msg/Twist \
    '{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'

# Move sideways (mecanum specific)
ros2 topic pub /home_base_controller/cmd_vel geometry_msgs/msg/Twist \
    '{linear: {x: 0.0, y: 0.5, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'

# Rotate
ros2 topic pub /home_base_controller/cmd_vel geometry_msgs/msg/Twist \
    '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}'
```

## Available Topics

- `/home_base_controller/cmd_vel` - Velocity commands
- `/home_base_controller/odom` - Odometry data
- `/home_base/imu` - IMU data
- `/home_base/camera/image` - Camera image (if enabled)
- `/home_base/scan` - Lidar scan data (if enabled)
- `/joint_states` - Joint state information

## Controllers

The simulation uses the following ROS2 controllers:

1. **Joint State Broadcaster**: Publishes joint states
2. **Mecanum Drive Controller**: Handles omnidirectional movement

## Dependencies

- ros_gz_sim
- gz_ros2_control
- controller_manager
- mecanum_drive_controller
- joint_state_broadcaster
- robot_state_publisher
