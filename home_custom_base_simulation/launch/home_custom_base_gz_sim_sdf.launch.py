from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import LaunchConfigurationEquals
from launch.substitutions import FindExecutable
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.conditions import LaunchConfigurationEquals, IfCondition
from launch.actions import DeclareLaunchArgument
import os

def generate_launch_description():
    # Declare arguments
    declared_arguments = [
        DeclareLaunchArgument(
            "rviz_config_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("home_custom_base_description"), "rviz", "robot.rviz"
            ]),
            description="Path to the RViz configuration file"
        ),
        DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
            description="Launch RViz if true"
        )
    ]

    # Generate robot_description from Xacro
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ',
        PathJoinSubstitution([
            FindPackageShare('home_custom_base_description'),
            'xacro',
            'robot.xacro'
        ])
    ])
    robot_description = {"robot_description": robot_description_content}

    # Nodes
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description],
        output="screen"
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration("rviz_config_file")],
        condition=LaunchConfigurationEquals("use_rviz", "true")
    )
    
    # Path to the robot.sdf
    sdf_path = os.path.join(
        get_package_share_directory('home_custom_base_description'),
        'examples',
        'robot.sdf'
    )

    # Start Gazebo with the SDF robot
    gazebo_launch = ExecuteProcess(
        cmd=[
            'ign', 'gazebo', sdf_path,
            '--gui-config', '/usr/share/ign-gazebo/gui.config',
            '-r'  # this means "run immediately"
        ],
        output='screen'
    )


    # Start joint state bridge
    joint_state_bridge = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        arguments=[
            '/world/Moving_robot/model/home_base/joint_state@sensor_msgs/msg/JointState@ignition.msgs.Model'
        ],
        remappings=[
            ('/world/Moving_robot/model/home_base/joint_state', '/joint_states')
        ],
        output='screen'
    )

    # You can add more bridges for e.g., /cmd_vel, /laser_scan, etc.
    # Example for cmd_vel:
    cmd_vel_bridge = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        arguments=['/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist'],
        output='screen'
    )

    return LaunchDescription(declared_arguments + [
        robot_state_publisher_node,
        # joint_state_publisher_node,
        rviz_node,
        gazebo_launch,
        joint_state_bridge,
        cmd_vel_bridge
    ])

