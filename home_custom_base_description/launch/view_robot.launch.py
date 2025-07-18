import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch.conditions import LaunchConfigurationEquals, IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    declared_arguments = []
    
    #TODO: Enable multiple robot types in the future
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_type",
            default_value="home_base",
            description="Type of robot to launch (e.g., home_base, FRIDA_2025, etc.)",
            choices=[
                "home_base",
                "FRIDA_2025",
            ]
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "decription_file",
            default_value=PathJoinSubstitution(
                [FindPackageShare("home_custom_base_description"), "urdf", "robot.xacro"]
            ),
            description="Path to the robot description file (URDF/XACRO)"
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            "rviz_config_file",
            default_value=PathJoinSubstitution(
                [FindPackageShare("home_custom_base_description"), "rviz", "robot.rviz"]
            ),
            description="Path to the RViz configuration file"
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            "tf_prefix",
            default_value="",
            description="TF prefix for the robot in case of multiple robots"
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation (Gazebo) clock if true"
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
            description="Launch RViz if true"
        )
    )
    
    #Argument initialization
    robot_type = LaunchConfiguration("robot_type")
    description_file = LaunchConfiguration("decription_file")
    rviz_config_file = LaunchConfiguration("rviz_config_file")
    tf_prefix = LaunchConfiguration("tf_prefix")
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_rviz = LaunchConfiguration("use_rviz")
    
    robot_description_content = Command(
        [
            FindExecutable(name="xacro"),
            " ",
            description_file,
            " ",
            "robot_type:=", robot_type,
            " ",
            "tf_prefix:=", tf_prefix,
            " ",
            "use_sim_time:=", use_sim_time
        ]
    )
    
    robot_description = {
        "robot_description": ParameterValue(
            robot_description_content, value_type=str
        )
    }
    
    joint_state_publisher_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
    )
    
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description],
        output="both",
    )
    
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz",
        arguments=["-d", rviz_config_file],
        output="screen",
        condition=LaunchConfigurationEquals("use_rviz", "true"),
    )
    
    nodes_to_start = [
        joint_state_publisher_node,
        robot_state_publisher_node,
        rviz_node
    ]
    
    return LaunchDescription(
        declared_arguments + nodes_to_start
    )