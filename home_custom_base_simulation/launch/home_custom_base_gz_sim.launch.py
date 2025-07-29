from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def launch_setup(context, *args, **kwargs):
    robot_type = LaunchConfiguration("robot_type")
    
    runtime_config_package = LaunchConfiguration("runtime_config_package")
    controllers_file = LaunchConfiguration("controllers_file")
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    prefix = LaunchConfiguration("prefix")
    launch_rviz = LaunchConfiguration("launch_rviz")
    gazebo_gui = LaunchConfiguration("gazebo_gui")
    world_file = LaunchConfiguration("world_file")
    use_sim_time = LaunchConfiguration("use_sim_time")

    initial_joint_controllers = PathJoinSubstitution(
        [FindPackageShare(runtime_config_package), "config", controllers_file]
    )
    
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(description_package), "rviz", "robot.rviz"]
    )
    
    # Use the simulation-specific robot description
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare("home_custom_base_description"), "urdf", "robot.xacro"]),
            " ",
            "robot_type:=", robot_type,
            " ",
            "prefix:=", prefix,
            " ",
            "use_sim_time:=", use_sim_time,
        ]
    )

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[{
            "robot_description": ParameterValue(robot_description_content, value_type=str),
            "use_sim_time": use_sim_time
        }],
    )

    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"])]
        ),
        launch_arguments={"gz_args": ["-r -v1 ", world_file], "on_exit_shutdown": "true"}.items(),
        condition=IfCondition(gazebo_gui),
    )

    # Spawn robot
    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-topic", "robot_description",
            "-name", "home_base",
            "-allow_renaming", "true",
        ],
        output="screen",
    )

    # Controller manager
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[initial_joint_controllers, {"use_sim_time": use_sim_time}],
        output="both",
        remappings=[
            ("~/robot_description", "/robot_description"),
        ],
    )

    # Joint state broadcaster spawner
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    # Mecanum drive controller spawner
    # mecanum_drive_spawner = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=["mecanum_controller", "--controller-manager", "/controller_manager"],
    # )
    
    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broadcaster"],
    )
    
    mecanum_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["mecanum_controller"],
    )

    # Delayed controller start (after robot is spawned)
    delayed_controller_manager = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[controller_manager],
        )
    )

    delayed_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=controller_manager,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )

    delayed_mecanum_drive_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[mecanum_drive_spawner],
        )
    )

    # RViz
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(launch_rviz),
        parameters=[{"use_sim_time": use_sim_time}],
    )

    nodes = [
        gazebo,
        robot_state_pub_node,
        spawn_entity,
        # delayed_controller_manager,
        # delayed_joint_state_broadcaster_spawner,
        # delayed_mecanum_drive_spawner,
        joint_broad_spawner,
        mecanum_drive_spawner,
        rviz_node,
    ]

    return nodes

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "robot_type",
            default_value="home_base",
            description="Type of robot to launch",
        ),
        DeclareLaunchArgument(
            "runtime_config_package",
            default_value="home_custom_base_simulation",
            description="Package with the controller's configuration in 'config' folder",
        ),
        DeclareLaunchArgument(
            "controllers_file",
            default_value="ros2_control_mecanum.yaml",
            description="YAML file with the controllers configuration",
        ),
        DeclareLaunchArgument(
            "description_package",
            default_value="home_custom_base_description",
            description="Description package with robot URDF/xacro files",
        ),
        DeclareLaunchArgument(
            "description_file",
            default_value="robot.xacro",
            description="URDF/XACRO description file with the robot",
        ),
        DeclareLaunchArgument(
            "prefix",
            default_value="home_base",
            description="Prefix of the joint names",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Start robot in Gazebo simulation",
        ),
        DeclareLaunchArgument(
            "launch_rviz",
            default_value="true",
            description="Launch RViz",
        ),
        DeclareLaunchArgument(
            "gazebo_gui",
            default_value="true",
            description="Launch Gazebo GUI",
        ),
        DeclareLaunchArgument(
            "world_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("home_custom_base_simulation"), "worlds", "empty.sdf"
            ]),
            description="Full path to the world model file to load",
        ),
        OpaqueFunction(function=launch_setup)
    ])