'''import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue

from ament_index_python.packages import (
    get_package_share_directory,
    get_package_prefix,
)


def generate_launch_description():

    # ------------------------------------------------
    # Basic configuration
    # ------------------------------------------------
    description_pkg = "rb1_ros2_description"
    robot_name = "rb1_robot"

    use_sim_time = LaunchConfiguration("use_sim_time")
    gui = LaunchConfiguration("gui")

    world_file = "/usr/share/gazebo-11/worlds/empty.world"
    install_dir = get_package_prefix(description_pkg)

    # ------------------------------------------------
    # Gazebo paths (IMPORTANT for meshes & plugins)
    # ------------------------------------------------
    os.environ["GAZEBO_MODEL_PATH"] = (
        install_dir + "/share:" +
        os.environ.get("GAZEBO_MODEL_PATH", "")
    )

    os.environ["GAZEBO_PLUGIN_PATH"] = (
        install_dir + "/lib:" +
        os.environ.get("GAZEBO_PLUGIN_PATH", "")
    )

    # ------------------------------------------------
    # Launch arguments
    # ------------------------------------------------
    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation clock",
    )

    declare_gui = DeclareLaunchArgument(
        "gui",
        default_value="true",
        description="Launch Gazebo GUI",
    )

    # ------------------------------------------------
    # Gazebo (OFFICIAL ROS 2 LAUNCH)
    # ------------------------------------------------
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("gazebo_ros"),
                "launch",
                "gazebo.launch.py",
            )
        ),
        launch_arguments={
            "world": world_file,
            "gui": gui,
            "verbose": "true",
            "factory": "true",   # REQUIRED for spawn_entity
        }.items(),
    )

    # ------------------------------------------------
    # Robot description (Xacro → URDF)
    # ------------------------------------------------
    robot_desc_path = os.path.join(
        get_package_share_directory(description_pkg),
        "xacro",
        "rb1_ros2_base.urdf.xacro",
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "robot_description": ParameterValue(
                    Command(["xacro ", robot_desc_path]),
                    value_type=str,
                ),
            }
        ],
    )

    # ------------------------------------------------
    # Spawn robot
    # ------------------------------------------------
    spawn_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-entity", robot_name,
            "-topic", "/robot_description",
            "-x", "0.0",
            "-y", "0.0",
            "-z", "0.05",
        ],
        output="screen",
    )

    # ------------------------------------------------
    # controller_manager (THIS FIXES EVERYTHING)
    # ------------------------------------------------
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            os.path.join(
                get_package_share_directory(description_pkg),
                "config",
                "rb1_controller.yaml",
            )
        ],
        output="screen",
    )

    # ------------------------------------------------
    # Controllers
    # ------------------------------------------------
    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
    )

    diff_drive_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "diffbot_base_controller",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
    )

    lift_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "forward_effort_controller",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
    )

    # ------------------------------------------------
    # Launch order (CRITICAL)
    # ------------------------------------------------
    return LaunchDescription([
        declare_use_sim_time,
        declare_gui,

        gazebo,
        robot_state_publisher,
        spawn_robot,

        controller_manager,

        TimerAction(
            period=3.0,
            actions=[joint_state_broadcaster],
        ),

        RegisterEventHandler(
            OnProcessExit(
                target_action=joint_state_broadcaster,
                on_exit=[diff_drive_controller],
            )
        ),

        RegisterEventHandler(
            OnProcessExit(
                target_action=diff_drive_controller,
                on_exit=[lift_controller],
            )
        ),
    ])

'''
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue

from ament_index_python.packages import get_package_share_directory, get_package_prefix


def generate_launch_description():

    # ------------------------------------------------
    # Basic configuration
    # ------------------------------------------------
    description_pkg = "rb1_ros2_description"
    robot_name = "rb1_robot"

    use_sim_time = LaunchConfiguration("use_sim_time")
    gui = LaunchConfiguration("gui")

    world_file = "/usr/share/gazebo-11/worlds/cafe.world"

    install_dir = get_package_prefix(description_pkg)

    # ------------------------------------------------
    # Gazebo paths
    # ------------------------------------------------
    os.environ["GAZEBO_MODEL_PATH"] = (
        install_dir + "/share:" +
        os.environ.get("GAZEBO_MODEL_PATH", "")
    )

    os.environ["GAZEBO_PLUGIN_PATH"] = (
        install_dir + "/lib:" +
        os.environ.get("GAZEBO_PLUGIN_PATH", "")
    )

    # ------------------------------------------------
    # Launch arguments
    # ------------------------------------------------
    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation clock",
    )

    declare_gui = DeclareLaunchArgument(
        "gui",
        default_value="true",
        description="Launch Gazebo GUI",
    )

    # ------------------------------------------------
    # Gazebo (OFFICIAL ROS 2 LAUNCH)
    # ------------------------------------------------
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("gazebo_ros"),
                "launch",
                "gazebo.launch.py",
            )
        ),
        launch_arguments={
            "world": world_file,
            "gui": gui,
            "verbose": "true",
            "factory": "true",
        }.items(),
    )

    # ------------------------------------------------
    # Robot description (Xacro → URDF)
    # ------------------------------------------------
    robot_desc_path = os.path.join(
        get_package_share_directory(description_pkg),
        "xacro",
        "rb1_ros2_base.urdf.xacro",
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "robot_description": ParameterValue(
                Command(["xacro ", robot_desc_path]),
                value_type=str,
            ),
        }],
    )

    # ------------------------------------------------
    # Spawn robot into Gazebo
    # ------------------------------------------------
    spawn_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-entity", robot_name,
            "-topic", "/robot_description",
            "-x", "0.0",
            "-y", "0.0",
            "-z", "0.05",
        ],
        output="screen",
    )

    # ------------------------------------------------
    # Controllers (spawned AFTER robot exists)
    # ------------------------------------------------
    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )

    diff_drive_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diffbot_base_controller"],
        output="screen",
    )

    lift_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["forward_effort_controller"],
        output="screen",
    )

    spawn_controllers = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_robot,
            on_exit=[
                joint_state_broadcaster,
                diff_drive_controller,
                lift_controller,
            ],
        )
    )

    # ------------------------------------------------
    # Launch description
    # ------------------------------------------------
    return LaunchDescription([
        declare_use_sim_time,
        declare_gui,

        gazebo,
        robot_state_publisher,
        spawn_robot,

        spawn_controllers,
    ])
