import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    bringup_dir = get_package_share_directory("nav2_bringup")
    pkg_dir = get_package_share_directory("rb1_nav2_bringup")

    map_yaml = LaunchConfiguration("map")
    params_file = LaunchConfiguration("params_file")

    return LaunchDescription([

        DeclareLaunchArgument(
            "map",
            default_value=os.path.join(pkg_dir, "maps", "empty.yaml"),
            description="Full path to map yaml",
        ),

        DeclareLaunchArgument(
            "params_file",
            default_value=os.path.join(pkg_dir, "config", "nav2_params.yaml"),
            description="Full path to Nav2 parameters",
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup_dir, "launch", "bringup_launch.py")
            ),
            launch_arguments={
                "map": map_yaml,
                "params_file": params_file,
                "use_sim_time": "true",
            }.items(),
        ),
    ])
