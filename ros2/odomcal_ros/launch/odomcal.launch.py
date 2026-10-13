import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("odomcal_ros")
    default_params = os.path.join(share, "config", "default.yaml")
    default_calibration = os.path.join(share, "config", "calibration.yaml")

    calibration_arg = DeclareLaunchArgument(
        "calibration_file",
        default_value=default_calibration,
        description="Flat YAML file with r_l, r_r, and track_width in metres.",
    )

    calibrator = Node(
        package="odomcal_ros",
        executable="odomcal_node",
        name="odom_calibrator",
        output="screen",
        parameters=[
            default_params,
            {"calibration_file": LaunchConfiguration("calibration_file")},
        ],
    )

    return LaunchDescription([calibration_arg, calibrator])
