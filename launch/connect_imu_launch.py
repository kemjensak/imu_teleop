import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ur_moveit_config.launch_common import load_yaml
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():

    waist_imu_id = LaunchConfiguration("waist_imu_id")
    shoulder_imu_id = LaunchConfiguration("shoulder_imu_id")
    elbow_imu_id = LaunchConfiguration("elbow_imu_id")
    wrist_imu_id = LaunchConfiguration("wrist_imu_id")

    waist_imu_id_launch_arg = DeclareLaunchArgument(
        "waist_imu_id",
        default_value="00B4BAEF",
        description="ID of the waist IMU.",
    )

    shoulder_imu_id_launch_arg = DeclareLaunchArgument(
        "shoulder_imu_id",
        default_value="00B4BAF0",
        description="ID of the shoulder IMU.",
    )

    elbow_imu_id_launch_arg = DeclareLaunchArgument(
        "elbow_imu_id",
        default_value="00B4BAF6",
        description="ID of the elbow IMU.",
    )
    
    wrist_imu_id_launch_arg = DeclareLaunchArgument(
        "wrist_imu_id",
        default_value="00B4BAFC",
        description="ID of the wrist IMU.",
    )

    xsens_mtw_driver_node = Node(
        package="xsens_mtw_driver",
        executable="mt_w_manager",
        output="screen",
    )
    
    calibrated_imu_publisher_node = Node(
        package="imu_teleop",
        executable="calibrated_imu_publisher.py",
        parameters=[
            {"waist_imu_id": waist_imu_id},
            {"shoulder_imu_id": shoulder_imu_id},
            {"elbow_imu_id": elbow_imu_id},
            {"wrist_imu_id": wrist_imu_id},
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            waist_imu_id_launch_arg,
            shoulder_imu_id_launch_arg,
            elbow_imu_id_launch_arg,
            wrist_imu_id_launch_arg,
            calibrated_imu_publisher_node,
            # xsens_mtw_driver_node,
        ]
    )