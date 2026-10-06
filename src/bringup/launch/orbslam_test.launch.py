"""Bench-only ORB-SLAM3 comparison entry; reuse the common startup chain."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    startup = Path(get_package_share_directory('bringup')) / 'launch/startup.launch.py'
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(startup)),
            launch_arguments={'estimator': 'orbslam', 'flight_control': 'false'}.items()),
    ])
