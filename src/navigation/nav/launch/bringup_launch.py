"""Start Nav2 after bringup/startup.launch.py has activated the map."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    nav_launch = Path(get_package_share_directory('nav')) / 'launch/navigation_launch.py'
    bringup = Path(get_package_share_directory('bringup'))
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(nav_launch)),
        launch_arguments={'use_sim_time': use_sim_time,
                          'params_file': params_file,
                          'autostart': autostart}.items())
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('params_file',
                              default_value=str(bringup / 'params/nav2_params.yaml')),
        DeclareLaunchArgument('autostart', default_value='true'),
        navigation,
    ])
