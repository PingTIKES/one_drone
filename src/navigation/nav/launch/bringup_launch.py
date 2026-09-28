"""Start Nav2 separately from sensing and flight control."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    nav_launch = Path(get_package_share_directory('nav')) / 'launch/navigation_launch.py'
    bringup = Path(get_package_share_directory('bringup'))
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')

    def include_navigation(context):
        mode = LaunchConfiguration('navigation_mode').perform(context)
        if mode not in ('map', 'odom'):
            raise ValueError('navigation_mode must be map or odom')
        selected = LaunchConfiguration('params_file').perform(context)
        if not selected:
            filename = 'nav2_params.yaml' if mode == 'map' else 'nav2_odom_params.yaml'
            selected = str(bringup / 'params' / filename)
        return [IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(nav_launch)),
            launch_arguments={'use_sim_time': use_sim_time,
                              'params_file': selected,
                              'autostart': autostart}.items())]

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('navigation_mode', default_value='map'),
        DeclareLaunchArgument('params_file', default_value=''),
        DeclareLaunchArgument('autostart', default_value='true'),
        OpaqueFunction(function=include_navigation),
    ])
