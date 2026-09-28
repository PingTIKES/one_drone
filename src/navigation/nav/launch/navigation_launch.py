"""Official Nav2 servers using the field parameters in the bringup package."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    bringup = Path(get_package_share_directory('bringup'))
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')
    configured_params = RewrittenYaml(
        source_file=params_file,
        param_rewrites={'use_sim_time': use_sim_time},
        convert_types=True)

    planner_server = Node(
        package='nav2_planner', executable='planner_server',
        name='planner_server', output='screen', parameters=[configured_params])
    controller_server = Node(
        package='nav2_controller', executable='controller_server',
        name='controller_server', output='screen', parameters=[configured_params])
    bt_navigator = Node(
        package='nav2_bt_navigator', executable='bt_navigator',
        name='bt_navigator', output='screen',
        # All operator goals must pass GoalManager's map and sensor gates.
        remappings=[('goal_pose', 'nav2_internal_goal_pose')],
        parameters=[configured_params,
                    {'default_nav_to_pose_bt_xml': str(bringup / 'behavior_trees/navigate.xml'),
                     'default_nav_through_poses_bt_xml': str(bringup / 'behavior_trees/navigate_through_poses.xml')}])
    velocity_smoother = Node(
        package='nav2_velocity_smoother', executable='velocity_smoother',
        name='velocity_smoother', output='screen', parameters=[configured_params])
    lifecycle_manager_navigation = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_navigation', output='screen',
        parameters=[{'use_sim_time': use_sim_time, 'autostart': autostart,
                     'node_names': ['planner_server', 'controller_server',
                                    'bt_navigator', 'velocity_smoother']}])

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('params_file',
                              default_value=str(bringup / 'params/nav2_params.yaml')),
        DeclareLaunchArgument('autostart', default_value='true'),
        planner_server,
        controller_server,
        bt_navigator,
        velocity_smoother,
        lifecycle_manager_navigation,
    ])
