from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
import os
from ament_index_python.packages import get_package_share_directory
def generate_launch_description():
    params = os.path.join(get_package_share_directory('modify_map_to_odom'), 'config', 'config.yaml')
    container = ComposableNodeContainer(
        name='multi_threaded_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            ComposableNode(
                package='modify_map_to_odom',
                plugin='modify_map_to_odom::ModifyMapToOdom',
                name='modify_map_to_odom_node',
                parameters=[params])
        ],
        output='screen'
    )

    return LaunchDescription([container])
