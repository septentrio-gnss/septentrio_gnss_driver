import os
import launch
from launch_ros.actions import Node
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory

os.environ['RCUTILS_CONSOLE_OUTPUT_FORMAT'] = '{time}: [{name}] [{severity}]\t{message}'
# Verbose log:
#os.environ['RCUTILS_CONSOLE_OUTPUT_FORMAT'] = '{time}: [{name}] [{severity}]\t{message} ({function_name}() at {file_name}:{line_number})'

config = os.path.join(
    get_package_share_directory('septentrio_gnss_driver'),
    'config',
    'rover.yaml'
    )

# Start as component:

def generate_launch_description():

    composable_node = ComposableNode(
        name='septentrio_gnss_driver',
        package='septentrio_gnss_driver', plugin='rosaic_node::ROSaicNode',
        parameters=[config])

    container = ComposableNodeContainer(
        name='septentrio_gnss_driver_container',
        namespace='septentrio_gnss_driver',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[composable_node],
        output='screen'
    )

    return launch.LaunchDescription([container])
