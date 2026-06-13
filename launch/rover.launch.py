import os
import launch
from launch_ros.actions import Node
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, TextSubstitution
from ament_index_python.packages import get_package_share_directory

os.environ['RCUTILS_CONSOLE_OUTPUT_FORMAT'] = '{time}: [{name}] [{severity}]\t{message}'
# Verbose log:
#os.environ['RCUTILS_CONSOLE_OUTPUT_FORMAT'] = '{time}: [{name}] [{severity}]\t{message} ({function_name}() at {file_name}:{line_number})'

# Start as component:

def generate_launch_description():
    
    tf_imu = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments = "--x 0 --y 0 --z 0 --roll 0 --pitch 0 -- yaw 0 --frame-id base_link --child-frame-id imu".split(' ')
    )

    tf_gnss = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments = "--x 0 --y 0 --z 0 --roll 0 --pitch 0 -- yaw 0 --frame-id imu --child-frame-id gnss".split(' ')
    )

    tf_vsm = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments = "--x 0 --y 0 --z 0 --roll 0 --pitch 0 -- yaw 0 --frame-id imu --child-frame-id vsm".split(' ')
    )

    tf_aux1 = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments = "--x 0 --y -1 --z 0 --roll 0 --pitch 0 -- yaw 0 --frame-id imu --child-frame-id aux1".split(' ')
    )

    default_file_name = 'rover.yaml'
    name_arg_file_name = "file_name"
    arg_file_name = DeclareLaunchArgument(name_arg_file_name,
                                          default_value=TextSubstitution(text=str(default_file_name)))
    name_arg_file_path = 'path_to_config'
    arg_file_path = DeclareLaunchArgument(name_arg_file_path,
                                          default_value=[get_package_share_directory('septentrio_gnss_driver'), '/config/', LaunchConfiguration(name_arg_file_name)])

    composable_node = ComposableNode(
        name='septentrio_gnss_driver',
        package='septentrio_gnss_driver', 
        plugin='rosaic_node::ROSaicNode',
        #emulate_tty=True,
        parameters=[LaunchConfiguration(name_arg_file_path)])

    container = ComposableNodeContainer(
        name='septentrio_gnss_driver_container',
        namespace='septentrio_gnss_driver',
        package='rclcpp_components',
        executable='component_container_isolated',
        emulate_tty=True,
        sigterm_timeout = '20',
        composable_node_descriptions=[composable_node],
        output='screen'
    )

    return launch.LaunchDescription([arg_file_name, arg_file_path, container, tf_imu, tf_gnss, tf_vsm, tf_aux1])
