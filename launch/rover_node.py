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

    default_file_name = 'rover_node.yaml'
    name_arg_file_name = "file_name"
    arg_file_name = DeclareLaunchArgument(name_arg_file_name,
                                          default_value=TextSubstitution(text=str(default_file_name)),
                                          description="if true then the joint state gui is displayed (with sliders in order to set the joint positions.)")
    name_arg_file_path = 'path_to_config'
    arg_file_path = DeclareLaunchArgument(name_arg_file_path,
                                          default_value=[get_package_share_directory('septentrio_gnss_driver'), '/config/', LaunchConfiguration(name_arg_file_name)])

    node = Node(
            package='septentrio_gnss_driver',
            executable='septentrio_gnss_driver_node',
            name='septentrio_gnss_driver',
            parameters=[LaunchConfiguration(name_arg_file_path)])

    return launch.LaunchDescription([arg_file_name, arg_file_path, node])