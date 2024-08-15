import os
import launch
from launch_ros.actions import Node
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, TextSubstitution
from ament_index_python.packages import get_package_share_directory

os.environ['RCUTILS_CONSOLE_OUTPUT_FORMAT'] = '{time}: [{name}] [{severity}]\t{message}'

def generate_launch_description():
    
    tf_imu = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments = "0 0 0 0 0 0 base_link imu".split(' ')
    )

    tf_gnss = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments = "0 0 0 0 0 0 imu gnss".split(' ')
    )

    tf_vsm = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments = "0 0 0 0 0 0 imu vsm".split(' ')
    )

    tf_aux1 = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments = "0 0 0 0 0 0 imu aux1".split(' ')
    )

    parameters_list = [
    {
        "covariance_threshold": 0.0,
        "device": "tcp://10.27.1.102:28784",
        "serial": {
            "baudrate": 921600,
            "hw_flow_control": "off"
        },
        "stream_device": {
            "tcp": {
                "ip_server": "",
                "port": 0
            },
            "udp": {
                "ip_server": "",
                "port": 0,
                "unicast_ip": ""
            }
        },
        "configure_rx": True,
        "custom_commands_file": "",
        "login": {
            "user": "",
            "password": ""
        },
        "osnma": {
            "mode": "off"
        },
        "ntp_server": "",
        "keep_open": True,
        "frame_id": "septentrio_gnss",
        "imu_frame_id": "imu",
        "poi_frame_id": "base_link",
        "vsm_frame_id": "vsm",
        "aux1_frame_id": "aux1",
        "vehicle_frame_id": "base_link",
        "local_frame_id": "odom",
        "insert_local_frame": False,
        "get_spatial_config_from_tf": False,
        "lock_utm_zone": True,
        "use_ros_axis_orientation": True,
        "receiver_type": "gnss",
        "multi_antenna": True,
        "datum": "Default",
        "poi_to_arp": {
            "delta_e": 0.0,
            "delta_n": 0.0,
            "delta_u": 0.0
        },
        "att_offset": {
            "heading": 0.0,
            "pitch": 0.0
        },
        "ant_type": "Unknown",
        "ant_serial_nr": "Unknown",
        "ant_aux1_type": "Unknown",
        "ant_aux1_serial_nr": "Unknown",
        "polling_period": {
            "pvt": 100,
            "rest": 500
        },
        "use_gnss_time": False,
        "ntp_server": True,
        "ptp_server_clock": False,
        "latency_compensation": True,
        "rtk_settings": {
            "ntrip_1": {
                "id": "",
                "caster": "",
                "caster_port": 2101,
                "username": "",
                "password": "",
                "mountpoint": "",
                "version": "v2",
                "tls": False,
                "fingerprint": "",
                "rtk_standard": "auto",
                "send_gga": "auto",
                "keep_open": True
            },
            "ip_server_1": {
                "id": "",
                "port": 0,
                "rtk_standard": "auto",
                "send_gga": "auto",
                "keep_open": True
            },
            "serial_1": {
                "port": "",
                "baud_rate": 115200,
                "rtk_standard": "auto",
                "send_gga": "auto",
                "keep_open": True
            }
        },
        "publish": {
            "auto_publish": False,
            "publish_only_valid": False,
            "navsatfix": False,
            "gpsfix": False,
            "gpgga": False,
            "gprmc": False,
            "gpst": False,
            "measepoch": False,
            "pvtcartesian": False,
            "pvtgeodetic": False,
            "basevectorcart": False,
            "basevectorgeod": False,
            "poscovcartesian": False,
            "poscovgeodetic": False,
            "velcovcartesian": False,
            "velcovgeodetic": False,
            "atteuler": False,
            "attcoveuler": False,
            "pose": False,
            "pose_stamped": True,
            "geopose_stamped": True,
            "geopose_covariance_stamped": False,
            "twist": False,
            "twist_stamped": True,
            "diagnostics": True,
            "aimplusstatus": False,
            "galauthstatus": False,
            "gpgsa": False,
            "gpgsv": False,
            "insnavcart": False,
            "insnavgeod": False,
            "extsensormeas": False,
            "imusetup": False,
            "velsensorsetup": False,
            "exteventinsnavcart": False,
            "exteventinsnavgeod": False,
            "imu": False,
            "localization": False,
            "tf": False,
            "localization_ecef": False,
            "tf_ecef": False
        },
        "ins_spatial_config": {
            "imu_orientation": {
                "theta_x": 0.0,
                "theta_y": 0.0,
                "theta_z": 0.0
            },
            "poi_lever_arm": {
                "delta_x": 0.0,
                "delta_y": 0.0,
                "delta_z": 0.0
            },
            "ant_lever_arm": {
                "x": 0.0,
                "y": 0.0,
                "z": 0.0
            },
            "vsm_lever_arm": {
                "vsm_x": 0.0,
                "vsm_y": 0.0,
                "vsm_z": 0.0
            }
        },
        "ins_initial_heading": "auto",
        "ins_std_dev_mask": {
            "att_std_dev": 5.0,
            "pos_std_dev": 10.0
        },
        "ins_use_poi": False,
        "ins_vsm": {
            "ros": {
                "source": "",
                "config": [False, False, False],
                "variances_by_parameter": False,
                "variances": [0.0, 0.0, 0.0]
            },
            "ip_server": {
                "id": "",
                "port": 0,
                "keep_open": True
            },
            "serial": {
                "port": "",
                "baud_rate": 115200,
                "keep_open": True
            }
        },
        "activate_debug_log": False
    }
]
    
    composable_node = ComposableNode(
        name='septentrio_gnss',
        package='septentrio_gnss_driver', 
        plugin='rosaic_node::ROSaicNode',
        #emulate_tty=True,
        parameters=parameters_list)

    container = ComposableNodeContainer(
        name='septentrio_gnss_container',
        namespace='septentrio_gnss_driver',
        package='rclcpp_components',
        executable='component_container_isolated',
        emulate_tty=True,
        sigterm_timeout = '20',
        composable_node_descriptions=[composable_node],
        output='screen'
    )

    return launch.LaunchDescription([container, tf_imu, tf_gnss, tf_vsm, tf_aux1])
