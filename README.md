# ROSaic = ROS + mosaic

<img src="ROSaicLogo.png" width="60%">

## Overview
This repository hosts a ROS Melodic and Noetic driver (i.e. for Linux only) - written in C++ - that works with mosaic and AsteRx - two of Septentrio's cutting-edge GNSS/INS receiver families - and beyond. Since Noetic will only be supported until 2025, we plan to make ROSaic compatible with ROS2.

Main Features:
- Supports serial, TCP/IP and USB connections, the latter being compatible with both serial and TCP/IP protocols
- Supports several ASCII (including key NMEA ones) messages and SBF (Septentrio Binary Format) blocks
- Easy to add support for more log types
- Can play back PCAP capture logs for testing purposes
- Can blend SBF blocks `PVTGeodetic`, `PosCovGeodetic`, `ChannelStatus`, `MeasEpoch`, `AttEuler`, `AttCovEuler`, `VelCovGeodetic` and `DOP` in order to publish `gps_common/GPSFix` messages
- Tested with the mosaic-X5 receiver and the AsteRx-SBi3 Pro receiver

Please [let the maintainers know](mailto:githubuser@septentrio.com?subject=[GitHub]%20ROSaic) of your success or failure in using the driver with other devices so we can update this page appropriately.

## Dependencies
The `master` branch for this driver functions on both ROS Melodic (Ubuntu 18.04) and Noetic (Ubuntu 20.04). It is thus necessary to [install](https://wiki.ros.org/Installation/Ubuntu) the ROS version that has been designed for your Linux distro.<br><br>
The serial and TCP/IP communication interface of the ROS driver is established by means of the [Boost C++ library](https://www.boost.org/). In the unlikely event that the below installation instructions fail to install Boost on the fly, please install the Boost libraries via<br><br>
`sudo apt install libboost-all-dev`.<br><br>
Compatiblity with PCAP captures are incorporated through [pcap libraries](https://github.com/the-tcpdump-group/libpcap). Install the necessary headers via<br><br>
`sudo apt install libpcap-dev`.

## Usage
<details>
<summary>Binary Install</summary>
  
  The binary release is now available for Melodic and Noetic. To install the binary package on Melodic for instance, simply run `sudo apt-get install ros-melodic-septentrio-gnss-driver`.
</details>

<details>
<summary>Build from Source </summary>
  
  Alternatively, the package can also be built from source using [`catkin_tools`](https://catkin-tools.readthedocs.io/en/latest/installing.html), where the latter can be installed using the command
  `sudo apt-get install python-catkin-tools` for Melodic or `sudo apt-get install python3-catkin-tools` for Noetic. The typical `catkin_tools` [workflow](https://catkin-tools.readthedocs.io/en/latest/quick_start.html) should suffice:

  ```
  source /opt/ros/${ROS_DISTRO}/setup.bash                            # In case you do not use the default shell of Ubuntu, you need to source another script, e.g. setup.sh.
  mkdir -p ~/septentrio/src                                           # Note: Change accordingly dependending on where you want your package to be installed.
  cd ~/septentrio
  catkin init                                                         # Initialize with a hidden marker file
  catkin config --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo        # CMake build types pass compiler-specific flags to your compiler. This type amounts to a release with debug info, while keeping debugging symbols and doing optimization. I.e. for GCC the flags would be -O2, -g and -DNDEBUG.
  cd src
  git clone https://github.com/septentrio-gnss/septentrio_gnss_driver
  rosdep install . --from-paths -i                                    # Might raise "rosaic: Unsupported OS [mint]" warning, if your OS is Linux Mint, since rosdep does not know Mint (and possible other OSes). In that case, add the "--os=ubuntu:saucy" option to "fool" rosdep into believing it faces some Ubuntu version. The syntax is "--os=OS_NAME:OS_VERSION".
  catkin build                                                        # If catkin cannot find empy, tell catkin to use Python 3 by adding "-DPYTHON_EXECUTABLE=/usr/bin/python3".
  echo "source ~/septentrio/devel/setup.bash" >> ~/.bashrc            # It is convenient if the ROS environment variable is automatically added to your bash session every time a new shell is launched. Again, this works for bash shells only. Also note that if you have more than one ROS distribution installed, ~/.bashrc must only source the setup.bash for the version you are currently using.
  source ~/.bashrc 
  ```
</details>

<details>
<summary>Notes Before Usage</summary>

  + In your bash sessions, navigating to the ROSaic package can be achieved from anywhere with no more effort than `roscd septentrio_gnss_driver`. 
  + The driver assumes that our anonymous access to the Rx grants us full control rights. This should be the case by default, and can otherwise be changed with the `setDefaultAccessLevel` command. 
  + Currently, the driver only works on systems that are little-endian. Most modern computers, including PCs, are little-endian.
  + The development process of this driver has been performed with mosaic-x5, firmware (FW) revision number 2, and AsteRx-SBi3 Pro, FW revision number 1. If a more up-to-date FW (higher revision number) is uploaded to the mosaic, the driver will not be able to take account of new or updated SBF fields. 
  + ROSaic only works from C++11 onwards due to std::to_string() etc.
  + Septentrio's mosaic receivers and many others are only capable of establishing 10 streams !in total! of SBF blocks / NMEA messages. Please make sure that you do not set too many ROSaic parameters specifying the publishing of ROS messages to `true`. Note that in the GNSS case `gpsfix` accounts for 3 additional streams (`ChannelStatus`, `DOP` and `MeasEpoch` blocks), for instance.
  + Once the catkin build or binary installation is finished, adapt the `config/rover.yaml` file according to your needs. The `launch/rover.launch` need not be modified. Specify the communication parameters, the ROS messages to be published, the frequency at which the latter should happen etc.:<br>

  ```
  # Configuration Settings for the Rover Rx

  device: tcp://xxx.xxx.xxx.xxx:xxxx

  serial:
    baudrate: 115200
    rx_serial_port: USB1
    hw_flow_control: off

  frame_id: gnss
  
  receiver_type: gnss

  datum: ETRS89

  poi_to_arp:
    delta_e: 0.0
    delta_n: 0.0
    delta_u: 0.0
	
  poi_to_aux1_arp:
    delta_e: 0.0
    delta_n: 0.0
    delta_u: 0.0

  ant_type: Unknown
  ant_aux1_type: Unknown
  ant_serial_nr: Unknown
  ant_aux1_serial_nr: Unknown

  leap_seconds: 18

  polling_period:
    pvt: 500
    rest: 500

  use_gnss_time: false

  ntrip_settings:
    mode: off
    caster: 0
    caster_port: 0
    username: 0
    password: 0
    mountpoint: 0
    ntrip_version: v2
    send_gga: auto
    rx_has_internet: false
    rtcm_version: RTCMv2
    rx_input_corrections_tcp: 6666
    rx_input_corrections_serial: USB2

  publish:
    # For both GNSS and INS Rxs
	navsatfix: false
    gpsfix: true
    gpgga: false
    gprmc: false
    gpst : false
    pvtcartesian: false
    pvtgeodetic: true
    poscovcartesian: false
    poscovgeodetic: true
	velcovgeodetic: false
    atteuler: true
    attcoveuler: true
    pose: false
    diagnostics: false
    # For GNSS Rx only
    gpgsa: false
    gpgsv: false
    # For INS Rx only
    insnavcart: false
    insnavgeod: false
    extsensormeas: false
    imusetup: false
    velsensorsetup: false
    exteventinsnavcart: false
    exteventinsnavgeod: false

  # INS-Specific Parameters

  ins_spatial_config:
    att_offset:
      heading: 0.0
      pitch: 0.0
    imu_orientation:
      theta_x: 0.0
      theta_y: 0.0
      theta_z: 0.0
    poi_to_imu:
      delta_x: 0.0
      delta_y: 0.0
      delta_z: 0.0
    ant_lever_arm:
      x: 0.0
      y: 0.0
      z: 0.0
    vel_sensor_lever_arm:
      vsm_x: 0.0
      vsm_y: 0.0
      vsm_z: 0.0

  ins_initial_heading: auto

  ins_std_dev_mask:
    att_std_dev: 0.0
    pos_std_dev: 0.0

  ins_use_poi: false
  ```
  In order to launch ROSaic, one must specify all `arg` fields of the `rover.launch` file which have no associated default values, i.e. for now only the `param_file_name` field. Hence, the launch command reads `roslaunch septentrio_gnss_driver rover.launch param_file_name:=rover`.

</details>

# Inertial Navigation System (INS): Basics
  -  An Inertial Navigation System (INS) is a device which takes the rotation and acceleration solutions as obtained from its Inertial Measurement Unit (IMU) and combines those with position and velocity information from the GNSS module. Compared to a GNSS system with 7D or 8D (dual-antenna systems) phase space solutions, the combined, Kalman-filtered 9D phase space solution (3 for position, 3 for velocity, 3 for orientation) of an INS is more accurate, more precise and more stable against GNSS outages.
  - The IMU is typically made up of a 3-axis accelerometer, a 3-axis gyroscope and sometimes a 3-axis magnetometer and measures the system's angular rate and acceleration.

    <details>
    <summary>Measure and Compensate for IMU-Antenna Lever Arm</summary>
  
    + The IMU-antenna lever-arm is the relative position between the IMU reference point and the GNSS Antenna Reference Point (ARP), measured in the vehicle frame.
    + In case of AsteRx SBi3, the IMU reference point is clearly marked on the top panel of the receiver. It is important to compensate for the effect of the lever arm, otherwise the receiver may not be able to calculate an accurate INS position.
    + The IMU/antenna position can be changed by specifying the lever arm's `x`,`y`and `z` parameters in the `config.yaml` file under the `ins_spatial_config/ant_lever_arm` parameter.
  
      ![Screenshot from 2021-08-03 09-23-19 (1)](https://user-images.githubusercontent.com/62261460/127984869-f6892a30-e30d-4d41-bee3-ee1e4bfceab8.jpg)
  
    </details>

      <details>
    <summary>Compensate for IMU Orientation</summary>

    + It is important to take into consideration the mounting direction of the IMU in the body frame of the vehicle. For e.g. when the receiver is installed horizontally with the front panel facing the direction of travel, we must compensate for the IMU’s orientation to make sure the IMU reference frame is aligned with the vehicle reference frame.
    + The IMU's orientation can be changed by specifying the orientation angles `theta_x`,`theta_y`and `theta_z` in the `config.yaml` file under the `ins_spatial_config/imu_orientation`
    + The below image illustrates the orientation of the IMU reference frame with the associated IMU orientation for the depicted installation

    ![Capture (1)](https://user-images.githubusercontent.com/62261460/135855781-96459583-5268-4cf0-8995-f00cd0bd91e9.jpg)

    </details>
 
  - These Steps should be followed to configure the receiver in INS integration mode:
    - Specify `receiver_type: INS`
    - Specify the orientation of the IMU sensor with respect to your vehicle, using the `ins_spatial_config/imu_orientation` parameter
    - Specify the IMU-antenna lever arm in the vehicle reference frame. This is the vector starting from the IMU reference point to the ARP of the main GNSS antenna. This can be done by means of the `ins_spatial_config/ant_lever_arm` parameter.
    - If the point of interest is neither the IMU nor the ARP of the main GNSS antenna, the vector between the IMU and the point of interest can be provided with the `ins_solution/poi_to_imu` parameter.
    
  - For further more information about Septentrio receivers, visit Septentrio [support resources](https://www.septentrio.com/en/supportresources) or check out the [user manual](https://www.septentrio.com/system/files/support/asterx_sbi3_user_manual_v1.0_0.pdf) and [reference guide](https://www.septentrio.com/system/files/support/asterx_sbi3_pro_firmware_v1.3.0_reference_guide.pdf) of the AsteRx SBi3 receiver.

# ROSaic Parameters
The following is a list of ROSaic parameters found in the `config/rover.yaml` file.
* Parameters Configuring Communication Ports and Processing of GNSS and INS Data
  <details>
  <summary>Connectivity Specs</summary>

  + `device`: location of device connection
    + `serial:xxx` format for serial connections, where xxx is the device node, e.g. `serial:/dev/ttyUSB0`
    + `file_name:path/to/file.sbf` format for publishing from an SBF log
    + `file_name:path/to/file.pcap` format for publishing from PCAP capture.
      + Regarding the file path, ROS_HOME=\`pwd\` in front of `roslaunch septentrio...` might be useful to specify that the node should be started using the executable's directory as its working-directory.
    + `tcp://host:port` format for TCP/IP connections
      + `28784` should be used as the default (command) port for TCP/IP connections. If another port is specified, the receiver needs to be (re-)configured via the Web Interface before ROSaic can be used.
    + default: empty
  + `serial`: specifications for serial communication
    + `serial/baudrate`: serial baud rate to be used in a serial connection 
    + `serial/rx_serial_port`: determines to which (virtual) serial port of the Rx we want to get connected to, e.g. USB1 or COM1
    + `hw_flow_control`: specifies whether the serial (the Rx's COM ports, not USB1 or USB2) connection to the Rx should have UART HW flow control enabled or not
      + `off` to disable UART HW flow control, `RTS|CTS` to enable it
    + default: `115200`, `USB1`, `off`
  </details>
  
  <details>
  <summary>Receiver Type</summary>
  
  + `receiver_type`: This parameter is to select the type of the Septentrio receiver
    + If `gnss`, then ROS can only output data related to GNSS receivers.
    + If `ins`, then ROS can only output data related to INS receivers.
	+ default: `gnss`
  </details>
  
  <details>
  <summary>Frame ID</summary>
  
  + `frame_id`: name of the ROS tf frame for the Rx, placed in the header of all published messages
    + In ROS, the [tf package](https://wiki.ros.org/tf) lets you keep track of multiple coordinate frames over time. The frame ID will be resolved by [`tf_prefix`](http://wiki.ros.org/geometry/CoordinateFrameConventions) if defined. If a ROS message has a header (all of those we publish do), the frame ID can be found via `rostopic echo /topic`, where `/topic` is the topic into which the message is being published.
    + default: `gnss`
  </details>
  
  <details>
  <summary>Datum</summary>
  
  + `datum`: datum that (ellipsoidal) height should be referenced to in all published ROS messages
    + Since the standardized GGA message does only provide the orthometric height (= MSL height = distance from Earth's surface to geoid) and the geoid undulation (distance from geoid to ellipsoid) for which non-WGS84 datums cannot be specified, it does not affect the GGA message.
    + default: `ETRS89`
  </details>
  
  <details>
  <summary>POI-ARP Offset</summary>
  
  + `poi_to_arp`: offsets of the main GNSS antenna reference point (ARP) with respect to the point of interest (POI = marker)
    + The parameters `delta_e`, `delta_n` and `delta_u` are the offsets in the East, North and Up (ENU) directions respectively, expressed in meters.
    + All absolute positions reported by the receiver are POI positions, obtained by subtracting this offset from the ARP. The purpose is to take into account the fact that the antenna may not be located directly on the surveying POI.
    + default: `0.0`, `0.0` and `0.0`
  + `poi_to_aux1_arp`: same for Aux1 antenna
  </details>
  
  <details>
  <summary>Antenna Specs</summary>
  
  + `ant_type`: type of your main GNSS antenna
    + For best positional accuracy, it is recommended to select a type from the list returned by the command `lstAntennaInfo, Overview`. This is the list of antennas for which the receiver can compensate for phase center variation.
    + By default and if `ant_type` does not match any entry in the list returned by `lstAntennaInfo, Overview`, the receiver will assume that the phase center variation is zero at all elevations and frequency bands, and the position will not be as accurate.
    + default: `Unknown`
  + `ant_serial_nr`: serial number of your main GNSS antenna
  + `ant_aux1_type` and `ant_aux1_serial_nr`: same for Aux1 antenna
  </details>
  
  <details>
  <summary>Leap Seconds</summary>
  
  + `leap_seconds`: number of leap seconds that have been inserted up until the point of ROSaic usage
    + At the time of writing the code (2020), the GPS time, which is unaffected by leap seconds, was ahead of UTC time by 18 leap seconds. Adapt the leap_seconds parameter accordingly as soon as the next leap second is inserted into the UTC time or in case you are using ROSaic for the purpose of simulations. In the latter case, in addition please set the parameter `use_GNSS_time` to true and uncomment a paragraph in the `UTCtoUnix()` function definition found in the file `septentrio_gnss_driver/src/septentrio_gnss_driver/parsers/parsing_utilities.cpp` and enter the year, month and date to be simulated.
  </details>
  
  <details>
  <summary>Polling Periods</summary>
  
  + `polling_period/pvt`: desired period in milliseconds between the polling of two consecutive `PVTGeodetic`, `PosCovGeodetic`, `PVTCartesian` and `PosCovCartesian` blocks and - if published - between the publishing of two of the corresponding ROS messages (e.g. `septentrio_gnss_driver/PVTGeodetic.msg`) 
    + Clearly, the publishing of composite ROS messages such as [`sensor_msgs/NavSatFix.msg`](https://docs.ros.org/kinetic/api/sensor_msgs/html/msg/NavSatFix.html) or [`gps_common/GPSFix.msg`](https://docs.ros.org/hydro/api/gps_common/html/msg/GPSFix.html) is triggered by the SBF block that arrives last among the blocks of the current epoch.
    + default: `500` (2 Hz)
  + `polling_period/rest`: desired period in milliseconds between the polling of all other SBF blocks and NMEA sentences not addressed by the previous parameter, and - if published - between the publishing of all other ROS messages
    + default: `500` (2 Hz)
  </details>
  
  <details>
  <summary>Time Systems</summary>
  
  + `use_GNSS_time`:  `true` if the ROS message headers' unix epoch time field shall be constructed from the TOW (in the SBF case) and UTC (in the NMEA case) data, `false` if those times shall be constructed by the driver via the time(NULL) function found in the `ctime` library
    + default: `true`
  </details>
  
  <details>
  <summary>Correction Services</summary>
  
  + `ntrip_settings`: determines NTRIP connection parameters
    + The two implemented use cases are 
      + a) The Rx has internet access, set `rx_has_internet` to true, and 
      + b) The Rx has no internet access, set `rx_has_internet` to false, but `Data Link` from Septentrio's RxTools is installed on the computer.
    + The first nested ROS parameter, `ntrip_settings/mode`, specifies the type of the NTRIP connection and must be one of `Client`, `Client-Sapcorda` or `off`. In `Client` mode, the receiver receives data from the NTRIP caster. When selecting the `Client-Sapcorda` mode, the receiver receives data from the Sapcorda NTRIP service and no further settings are required, i.e. all other nested parameters are ignored. Note that the latter mode only works in Europe and North America. Set mode to `off` to disable all correction services.
    + Next, `ntrip_settings/caster` is the hostname or IP address of the NTRIP caster to connect to. To send data to the built-in NTRIP caster, use "localhost" for this parameter. 
    + Note that `ntrip_settings/port`, `ntrip_settings/username`, `ntrip_settings/password` and `ntrip_settings/mountpoint` are the IP port number, the user name, the password and the mount point, respectively, to be used when connecting to the NTRIP caster. The receiver encrypts the password so that it cannot be read back with the command "getNtripSettings". The `ntrip_settings/version` argument specifies which version of the NTRIP protocol to use (`v1` or `v2`).
    + Further, `send_gga` specifies whether or not to send NMEA GGA messages to the NTRIP caster, and at which rate. It must be one of `auto`, `off`, `sec1`, `sec5`, `sec10` or `sec60`. In `auto` mode, the receiver automatically sends GGA messages if requested by the caster. 
    + The boolean parameter `rx_has_internet` specifies whether the Rx has internet access or not. Note that an Ethernet cable is the only way to enable internet access on mosaic receivers (and most others) at the moment. In case internet is available, NTRIP will be configured with a simple command `snts, ...` that ROSaic sends to the receiver.
    + The parameter `rtcm_version` specifies the type of RTCM data transmitted to ROSaic by the NTRIP caster, either `RTCMv2` or `RTCMv3`. It depends on the mountpoint.
    + In case the connection to the receiver is via TCP, `rx_input_corrections_tcp` specifies the port number of the IP server (IPS1) connection that ROSaic establishes on the receiver. Note that ROSaic will send GGA messages on this connection, such that in the `Data Link` application of `RxTools` one just needs to set up a TCP client to the host name as found in the ROSaic parameter `device` with the port as found in `rx_input_corrections_tcp`. If the latter connection were connection 1 on `Data Link`, then connection 2 would set up an NTRIP client connecting to the NTRIP caster as specified in the above parameters in order to forward the corrections from connection 2 to connection 1.
    + Finally, in case we are facing a serial connection (COM or USB), the parameter `rx_input_corrections_serial` analogously determines the port on which corrections could be serially forwarded to the Rx via `Data Link`.
    + default: `off`, empty, empty, empty, empty, empty, `v2`, `auto`, `false`, `RTCMv2`, `6666`, `USB2`
  </details>
  
  <details>
  <summary>INS Specs</summary>

    + `ins_spatial_config`: Spatial configuration of INS/IMU
      + `att_offset`: Angular offset between two antenna (Main and Aux) and vehicle heading
        + `heading`: The perpendicular axis can be compensated for by adjusting the `heading` parameter
        + `pitch`: Vertical offset can be compensated for by adjusting the `pitch` parameter
        + default: `0.0`, `0.0` (degrees)
      + `imu_orientation`: IMU sensor orientation
        + Parameters `theta_x`, `theta_y` and `theta_z` are used to determine the sensor orientation with respect to the vehicle frame. Positive angles correspond to a right-handed (clockwise) rotation of the IMU with respect to its nominal orientation (see below). The order of the rotations is as follows: `theta_z` first, then `theta_y`, then `theta_x`.
        + The nominal orientation is where the IMU is upside up and with the `X axis` marked on the receiver pointing to the front of the vehicle.
        + default: `0.0`, `0.0`, `0.0` (degrees)
      + `poi_to_imu`: The lever arm from the IMU reference point to a user-defined POI
        + Parameters `delta_x`,`delta_y` and `delta_z` refer to the vehicle reference frame
        + default: `0.0`, `0.0`, `0.0` (meters)
      + `ant_lever_arm`: The lever arm from the IMU reference point to the main GNSS antenna
        + The parameters `x`,`y` and `z` refer to the vehicle reference frame
        + default: `0.0`, `0.0`, `0.0` (meters)
      + `vel_sensor_lever_arm`: The lever arm from the IMU reference point to the velocity sensor
        + The parameters `vsm_x`,`vsm_y` and `vsm_z` refer to the vehicle reference frame 
        + default: `0.0`, `0.0`, `0.0` (meters)
    + `ins_initial_heading`: How the receiver obtains the initial INS/GNSS integrated heading during the alignment phase
        + In case it is `auto`, the initial integrated heading is determined from GNSS measurements.
        + In case it is `stored`, the last known heading when the vehicle stopped before switching off the receiver is used as initial heading. Use if vehicle does not move when the receiver is switched off.
        + default: `auto`
    + `ins_std_dev_mask`: Maximum accepted error
      + `att_std_dev`: Configures an output limit on standard deviation of the attitude angles (max error accepted: 5 degrees)
      + `pos_std_dev`: Configures an output limit on standard deviation of the position (max error accepted: 100 meters)
      + default: `2` degrees, `100` meters
    + `ins_use_poi`: Whether or not to use the POI defined in `ins_spatial_config/poi_to_imu`
      + If true, the point at which the INS navigation solution (e.g. in `insnavgeod` ROS topic) is calculated will be the POI as defined above, otherwise it'll be the main GNSS antenna.
      + default: `false`
  </details>
  
* Parameters Configuring (Non-)Publishing of ROS Messages
  <details>
  <summary>NMEA/SBF Messages to be Published</summary>
  
    + `publish/gpgga`: `true` to publish `septentrio_gnss_driver/GPGGA.msg` messages into the topic `/gpgga`
    + `publish/gprmc`: `true` to publish `septentrio_gnss_driver/GPRMC.msg` messages into the topic `/gprmc`
    + `publish/gpgsa`: `true` to publish `septentrio_gnss_driver/GPGSA.msg` messages into the topic `/gpgsa`
    + `publish/gpgsv`: `true` to publish `septentrio_gnss_driver/GPGSV.msg` messages into the topic `/gpgsv`
    + `publish/pvtcartesian`: `true` to publish `septentrio_gnss_driver/PVTCartesian.msg` messages into the topic `/pvtcartesian`
    + `publish/pvtgeodetic`: `true` to publish `septentrio_gnss_driver/PVTGeodetic.msg` messages into the topic `/pvtgeodetic`
    + `publish/poscovcartesian`: `true` to publish `septentrio_gnss_driver/PosCovCartesian.msg` messages into the topic `/poscovcartesian`
    + `publish/poscovgeodetic`: `true` to publish `septentrio_gnss_driver/PosCovGeodetic.msg` messages into the topic `/poscovgeodetic`
    + `publish/velcovgeodetic`: `true` to publish `septentrio_gnss_driver/VelCovGeodetic.msg` messages into the topic `/velcovgeodetic`
    + `publish/atteuler`: `true` to publish `septentrio_gnss_driver/AttEuler.msg` messages into the topic `/atteuler`
    + `publish/attcoveuler`: `true` to publish `septentrio_gnss_driver/AttCovEuler.msg` messages into the topic `/attcoveuler`
    + `publish/gpst`: `true` to publish `sensor_msgs/TimeReference.msg` messages into the topic `/gpst`
    + `publish/navsatfix`: `true` to publish `sensor_msgs/NavSatFix.msg` messages into the topic `/navsatfix`
    + `publish/gpsfix`: `true` to publish `gps_common/GPSFix.msg` messages into the topic `/gpsfix`
    + `publish/pose`: `true` to publish `geometry_msgs/PoseWithCovarianceStamped.msg` messages into the topic `/pose`
    + `publish/diagnostics`: `true` to publish `diagnostic_msgs/DiagnosticArray.msg` messages into the topic `/diagnostics`
    + `publish/insnavcart`: `true` to publish `septentrio_gnss_driver/INSNavCart.msg` message into the topic`/insnavcart` 
    + `publish/insnavgeod`: `true` to publish `septentrio_gnss_driver/INSNavGeod.msg` message into the topic`/insnavgeod`  
    + `publish/extsensormeas`: `true` to publish `septentrio_gnss_driver/ExtSensorMeas.msg` message into the topic`/extsensormeas`
    + `publish/imusetup`: `true` to publish `septentrio_gnss_driver/IMUSetup.msg` message into the topic`/imusetup` 
    + `publish/velsensorsetup`: `true` to publish `septentrio_gnss_driver/VelSensorSetup.msgs` message into the topic`/velsensorsetup` 
    + `publish/exteventinsnavcart`: `true` to publish `septentrio_gnss_driver/ExtEventINSNavCart.msgs` message into the topic`/exteventinsnavcart` 
    + `publish/exteventinsnavgeod`: `true` to publish `septentrio_gnss_driver/ExtEventINSNavGeod.msgs` message into the topic`/exteventinsnavgeod`
  </details>

## ROS Topic Publications
A selection of NMEA sentences, the majority being standardized sentences, and proprietary SBF blocks is translated into ROS messages, partly generic and partly custom, and can be published at the discretion of the user into the following ROS topics. All published ROS messages, even custom ones, start with a ROS generic header [`std_msgs/Header.msg`](https://docs.ros.org/melodic/api/std_msgs/html/msg/Header.html), which includes the receiver time stamp as well as the frame ID, the latter being specified in the ROS parameter `frame_id`.
<details>
  <summary>Available ROS Topics</summary>
  
  + `/gpgga`: publishes custom ROS message `septentrio_gnss_driver/Gpgga.msg` - equivalent to [`nmea_msgs/Gpgga.msg`](https://docs.ros.org/api/nmea_msgs/html/msg/Gpgga.html) - converted from the NMEA sentence GGA
  + `/gprmc`: publishes custom ROS message `septentrio_gnss_driver/Gprmc.msg` - equivalent to [`nmea_msgs/Gprmc.msg`](https://docs.ros.org/api/nmea_msgs/html/msg/Gprmc.html) - converted from the NMEA sentence RMC
  + `/gpgsa`: publishes custom ROS message `septentrio_gnss_driver/Gpgsa.msg` - equivalent to [`nmea_msgs/Gpgsa.msg`](https://docs.ros.org/api/nmea_msgs/html/msg/Gpgsa.html) - converted from the NMEA sentence GSA
  + `/gpgsv`: publishes custom ROS message `septentrio_gnss_driver/Gpgsv.msg` - equivalent to [`nmea_msgs/Gpgsv.msg`](https://docs.ros.org/api/nmea_msgs/html/msg/Gpgsv.html) - converted from the NMEA sentence GSV
  + `/pvtcartesian`: publishes custom ROS message `septentrio_gnss_driver/PVTCartesian.msg`, corresponding to the SBF block `PVTCartesian` (GNSS case) or `INSNavGeod` (INS case)
  + `/pvtgeodetic`: publishes custom ROS message `septentrio_gnss_driver/PVTGeodetic.msg`, corresponding to the SBF block `PVTGeodetic` (GNSS case) or `INSNavGeod` (INS case)
  + `/poscovcartesian`: publishes custom ROS message `septentrio_gnss_driver/PosCovCartesian.msg`, corresponding to SBF block `PosCovCartesian` (GNSS case) or `INSNavGeod` (INS case)
  + `/poscovgeodetic`: publishes custom ROS message `septentrio_gnss_driver/PosCovGeodetic.msg`, corresponding to SBF block `PosCovGeodetic` (GNSS case) or `INSNavGeod` (INS case)
  + `/velcovgeodetic`: publishes custom ROS message `septentrio_gnss_driver/VelCovGeodetic.msg`, corresponding to SBF block `VelCovGeodetic` (GNSS case)
  + `/atteuler`: publishes custom ROS message `septentrio_gnss_driver/AttEuler.msg`, corresponding to SBF block `AttEuler` (GNSS case) or `INSNavGeod` (INS case)
  + `/attcoveuler`: publishes custom ROS message `septentrio_gnss_driver/AttCovEuler.msg`, corresponding to the SBF block `AttCovEuler` (GNSS case) or `INSNavGeod` (INS case)
  + `/gpst` (for GPS Time): publishes generic ROS message [`sensor_msgs/TimeReference.msg`](https://docs.ros.org/melodic/api/sensor_msgs/html/msg/TimeReference.html), converted from the `PVTGeodetic` (GNSS case) or `INSNavGeod` (INS case) block's GPS time information, stored in its header, or - if `use_gnss_time` is set to `false` - from the systems's wall-clock time
  + `/navsatfix`: publishes generic ROS message [`sensor_msgs/NavSatFix.msg`](https://docs.ros.org/kinetic/api/sensor_msgs/html/msg/NavSatFix.html), converted from the SBF blocks `PVTGeodetic`,`PosCovGeodetic` (GNSS case) or `INSNavGeod` (INS case)
    + The ROS message [`sensor_msgs/NavSatFix.msg`](https://docs.ros.org/kinetic/api/sensor_msgs/html/msg/NavSatFix.html) can be fed directly into the [`navsat_transform_node`](https://docs.ros.org/melodic/api/robot_localization/html/navsat_transform_node.html) of the ROS navigation stack.
  + `/gpsfix`: publishes generic ROS message [`gps_common/GPSFix.msg`](https://docs.ros.org/hydro/api/gps_common/html/msg/GPSFix.html), which is much more detailed than [`sensor_msgs/NavSatFix.msg`](https://docs.ros.org/kinetic/api/sensor_msgs/html/msg/NavSatFix.html), converted from the SBF blocks `PVTGeodetic`, `PosCovGeodetic`, `ChannelStatus`, `MeasEpoch`, `AttEuler`, `AttCovEuler`, `VelCovGeodetic`, `DOP` (GNSS case) or `INSNavGeod`, `DOP` (INS case)
  + `/pose`: publishes generic ROS message [`geometry_msgs/PoseWithCovarianceStamped.msg`](https://docs.ros.org/melodic/api/geometry_msgs/html/msg/PoseWithCovarianceStamped.html), converted from the SBF blocks `PVTGeodetic`, `PosCovGeodetic`, `AttEuler`, `AttCovEuler` (GNSS case) or `INSNavGeod` (INS case)
    + Note that GNSS provides absolute positioning, while robots are often localized within a local level frame. The pose field of this ROS message contains position with respect to the absolute ENU frame (longitude, latitude, height), while the orientation is with respect to a vehicle-fixed (e.g. for mosaic-x5 in moving base mode via the command `setAntennaLocation`, ...) !local! NED frame. Thus the orientation is !not! given with respect to the same frame as the position is given in. The cross-covariances are hence set to 0.
    + In ROS, all state estimation nodes in the [`robot_localization` package](https://docs.ros.org/melodic/api/robot_localization/html/index.html) can accept the ROS message `geometry_msgs/PoseWithCovarianceStamped.msg`.
  + `/insnavcart`: publishes custom ROS message `septentrio_gnss_driver/INSNavCart.msg`, corresponding to SBF block `INSNavCart` 
  + `/insnavgeod`: publishes custom ROS message `septentrio_gnss_driver/INSNavGeod.msg`, corresponding to SBF block `INSNavGeod` 
  + `/extsensormeas`: publishes custom ROS message `septentrio_gnss_driver/ExtSensorMeas.msg`, corresponding to SBF block `ExtSensorMeas` 
  + `/imusetup`: publishes custom ROS message `septentrio_gnss_driver/IMUSetup.msg`, corresponding to SBF block `IMUSetup` 
  + `/velsensorsetup`: publishes custom ROS message `septentrio_gnss_driver/VelSensorSetup.msg` corresponding to SBF block `VelSensorSetup` 
  + `/exteventinsnavcart`: publishes custom ROS message `septentrio_gnss_driver/ExtEventINSNavCart.msg`, corresponding to SBF block `ExtEventINSNavCart` 
  + `/exteventinsnavgeod`: publishes custom ROS message `septentrio_gnss_driver/ExtEventINSNavGeod.msg`, corresponding to SBF block `ExtEventINSNavGeod` 
  + `/diagnostics`: accepts generic ROS message [`diagnostic_msgs/DiagnosticArray.msg`](https://docs.ros.org/api/diagnostic_msgs/html/msg/DiagnosticArray.html), converted from the SBF blocks `QualityInd`, `ReceiverStatus` and `ReceiverSetup`
</details>

## Suggestions for Improvements
<details>
  <summary>Some Ideas</summary>

  + Automatic Search: If the host address of the receiver is omitted in the `host:port` specification, the driver could automatically search and establish a connection on the specified port.
  + Publishing the topic `/measepoch`: It could accept the custom ROS message `septentrio_gnss_driver/MeasEpoch.msg`, corresponding to the SBF block `MeasEpoch` (raw GNSS data).
  + Publishing the topic `/twist`: It could accept the generic ROS message [`geometry_msgs/TwistWithCovarianceStamped.msg`](https://docs.ros.org/melodic/api/geometry_msgs/html/msg/TwistWithCovarianceStamped.html), converted from the SBF blocks `PVTGeodetic`, `PosCovGeodetic` and others or via standardized NMEA sentences (cf. the [NMEA driver](https://wiki.ros.org/nmea_navsat_driver)).
    + The ROS message [`geometry_msgs/TwistWithCovarianceStamped.msg`](https://docs.ros.org/melodic/api/geometry_msgs/html/msg/TwistWithCovarianceStamped.html) could be fed directly into the [`robot_localization`](https://docs.ros.org/melodic/api/robot_localization/html/index.html) nodes of the ROS navigation stack.
  + Additional ROSaic parameter: endianness of the system.
  + Equip ROSaic with an NTRIP client such that it can forward corrections to the receiver independently of `Data Link`.
</details>

## Adding New SBF Blocks or NMEA Sentences
<details>
  <summary>Steps to Follow</summary>

  Is there an SBF or NMEA message that is not being addressed while being important to your application? If yes, follow these steps:
  1. Find the log reference of interest in the publicly accessible, official documentation. Hence select the reference guide file, e.g. for mosaic-x5 in the [product support section for mosaic-X5](https://www.septentrio.com/en/support/mosaic/mosaic-x5), Chapter 4, of Septentrio's homepage.
  2. Add a new `.msg` file to the `septentrio_gnss_driver/msg` folder.
  3. SBF: Add the new struct definition to the `sbf_structs.hpp` file.
  4. Parsing/Processing the message/block:
      - Both: Add a new include guard to let the compiler know about the existence of the header file (such as `septentrio_gnss_driver/PVTGeodetic.h`) that gets compiler-generated from the `.msg` file constructed in step 3.
      - SBF: Extend the `NMEA_ID_Enum` enumeration in the `rx_message.hpp` file with a new entry.
      - SBF: Extend the initialization of the `RxIDMap` map in the `rx_message.cpp` file with a new pair.
      - SBF: Add a new callback function declaration, a new method, to the `io_comm_rx::RxMessage class` in the `rx_message.hpp` file.
      - SBF: Add the latter's definition to the `rx_message.cpp` file.
      - SBF: Add a new C++ "case" (part of the C++ switch-case structure) in the `rx_message.hpp` file. It should be modeled on the existing `evPVTGeodetic` case, e.g. one needs a static counter variable declaration.
      - NMEA: Construct two new parsing files such as `gpgga.cpp` to the `septentrio_gnss_driver/src/septentrio_gnss_driver/parsers/nmea_parsers` folder and one such as `gpgga.hpp` to the `septentrio_gnss_driver/include/septentrio_gnss_driver/parsers/nmea_parsers` folder.
  5. Create a new `publish/..` ROSaic parameter in the `septentrio_gnss_driver/config/rover.yaml` file, create a global boolean variable `publish_...` in the `septentrio_gnss_driver/src/septentrio_gnss_driver/node/rosaic_node.cpp` file, insert the publishing callback function to the C++ "multimap" `IO.handlers_.callbackmap_` - which is already storing all the others - in the `rosaic_node::ROSaicNode::defineMessages()` method in the same file and add an `extern bool publish_...;` line to the `septentrio_gnss_driver/include/septentrio_gnss_driver/node/rosaic_node.hpp` file.
  6. Modify the `septentrio_gnss_driver/CMakeLists.txt` file by adding a new entry to the `add_message_files` section.
</details>
