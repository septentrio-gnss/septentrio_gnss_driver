# ROSaic Parameters
The following is a list of ROSaic parameters found in the `config/rover.yaml` file.
* Parameters Configuring Communication Ports and Processing of GNSS and INS Data
  - `device`: location of device connection
    - `serial:xxx` format for serial connections, where xxx is the device node, e.g. `serial:/dev/ttyUSB0`
    - `file_name:path/to/file.sbf` format for publishing from an SBF log
    - `file_name:path/to/file.pcap` format for publishing from PCAP capture.
      - Regarding the file path, ROS_HOME=\`pwd\` in front of `roslaunch septentrio...` might be useful to specify that the node should be started using the executable's directory as its working-directory.
    - `tcp://host:port` format for TCP/IP connections
      - `28784` should be used as the default (command) port for TCP/IP connections. If another port is specified, the receiver needs to be (re-)configured via the Web Interface before ROSaic can be used.
    - default: empty
  - `serial`: specifications for serial communication
    - `serial/baudrate`: serial baud rate to be used in a serial connection 
    - `serial/rx_serial_port`: determines to which (virtual) serial port of the Rx we want to get connected to, e.g. USB1 or COM1
    - `hw_flow_control`: specifies whether the serial (the Rx's COM ports, not USB1 or USB2) connection to the Rx should have UART HW flow control enabled or not
      - `off` to disable UART HW flow control, `RTS|CTS` to enable it
    - default: `115200`, `USB1`, `off`
  - `frame_id`: name of the ROS tf frame for the Rx, placed in the header of all published messages
    - In ROS, the [tf package](https://wiki.ros.org/tf) lets you keep track of multiple coordinate frames over time. The frame ID will be resolved by [`tf_prefix`](http://wiki.ros.org/geometry/CoordinateFrameConventions) if defined. If a ROS message has a header (all of those we publish do), the frame ID can be found via `rostopic echo /topic`, where `/topic` is the topic into which the message is being published.
    - default: `gnss`
  - `datum`: datum that (ellipsoidal) height should be referenced to in all published ROS messages
    - Since the standardized GGA message does only provide the orthometric height (= MSL height = distance from Earth's surface to geoid) and the geoid undulation (distance from geoid to ellipsoid) for which non-WGS84 datums cannot be specified, it does not affect the GGA message.
    - default: `ETRS89`
  - `marker_to_arp`: offsets of the antenna reference point (ARP) with respect to the marker
    - The parameters `delta_e`, `delta_n` and `delta_u` are the offsets in the East, North and Up (ENU) directions respectively, expressed in meters.
    - All absolute positions reported by the receiver are marker positions, obtained by subtracting this offset from the ARP. The purpose is to take into account the fact that the antenna may not be located directly on the surveying point of interest.
    - default: `0.0`, `0.0` and `0.0`
  - `ant_type`: type of your antenna
    - For best positional accuracy, it is recommended to select a type from the list returned by the command `lstAntennaInfo, Overview`. This is the list of antennas for which the receiver can compensate for phase center variation.
    - By default and if `ant_type` does not match any entry in the list returned by `lstAntennaInfo, Overview`, the receiver will assume that the phase center variation is zero at all elevations and frequency bands, and the position will not be as accurate.
    - default: `Unknown`
  - `ant_serial_nr`: serial number of your antenna
 
  **------ To be Implemented------**
  - `imu_orientation`: IMU sensor orientation
    - If orientation is set to `sensor_default`, the receiver assumes that the IMU is attached to the vehicle in the nominal orientation, i.e. horizontally, upside up and with the `X axis` marked on the receiver pointing to the front of the vehicle.
    - If orientation is set to `manual`, the receiver will use parameters `ThetaX`, `ThetaY` and `ThetaZ` to determine the sensor orientation with respect to the vehicle frame. Positive angles correspond to a right-handed (clockwise) rotation of the IMU with respect to its nominal orientation. The order of the rotations is as follows: `ThetaZ` first, then `ThetaY`, then `ThetaX`.
  - `ins_ant_lever_arm`: The lever arm from the IMU reference point to the main GNSS antenna
    - The parameters `X`,`Y` and `Z` refers to the vehicle reference  frame
    - **Note**: For an accurate navigation it is essential to provide an accurate `ins_ant_lever_arm`
  - `set_insinitial_heading`: Speed up the INS heading calibration
    - `auto`: This mode will store the heading determined from GNSS measurements.
    - `stored`: This mode is used to store the last heading alignment when the vehicle stopped before switching of the receiver.  
  - `ins_nav_config`: The INS navigation filter
    - The `output_type` parameter enables or disables the computation of INS attitude or velocity and the associated standard deviations . If set to `none`, only the INS position is computed and output in the INSNavGeod and INSNavCart SBF blocks.
    - The INS solution can either refer to the main GNSS antenna ARP or to a user-defined point of interest (POI) on the vehicle, as set with the `output_location` parameter. The lever arms from the IMU to the main antenna and to the POI must be specified with the `ins_ant_lever_arm` and `ins_poi_lever_arm` parameter
  - `ins_poi_lever_arm`: The lever arm from the IMU reference point to a user-defined point of interest in the vehicle
    - The parameters `X`,`Y` and `Z` refers to the vehicle reference  frame
    - The reference point of the navigation output in the insnavcart and insnavgeod of ROS /topic is either the main GNSS antenna, or POI. By default, POI is colocated with the IMU reference point (lever arm is zero by default)

     **------ Till here------**
  - `leap_seconds`: number of leap seconds that have been inserted up until the point of ROSaic usage
    - At the time of writing the code (2020), the GPS time, which is unaffected by leap seconds, was ahead of UTC time by 18 leap seconds. Adapt the leap_seconds parameter accordingly as soon as the next leap second is inserted into the UTC time or in case you are using ROSaic for the purpose of simulations. In the latter case, in addition please set the parameter `use_GNSS_time` to true and uncomment a paragraph in the `UTCtoUnix()` function definition found in the file `septentrio_gnss_driver/src/septentrio_gnss_driver/parsers/parsing_utilities.cpp` and enter the year, month and date to be simulated.
  - `polling_period/pvt`: desired period in milliseconds between the polling of two consecutive `PVTGeodetic`, `PosCovGeodetic`, `PVTCartesian` and `PosCovCartesian` blocks and - if published - between the publishing of two of the corresponding ROS messages (e.g. `septentrio_gnss_driver/PVTGeodetic.msg`) 
    - Clearly, the publishing of composite ROS messages such as [`sensor_msgs/NavSatFix.msg`](https://docs.ros.org/kinetic/api/sensor_msgs/html/msg/NavSatFix.html) or [`gps_common/GPSFix.msg`](https://docs.ros.org/hydro/api/gps_common/html/msg/GPSFix.html) is triggered by the SBF block that arrives last among the blocks of the current epoch.
    - default: `500` (2 Hz)
  - `polling_period/rest`: desired period in milliseconds between the polling of all other SBF blocks and NMEA sentences not addressed by the previous parameter, and - if published - between the publishing of all other ROS messages
    - default: `500` (2 Hz)
  - `reconnect_delay_s`: delay in seconds between reconnection attempts to the connection specified in the parameter `device`
    - default: `2`
  - `use_GNSS_time`:  `true` if the ROS message headers' unix epoch time field shall be constructed from the TOW (in the SBF case) and UTC (in the NMEA case) data, `false` if those times shall be constructed by the driver via the time(NULL) function found in the `ctime` library
    - default: `true`
  - `ntrip_settings`: determines NTRIP connection parameters
    - The two implemented use cases are 
      - a) The Rx has internet access, set `rx_has_internet` to true, and 
      - b) The Rx has no internet access, set `rx_has_internet` to false, but `Data Link` from Septentrio's RxTools is installed on the computer.
    - The first nested ROS parameter, `ntrip_settings/mode`, specifies the type of the NTRIP connection and must be one of `Client`, `Client-Sapcorda` or `off`. In `Client` mode, the receiver receives data from the NTRIP caster. When selecting the `Client-Sapcorda` mode, the receiver receives data from the Sapcorda NTRIP service and no further settings are required, i.e. all other nested parameters are ignored. Note that the latter mode only works in Europe and North America. Set mode to `off` to disable all correction services.
    - Next, `ntrip_settings/caster` is the hostname or IP address of the NTRIP caster to connect to. To send data to the built-in NTRIP caster, use "localhost" for this parameter. 
    - Note that `ntrip_settings/port`, `ntrip_settings/username`, `ntrip_settings/password` and `ntrip_settings/mountpoint` are the IP port number, the user name, the password and the mount point, respectively, to be used when connecting to the NTRIP caster. The receiver encrypts the password so that it cannot be read back with the command "getNtripSettings". The `ntrip_settings/version` argument specifies which version of the NTRIP protocol to use (`v1` or `v2`).
    - Further, `send_gga` specifies whether or not to send NMEA GGA messages to the NTRIP caster, and at which rate. It must be one of `auto`, `off`, `sec1`, `sec5`, `sec10` or `sec60`. In `auto` mode, the receiver automatically sends GGA messages if requested by the caster. 
    - The boolean parameter `rx_has_internet` specifies whether the Rx has internet access or not. Note that an Ethernet cable is the only way to enable internet access on mosaic receivers (and most others) at the moment. In case internet is available, NTRIP will be configured with a simple command `snts, ...` that ROSaic sends to the receiver.
    - The parameter `rtcm_version` specifies the type of RTCM data transmitted to ROSaic by the NTRIP caster, either `RTCMv2` or `RTCMv3`. It depends on the mountpoint.
    - In case the connection to the receiver is via TCP, `rx_input_corrections_tcp` specifies the port number of the IP server (IPS1) connection that ROSaic establishes on the receiver. Note that ROSaic will send GGA messages on this connection, such that in the `Data Link` application of `RxTools` one just needs to set up a TCP client to the host name as found in the ROSaic parameter `device` with the port as found in `rx_input_corrections_tcp`. If the latter connection were connection 1 on `Data Link`, then connection 2 would set up an NTRIP client connecting to the NTRIP caster as specified in the above parameters in order to forward the corrections from connection 2 to connection 1.
    - Finally, in case we are facing a serial connection (COM or USB), the parameter `rx_input_corrections_serial` analogously determines the port on which corrections could be serially forwarded to the Rx via `Data Link`.
    - default: `off`, empty, empty, empty, empty, empty, `v2`, `auto`, `false`, `RTCMv2`, `6666`, `USB2`
  
- Parameters Configuring (Non-)Publishing of ROS Messages 
  - `publish/gpgga`: `true` to publish `septentrio_gnss_driver/GPGGA.msg` messages into the topic `/gpgga`
  - `publish/gprmc`: `true` to publish `septentrio_gnss_driver/GPRMC.msg` messages into the topic `/gprmc`
  - `publish/gpgsa`: `true` to publish `septentrio_gnss_driver/GPGSA.msg` messages into the topic `/gpgsa`
  - `publish/gpgsv`: `true` to publish `septentrio_gnss_driver/GPGSV.msg` messages into the topic `/gpgsv`
  - `publish/pvtcartesian`: `true` to publish `septentrio_gnss_driver/PVTCartesian.msg` messages into the topic `/pvtcartesian`
  - `publish/pvtgeodetic`: `true` to publish `septentrio_gnss_driver/PVTGeodetic.msg` messages into the topic `/pvtgeodetic`
  - `publish/poscovcartesian`: `true` to publish `septentrio_gnss_driver/PosCovCartesian.msg` messages into the topic `/poscovcartesian`
  - `publish/poscovgeodetic`: `true` to publish `septentrio_gnss_driver/PosCovGeodetic.msg` messages into the topic `/poscovgeodetic`
  - `publish/atteuler`: `true` to publish `septentrio_gnss_driver/AttEuler.msg` messages into the topic `/atteuler`
  - `publish/attcoveuler`: `true` to publish `septentrio_gnss_driver/AttCovEuler.msg` messages into the topic `/attcoveuler`
  - `publish/gpst`: `true` to publish `sensor_msgs/TimeReference.msg` messages into the topic `/gpst`
  - `publish/navsatfix`: `true` to publish `sensor_msgs/NavSatFix.msg` messages into the topic `/navsatfix`
  - `publish/gpsfix`: `true` to publish `gps_common/GPSFix.msg` messages into the topic `/gpsfix`
  - `publish/pose`: `true` to publish `geometry_msgs/PoseWithCovarianceStamped.msg` messages into the topic `/pose`
  - `publish/diagnostics`: `true` to publish `diagnostic_msgs/DiagnosticArray.msg` messages into the topic `/diagnostics`
        **------ To be Implemented------**
  - `publish/insnavcart`: `true` to publish `insnavcart_msgs/INSNavCart.msg` message into the topic`/insnavcart` 
  - `publish/insnavgeod`: `true` to publish `insnavgeod_msgs/INSNavGeod.msg` message into the topic`/insnavgeod` 
  - `publish/exteventinsnavcart`: `true` to publish `exteventinsnavcart_msgs/ExtEventINSNavCart.msgs` message into the topic`/exteventinsnavcart` 
  - `publish/exteventinsnavgeod`: `true` to publish `exteventinsnavgeod_msgs/ExtEventINSNavGeod.msgs` message into the topic`/exteventinsnavgeod` 
  - `publish/imusetup`: `true` to publish `imusetup_msgs/IMUSetup.msg` message into the topic`/imusetup` 
  - `publish/velsensorsetup`: `true` to publish `velsensorsetup_msg/VelSensorSetup.msgs` message into the topic`/velsensorsetup` 
  
       **------ Till here------**

## ROS Topic Publications
A selection of NMEA sentences, the majority being standardized sentences, and proprietary SBF blocks is translated into ROS messages, partly generic and partly custom, and can be published at the discretion of the user into the following ROS topics. All published ROS messages, even custom ones, start with a ROS generic header [`std_msgs/Header.msg`](https://docs.ros.org/melodic/api/std_msgs/html/msg/Header.html), which includes the receiver time stamp as well as the frame ID, the latter being specified in the ROS parameter `frame_id`.
- `/gpgga`: publishes custom ROS message `septentrio_gnss_driver/Gpgga.msg` - equivalent to [`nmea_msgs/Gpgga.msg`](https://docs.ros.org/api/nmea_msgs/html/msg/Gpgga.html) - converted from the NMEA sentence GGA
- `/gprmc`: publishes custom ROS message `septentrio_gnss_driver/Gprmc.msg` - equivalent to [`nmea_msgs/Gprmc.msg`](https://docs.ros.org/api/nmea_msgs/html/msg/Gprmc.html) - converted from the NMEA sentence RMC
- `/gpgsa`: publishes custom ROS message `septentrio_gnss_driver/Gpgsa.msg` - equivalent to [`nmea_msgs/Gpgsa.msg`](https://docs.ros.org/api/nmea_msgs/html/msg/Gpgsa.html) - converted from the NMEA sentence GSA
- `/gpgsv`: publishes custom ROS message `septentrio_gnss_driver/Gpgsv.msg` - equivalent to [`nmea_msgs/Gpgsv.msg`](https://docs.ros.org/api/nmea_msgs/html/msg/Gpgsv.html) - converted from the NMEA sentence GSV
- `/pvtcartesian`: publishes custom ROS message `septentrio_gnss_driver/PVTCartesian.msg`, corresponding to the SBF block `PVTCartesian`
- `/pvtgeodetic`: publishes custom ROS message `septentrio_gnss_driver/PVTGeodetic.msg`, corresponding to the SBF block `PVTGeodetic`
- `/poscovcartesian`: publishes custom ROS message `septentrio_gnss_driver/PosCovCartesian.msg`, corresponding to SBF block `PosCovCartesian`
- `/poscovgeodetic`: publishes custom ROS message `septentrio_gnss_driver/PosCovGeodetic.msg`, corresponding to SBF block `PosCovGeodetic`
- `/atteuler`: publishes custom ROS message `septentrio_gnss_driver/AttEuler.msg`, corresponding to SBF block `AttEuler`
- `/attcoveuler`: publishes custom ROS message `septentrio_gnss_driver/AttCovEuler.msg`, corresponding to the SBF block `AttCovEuler`
- `/gpst` (for GPS Time): publishes generic ROS message [`sensor_msgs/TimeReference.msg`](https://docs.ros.org/melodic/api/sensor_msgs/html/msg/TimeReference.html), converted from the `PVTGeodetic` block's GPS time information, stored in its header, or - if `use_gnss_time` is set to `false` - from the systems's wall-clock time
- `/navsatfix`: publishes generic ROS message [`sensor_msgs/NavSatFix.msg`](https://docs.ros.org/kinetic/api/sensor_msgs/html/msg/NavSatFix.html), converted from the SBF blocks `PVTGeodetic`,`PosCovGeodetic` and `INSNavGeod`
  - The ROS message [`sensor_msgs/NavSatFix.msg`](https://docs.ros.org/kinetic/api/sensor_msgs/html/msg/NavSatFix.html) can be fed directly into the [`navsat_transform_node`](https://docs.ros.org/melodic/api/robot_localization/html/navsat_transform_node.html) of the ROS navigation stack.
- `/gpsfix`: publishes generic ROS message [`gps_common/GPSFix.msg`](https://docs.ros.org/hydro/api/gps_common/html/msg/GPSFix.html), which is much more detailed than [`sensor_msgs/NavSatFix.msg`](https://docs.ros.org/kinetic/api/sensor_msgs/html/msg/NavSatFix.html), converted from the SBF blocks `PVTGeodetic`, `PosCovGeodetic`, `ChannelStatus`, `MeasEpoch`, `AttEuler`, `AttCovEuler`, `VelCovGeodetic`, `INSNavGeod` and `DOP`
- `/pose`: publishes generic ROS message [`geometry_msgs/PoseWithCovarianceStamped.msg`](https://docs.ros.org/melodic/api/geometry_msgs/html/msg/PoseWithCovarianceStamped.html), converted from the SBF blocks `PVTGeodetic`, `PosCovGeodetic`, `AttEuler`, `AttCovEuler` and `INSNavGeod`
- `/insnavcart`: publish custom ROS message `septentrio_gnss_driver/INSNavCart.msg`, corresponding to SBF block `INSNavCart` 
- `/insnavgeod`: publish custom ROS message `septentrio_gnss_driver/INSNavGeod.msg`, corresponding to SBF block `INSNavGeod` 
- `/exteventinsnavcart`: publish custom ROS message `septentrio_gnss_driver/ExtEventINSNavCart.msg`, corresponding to SBF block `ExtEventINSNavCart` 
- `/exteventinsnavgeod`: publish custom ROS message `septentrio_gnss_driver/ExtEventINSNavGeod.msg`, corresponding to SBF block `ExtEventINSNavGeod` 
- `/imusetup`: publish custom ROS message `septentrio_gnss_driver/IMUSetup.msg`, corresponding to SBF block `IMUSetup` 
- `/velsensorsetup`: publish custom ROS message `septentrio_gnss_driver/VelSensorSetup.msg` corresponding to SBF block `VelSensorSetup` 
  - Note that GNSS provides absolute positioning, while robots are often localized within a local level frame. The pose field of this ROS message contains position with respect to the absolute ENU frame (longitude, latitude, height), while the orientation is with respect to a vehicle-fixed (e.g. for mosaic-x5 in moving base mode via the command `setAntennaLocation`, ...) !local! NED frame. Thus the orientation is !not! given with respect to the same frame as the position is given in. The cross-covariances are hence set to 0.
  - In ROS, all state estimation nodes in the [`robot_localization` package](https://docs.ros.org/melodic/api/robot_localization/html/index.html) can accept the ROS message `geometry_msgs/PoseWithCovarianceStamped.msg`.
- `/diagnostics`: accepts generic ROS message [`diagnostic_msgs/DiagnosticArray.msg`](https://docs.ros.org/api/diagnostic_msgs/html/msg/DiagnosticArray.html), converted from the SBF blocks `QualityInd`, `ReceiverStatus` and `ReceiverSetup`
