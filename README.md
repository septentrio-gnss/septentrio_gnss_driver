# ROSaic

## Overview
This repository aims at keeping track of the 
driver development process. Our goal is to build a ROS Melodic and Noetic driver (i.e. for Linux only) - written in C++ (stay tuned for Python version) - that is compatible with Septentrio's mosaic-X5 GNSS receiver. It will also be extended to ROS2, modifications mainly addressing launch file configurations. 

Main Features:
- Supports serial, TCP/IP and USB connections, the latter being compatible with both the serial and the TCP/IP protocols
- Facilitates extension to further log types, see instructions below
- Supports (as of now) a handful of ASCII (including key NMEA ones) messages and SBF (Septentrio Binary Format) blocks
- Tested with the mosaic-X5 receiver

Please [let the maintainers know](mailto:tibor.dome@septentrio.com?subject=[GitHub]%20ROSaic) of your success or failure in using the driver with other devices so we can update this page appropriately.

## Dependencies
The `master` branch for this driver functions on both ROS Melodic and Noetic. It is thus necessary to [install](https://wiki.ros.org/Installation/Ubuntu) the ROS version that is compatible with your Linux distro.
We refrained from redefining custom ROS messages that correspond to NMEA messages since those can be readily obtained via<br><br>
`sudo apt-get install ros-${ROS_DISTRO}-nmea-msgs`.<br><br>
The serial and TCP/IP communication interface of the ROS driver is established by means of the [Boost C++ library](https://www.boost.org/). Please install the Boost libraries via<br><br>
`sudo apt install libboost-all-dev`.<br><br>
Source and header files of the driver have been used as input for [Doxygen](https://www.doxygen.nl/index.html), a lexical scanner for generating documentation from annotated C++ files. The generated on-line HTML documention can be viewed by pointing an HTML browser to the `index.html` file located in `doxygen_out/html`. For best results, a browser that supports cascading style sheets (CSS) should be used, e.g. Mozilla Firefox or Google Chrome. If the driver is extended, e.g. a new SBF block added as detailed below, annotations would ideally be adapted and the documentation regenerated via the shell command `doxygen Doxyfile`, where the configuration file `Doxyfile` need not necessarily be changed. For this to work, Doxygen must be installed, either via<br><br>
`sudo apt-get install -y doxygen`<br><br>
or [from source](https://www.doxygen.nl/manual/install.html).

## Usage
 To install the binary packages, run the following command in a terminal (until now only tested on ROS Melodic):<br><br>
`sudo apt-get install ros-${ROS_DISTRO}-septentrio-gnss-driver`.<br><br>
Alternatively, the package can also be built from source using [`catkin_tools`](https://catkin-tools.readthedocs.io/en/latest/installing.html), where the latter can be installed using the command<br><br>
`sudo apt-get install python-catkin-tools`.<br><br>
The typical `catkin_tools` [workflow](https://catkin-tools.readthedocs.io/en/latest/quick_start.html) should suffice:<br><br>
```
source /opt/ros/${ROS_DISTRO}/setup.bash                       # In case you do not use the default shell of Ubuntu, you need to source another script, e.g. setup.sh.
mkdir -p ~/septentrio/src                                      # Note: Change accordingly dependending on where you want your package to be installed.
cd ~/septentrio
catkin init                                                    # Initialize with a hidden marker file
catkin config --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo   # CMake build types pass compiler-specific flags to your compiler. This type amounts to a release with debug info, while keeping debugging symbols and doing optimization. I.e. for GCC the flags would be -O2, -g and -DNDEBUG.
cd src
git clone https://github.com/tibordome/rosaic
rosdep install . --from-paths -i                               # Might raise "rosaic: Unsupported OS [mint]" warning, depending on OS.
catkin build
source ~/septentrio/devel/setup.bash 
catkin clean -y
```
Once the catkin package is installed, adapt the `rover.yaml` file according to your needs (the `mosaic_rover.launch` need not necessarily be modified). Later, one will also be able to adapt (if necessary) the `base.launch` file in the launch directory and configure it as desired.<br><br>
```
debug: 3

device: tcp://192.168.3.1:28784

frame_id: gnss

publish:
  gpgga: true
  pvtcartesian: true
  pvtgeodetic: true
  poscovgeodetic: true
  navsatfix: true

serial:
  baudrate: 115200

reconnect_delay_s: 0.1

use_GNSS_time: true
```

Subscribe to certain topics listed below (many not yet implemented!) depending on your application...
## ROS Wrapper
### ROS Parameters
The following is a list of ROS parameters found in the, say, `rover.yaml` file. The majority of the parameters is of string-type, yet some of them are integers, e.g. the parameter determining the serial baud rate or the one setting the polling period, and yet again others are boolean, e.g. `use_sbf`.
- Implemented Parameters Configuring Communication Ports and Processing of GNSS Data
  - `device`: location of device connection
    - for serial connections, the device node, e.g., `"/dev/ttyUSB0"`
    - for TCP/IP connections, a `"host:port"` specification
      - If the port is omitted, `28784` will be used as the default for TCP/IP connections. If another port is specified, the receiver needs to be (re-)configured before the ROS driver can be used.
    - default: empty string `""`  
  - `serial/baudrate`: serial baud rate to be used in a serial connection 
    - default: `115200`
  - `frame_id`: name of the ROS tf frame for the mosaic-X5, placed in the header of all published messages
    - In ROS, the [tf package](https://wiki.ros.org/tf) lets you keep track of multiple coordinate frames over time. The frame ID will be resolved by [`tf_prefix`](http://wiki.ros.org/geometry/CoordinateFrameConventions) if defined. If a ROS message has a header (all of those we publish do), the frame ID can be found via `rostopic echo /topic`, where `/topic` is the topic into which the message is being published.
    - default: `"gnss"`
  - `use_GNSS_time`:  `true` if the ROS message headers' unix epoch time field shall be constructed from the TOW (in the SBF case) and UTC (in the NMEA case) data, `false` if those times shall be constructed by the driver via the time(NULL) function found in the `ctime` library
- Planned Parameters Configuring Communication Ports and Processing of GNSS Data
  - `use_sbf`: `true` in order to request SBF blocks, `false` to request NMEA sentences
    - NMEA sentences are standardized (except for the proprietary NMEA sentences) and easier to parse, yet SBF blocks contain either more detailed or complementary information.
    - default: `true`
  - `poll_pub_pvt_period`: desired period in seconds between the polling of two consecutive `PVTGeodetic`, `PosCovGeodetic`, `PVTCartesian` and `PosCovCartesian` blocks and - if published - between the publishing of two of the corresponding ROS messages (e.g. `rosaic/PVTGeodetic.msg`) yet also [`sensor_msgs/NavSatFix.msg`](https://docs.ros.org/kinetic/api/sensor_msgs/html/msg/NavSatFix.html)
    - default: `0.05` (20 Hz)
  - `poll_pub_orientation_period`: desired period in seconds between the polling of two consecutive `AttEuler`, `AttCovEuler` blocks as well as the `HRP` NMEA sentence, and - if published - between the publishing of `AttEuler` and `AttCovEuler`
    - default: `0.05` (20 Hz)
  - `poll_pub_rest_period`: desired period in seconds between the polling of all other SBF blocks and NMEA sentences not addressed by the previous two ROS parameters, and - if published - between the publishing of all other ROS messages
    - The message [`gps_common/GPSFix.msg`](https://docs.ros.org/hydro/api/gps_common/html/msg/GPSFix.html) is - if published - published at the maximum of the all three `poll_pub_..._period`
    - default: `0.05` (20 Hz)
  - `reconnect_delay_s`: delay in seconds between reconnection attempts to the connection type specified in the parameter `connection_type`
    - default: `0.5` 
  - `navsatfix_with`: determines whether the published message [`sensor_msgs/NavSatFix.msg`](https://docs.ros.org/kinetic/api/sensor_msgs/html/msg/NavSatFix.html) is constructed from the SBF blocks `PVTGeodetic` and `PosCovGeodetic`, or from the NMEA sentences GGA and GSA via a covariance estimation algorithm, which postprocesses dilution of precision (DOP) data
    - must be one of `"SBF"` or `"NMEA"`
    - default: `"SBF"`
  - `orientation_with`: determines whether the published message [`geometry_msgs/PoseWithCovarianceStamped.msg`](https://docs.ros.org/melodic/api/geometry_msgs/html/msg/PoseWithCovarianceStamped.html) is constructed from the SBF blocks `AttEuler` and `AttCovEuler`, or from the proprietary NMEA sentence HRP
    - must be one of `"AttEuler"` or `"HRP"`
    - default: `"HRP"`
  - `ntrip_mode`: specifies the type of NTRIP connection
    - must be one of `"Server"`, `"Client"`, `"Client-Sapcorda"` or `"off"`
    - In `"Server"` mode, the receiver is sending data to an NTRIP caster. In `"Client"` mode, the receiver gets data from the "Sapcorda" NTRIP service. When selecting the `"Client-Sapcorda"` mode, no further settings are required. Note that the latter mode only works in Europe and North America. Set mode to `"off"` to disable the connection.
    - default: `"Client"`
  - `ntrip_settings`: determines NTRIP connection parameters in the format `"Caster:Port:UserName:Password:Mountpoint:Version"`
    - Here, *Caster* is the hostname or IP address of the NTRIP caster to connect to. To send data to the built-in NTRIP caster, use "localhost" for the *Caster* argument. *Port*, *UserName*, *Password* and *MountPoint* are the IP port number, the user name, the password and the mount point to be used when connecting to the NTRIP caster. The default NTRIP port number is 2101, yet you still need to include it within the string. Note that the receiver encrypts the password so that it cannot be read back with the command "getNtripSettings" (let alone via this ROS driver). The *Version* argument specifies which version of the NTRIP protocol to use ("v1" or "v2").
    - The user of this ROS driver could forward the command "getNtripSettings" to the mosaic receiver to inquire about current NTRIP settings.
    - This ROS parameter is ignored in the `"Client-Sapcorda"` mode.
    - default: `""`
  - `send_gga`: specifies whether or not to send NMEA GGA messages to the NTRIP caster, and at which rate
    - must be one of `"auto"`, `"off"`, `"sec1"`, `"sec5"`, `"sec10"` or `"sec60"`
    - In `"auto"` mode, the receiver automatically sends GGA messages if requested by the caster. 
    - This ROS parameter is ignored in the NTRIP server mode.
    - default: `"auto"`
  
- Implemented Parameters Configuring (Non-)Publishing of ROS Messages
  - `publish/navsatfix`: `true` to publish `sensor_msgs/NavSatFix.msg` messages into the topic `/navsatfix`
    - default: `true`
  - `publish/gpgga`: `true` to publish `rosaic/GPGGA.msg` messages into the topic `/gpgga`
    - default: `true`
  - `publish/pvtgeodetic`: `true` to publish `rosaic/PVTGeodetic.msg` messages into the topic `/pvtgeodetic`
    - default: `true`
  - `publish/pvtcartesian`: `true` to publish `rosaic/PVTCartesian.msg` messages into the topic `/pvtcartesian`
    - default: `true`
  - `publish/poscovgeodetic`: `true` to publish `rosaic/PosCovGeodetic.msg` messages into the topic `/poscovgeodetic`
    - default: `true`
- Planned Parameters Configuring (Non-)Publishing of ROS Messages
  - `publish/gpsfix`: `true` to publish `gps_common/GPSFix.msg` messages into the topic `/gpsfix`
    - default: `true`
  - `publish/poscovcartesian`: `true` to publish `rosaic/PosCovCartesian.msg` messages into the topic `/poscovcartesian`
    - default: `false`
  - `publish/orientation`: `true` to publish `geometry_msgs/PoseWithCovarianceStamped.msg` messages into the topic `/orientation`
    - default: `false`
  - `publish/gpst`: `true` to publish `sensor_msgs/TimeReference.msg` messages into the topic `/gpst`
    - default: `false`
  - `publish/glonasst`: `true` to publish `sensor_msgs/TimeReference.msg` messages into the topic `/glonasst`
    - default: `false`
  - `publish/gst`: `true` to publish `sensor_msgs/TimeReference.msg` messages into the topic `/gst`
    - default: `false`
  - `publish/bdt`: `true` to publish `sensor_msgs/TimeReference.msg` messages into the topic `/bdt`
    - default: `false`
  - `publish/gpgga`: `true` to publish `nmea_msgs/GPGGA.msg` messages into the topic `/gpgga`
    - default: `false`
  - `publish/gprmc`: `true` to publish `nmea_msgs/GPRMC.msg` messages into the topic `/gprmc`
    - default: `false`
  - `publish/gpgsa`: `true` to publish `nmea_msgs/GPGSA.msg` messages into the topic `/gpgsa`
    - default: `false`
  - `publish/gpgsv`: `true` to publish `nmea_msgs/GPGSV.msg` messages into the topic `/gpgsv`
    - default: `false`
  - `publish/rosdiagnostics`: `true` to publish `diagnostic_msgs/DiagnosticArray.msg` messages into the topic `/rosdiagnostics`
    - default: `false`
  - `publish/receiversetup`: `true` to publish `rosaic/ReceiverSetup.msg` messages into the topic `/receiversetup`
    - default: `false`
  - `publish/inputlink`: `true` to publish `rosaic/InputLink.msg` messages into the topic `/inputlink`
    - default: `false`
  - `publish/basestation`: `true` to publish `rosaic/BaseStation.msg` messages into the topic `/basestation`
    - default: `false`
  - `publish/rtcmdatum`: `true` to publish `rosaic/RTCMDatum.msg` messages into the topic `/rtcmdatum`
    - default: `false`
  - `publish/receivertime`: `true` to publish `rosaic/ReceiverTime.msg` messages into the topic `/receivertime`
    - default: `false`

### ROS Topic Publications
A selection of NMEA sentences, the majority being standardized sentences, and proprietary SBF blocks is translated into ROS messages, partly generic and partly custom, and can be published at the discretion of the user into the following ROS topics. Only ROS messages `sensor_msgs/NavSatFix` and `gps_common/GPSFix` are published by default. All published ROS messages, even custom ones, start with a ROS generic header [`std_msgs/Header.msg`](https://docs.ros.org/melodic/api/std_msgs/html/msg/Header.html), which includes the receiver time stamp as well as the frame ID, the latter being specified in the ROS parameter `frame_id`.
- Implemented Topics
  - `/navsatfix`: accepts generic ROS message [`sensor_msgs/NavSatFix.msg`](https://docs.ros.org/kinetic/api/sensor_msgs/html/msg/NavSatFix.html), converted from the SBF blocks `PVTGeodetic` and `PosCovGeodetic` or from the NMEA sentences GGA and GSA via a covariance estimation algorithm
    - The ROS message [`sensor_msgs/NavSatFix.msg`](https://docs.ros.org/kinetic/api/sensor_msgs/html/msg/NavSatFix.html) can be fed directly into the [`navsat_transform_node`](https://docs.ros.org/melodic/api/robot_localization/html/navsat_transform_node.html) of the ROS navigation stack.
  - `/pvtgeodetic`: accepts custom ROS message `rosaic/PVTGeodetic.msg`, corresponding to the SBF block `PVTGeodetic`
  - `/poscovgeodetic`: accepts custom ROS message `rosaic/PosCovGeodetic.msg`, corresponding to SBF block `PosCovGeodetic`
  - `/pvtcartesian`: accepts custom ROS message `rosaic/PVTCartesian.msg`, corresponding to the SBF block `PVTCartesian`
- Planned Topics
  - `/gpsfix`: accepts generic ROS message [`gps_common/GPSFix.msg`](https://docs.ros.org/hydro/api/gps_common/html/msg/GPSFix.html), which is much more detailed than [`sensor_msgs/NavSatFix.msg`](https://docs.ros.org/kinetic/api/sensor_msgs/html/msg/NavSatFix.html), converted from the SBF blocks `PVTGeodetic`, `PosCovGeodetic`, `SatVisibility`, `ChannelStatus`, `AttEuler` and `AttCovEuler` (or from proprietary NMEA sentence HRP) and `DOP` 
  - `/poscovcartesian`: accepts custom ROS message `rosaic/PosCovCartesian.msg`, corresponding to SBF block `PosCovCartesian`
  - `/orientation`: accepts generic ROS message [`geometry_msgs/PoseWithCovarianceStamped.msg`](https://docs.ros.org/melodic/api/geometry_msgs/html/msg/PoseWithCovarianceStamped.html), converted from either SBF blocks `AttEuler` and `AttCovEuler`, or from proprietary NMEA sentence HRP
  - `/atteuler`: accepts custom ROS message `rosaic/AttEuler.msg`, corresponding to SBF block `AttEuler`
  - `/attcoveuler`: accepts custom ROS message `rosaic/AttCovEuler.msg`, corresponding to the SBF block `AttCovEuler`
    - In ROS, all state estimation nodes in the [`robot_localization` package](https://docs.ros.org/melodic/api/robot_localization/html/index.html) can accept the ROS message `geometry_msgs/PoseWithCovarianceStamped.msg`.
  - `/gpst` (for GPS Time): accepts generic ROS message [`sensor_msgs/TimeReference.msg`](https://docs.ros.org/melodic/api/sensor_msgs/html/msg/TimeReference.html), converted from the SBF block `GPSUtc`
  - `/glonasst` (for GLONASS Time): accepts generic ROS message [`sensor_msgs/TimeReference.msg`](https://docs.ros.org/melodic/api/sensor_msgs/html/msg/TimeReference.html), converted from the SBF block `GLOTime`
  - `/gst` (for Galileo System Time): accepts generic ROS message [`sensor_msgs/TimeReference.msg`](https://docs.ros.org/melodic/api/sensor_msgs/html/msg/TimeReference.html), converted from the SBF block `GALUtc`
  - `/bdt` (for BeiDou Time): accepts generic ROS message [`sensor_msgs/TimeReference.msg`](https://docs.ros.org/melodic/api/sensor_msgs/html/msg/TimeReference.html), converted from the SBF block `BDSUtc`
  - `/gpgga`: accepts generic ROS message [`nmea_msgs/Gpgga.msg`](https://docs.ros.org/api/nmea_msgs/html/msg/Gpgga.html), converted from the NMEA sentence GGA
  - `/gprmc`: accepts generic ROS message [`nmea_msgs/Gprmc.msg`](https://docs.ros.org/api/nmea_msgs/html/msg/Gprmc.html), converted from the NMEA sentence RMC
  - `/gpgsa`: accepts generic ROS message [`nmea_msgs/Gpgsa.msg`](https://docs.ros.org/api/nmea_msgs/html/msg/Gpgsa.html), converted from the NMEA sentence GSA
  - `/gpgsv`: accepts generic ROS message [`nmea_msgs/Gpgsv.msg`](https://docs.ros.org/api/nmea_msgs/html/msg/Gpgsv.html), converted from the NMEA sentence GSV
  - `/hrp`: accepts custom ROS message `rosaic/HRP.msg`, corresponding to the proprietary NMEA sentence GSV
  - `/rosdiagnostics`: accepts generic ROS message [`diagnostic_msgs/DiagnosticArray.msg`](https://docs.ros.org/api/diagnostic_msgs/html/msg/DiagnosticArray.html), converted from the SBF blocks `QualityInd` and `ReceiverStatus`
  - `/receiversetup`: accepts custom ROS message `rosaic/ReceiverSetup.msg`, corresponding to the SBF block `ReceiverSetup`
  - `/inputlink`: accepts custom ROS message `rosaic/InputLink.msg`, corresponding to the SBF block `InputLink`
    - This message is useful for reporting statistics on the number of bytes and messages received and accepted on each active connection descriptor.
  - `/receivertime`: accepts custom ROS message `rosaic/ReceiverTime.msg`, corresponding to the SBF block `ReceiverTime`

## Suggestions for Improvements
- Automatic Search: If the host address of the receiver is omitted in the `host:port` specification, the driver could automatically search and establish a connection on the specified port.
- Incorporating PCAP: For PCAP connections the ROS parameter `device` could be generalized to accept the path to the .pcap file. In this case, the node could exit automatically after finishing playback.
- Publishing the topic `/basestation`: It could accept the custom ROS message `rosaic/BaseStation.msg`, corresponding to the SBF block `BaseStation` (e.g. to know position of base).
- Publishing the topic `/rtcmdatum`: It could accept the custom ROS message `rosaic/RTCMDatum.msg`, corresponding to the SBF block `RTCMDatum` (whose purpose is to get a datum that is more local).
- Attention, challenging! Publishing the topic `/measepoch`: It could accept the custom ROS message `rosaic/MeasEpoch.msg`, corresponding to the SBF block `MeasEpoch` (raw GNSS data).
- Publishing the topic `/twistwithcovariancestamped`: It could accept the generic ROS message [`geometry_msgs/TwistWithCovarianceStamped.msg`](https://docs.ros.org/melodic/api/geometry_msgs/html/msg/TwistWithCovarianceStamped.html), converted from the SBF blocks `PVTGeodetic`, `PosCovGeodetic` and some others (which?, where to get angular, i.e. rotational, velocity + covariances from?), or via standardized NMEA sentences (cf. the [NMEA driver](https://wiki.ros.org/nmea_navsat_driver)).
  - The ROS message [`geometry_msgs/TwistWithCovarianceStamped.msg`](https://docs.ros.org/melodic/api/geometry_msgs/html/msg/TwistWithCovarianceStamped.html) could be fed directly into the [`robot_localization`](https://docs.ros.org/melodic/api/robot_localization/html/index.html) nodes of the ROS navigation stack.

## Adding New SBF Blocks
Is there an SBF block that is not being addressed while being important to your application? If yes, follow these steps:
1. Find the log reference of interest in the publicly accessible, official documentation. Hence select the reference guide file in the [product support section for mosaic-X5](https://www.septentrio.com/en/support/mosaic/mosaic-x5) of Septentrio's homepage and focus on Chapter 4.
