// *****************************************************************************
//
// © Copyright 2020, Septentrio NV/SA.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//    1. Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//    2. Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//    3. Neither the name of the copyright holder nor the names of its
//       contributors may be used to endorse or promote products derived
//       from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// *****************************************************************************

#pragma once

#include <stdint.h>
#include <string>
#include <vector>

struct Osnma
{
    //! OSNMA mode
    std::string mode;
    //! Server for NTP synchronization
    std::string ntp_server;
    //! Wether OSNMA shall be kept open on shutdown
    bool keep_open;
};

struct RtkNtrip
{
    //! Id of the NTRIP port
    std::string id;
    //! Hostname or IP address of the NTRIP caster to connect to
    std::string caster;
    //! IP port number of NTRIP caster to connect to
    uint32_t caster_port;
    //! Username for NTRIP service
    std::string username;
    //! Password for NTRIP service
    std::string password;
    //! Mountpoint for NTRIP service
    std::string mountpoint;
    //! NTRIP version for NTRIP service
    std::string version;
    //! Wether to use TLS
    bool tls;
    //! Self-signed certificate fingerprint
    std::string fingerprint;
    //! RTCM version for correction data
    std::string rtk_standard;
    //! Whether (and at which rate) or not to send GGA to the NTRIP caster
    std::string send_gga;
    //! Wether RTK connections shall be kept open on shutdown
    bool keep_open;
};

struct RtkIpServer
{
    //! The IP server id
    std::string id;
    //! Rx TCP port number, e.g. 28785, on which Rx receives the corrections
    //! (can't be the same as main connection unless localhost concept is used)
    uint32_t port;
    //! RTCM version for correction data
    std::string rtk_standard;
    //! Whether (and at which rate) or not to send GGA to the NTRIP caster
    std::string send_gga;
    //! Wether RTK connections shall be kept open on shutdown
    bool keep_open;
};

struct RtkSerial
{
    //! Rx serial port, e.g. USB2, on which Rx receives the corrections (can't be
    //! the same as main connection unless localhost concept is used)
    std::string port;
    //! Baud rate of the serial port on which Rx receives the corrections
    uint32_t baud_rate;
    //! RTCM version for correction data
    std::string rtk_standard;
    //! Whether (and at which rate) or not to send GGA to the serial port
    std::string send_gga;
    //! Wether RTK connections shall be kept open on shutdown
    bool keep_open;
};

struct Rtk
{
    std::vector<RtkNtrip> ntrip;
    std::vector<RtkIpServer> ip_server;
    std::vector<RtkSerial> serial;
};

namespace device_type {
    enum DeviceType
    {
        TCP,
        SERIAL,
        SBF_FILE,
        PCAP_FILE
    };
} // namespace device_type

//! Settings struct
struct Settings
{
    //! Set logger level to DEBUG
    bool activate_debug_log;
    //! Device
    std::string device;
    //! Device type
    device_type::DeviceType device_type;
    //! TCP IP
    std::string device_tcp_ip;
    //! TCP port
    std::string device_tcp_port;
    //! UDP port
    uint32_t udp_port;
    //! UDP unicast destination ip
    std::string udp_unicast_ip;
    //! UDP IP server id
    std::string udp_ip_server;
    //! TCP port
    uint32_t tcp_port;
    //! TCP IP server id
    std::string tcp_ip_server;
    //! Filename
    std::string file_name;
    //! Username for login
    std::string login_user;
    //! Password for login
    std::string login_password;
    //! Delay in seconds between reconnection attempts to the connection type
    //! specified in the parameter connection_type
    float reconnect_delay_s;
    //! Baudrate
    uint32_t baudrate;
    //! HW flow control
    std::string hw_flow_control;
    // Wether to configure Rx
    bool configure_rx;
    //! Datum to be used
    std::string datum;
    //! Polling period for PVT-related SBF blocks
    uint32_t polling_period_pvt;
    //! Polling period for all other SBF blocks and NMEA messages
    uint32_t polling_period_rest;
    //! Marker-to-ARP offset in the eastward direction
    float delta_e;
    //! Marker-to-ARP offset in the northward direction
    float delta_n;
    //! Marker-to-ARP offset in the upward direction
    float delta_u;
    //! Main antenna type, from the list returned by the command "lstAntennaInfo,
    //! Overview"
    std::string ant_type;
    //! Aux1 antenna type, from the list returned by the command "lstAntennaInfo,
    //! Overview"
    std::string ant_aux1_type;
    //! Serial number of your particular Main antenna
    std::string ant_serial_nr;
    //! Serial number of your particular Aux1 antenna
    std::string ant_aux1_serial_nr;
    //! ROS axis orientation, body: front-left-up, geographic: ENU
    bool use_ros_axis_orientation;
    //! IMU orientation x-angle
    double theta_x;
    //! IMU orientation y-angle
    double theta_y;
    //! IMU orientation z-angle
    double theta_z;
    //! INS antenna lever arm x-offset
    double ant_lever_x;
    //! INS antenna lever arm y-offset
    double ant_lever_y;
    //! INS antenna lever arm z-offset
    double ant_lever_z;
    //! INS POI offset in x-dimension
    double poi_x;
    //! INS POI offset in y-dimension
    double poi_y;
    //! INS POI offset in z-dimension
    double poi_z;
    //! INS velocity sensor lever arm x-offset
    double vsm_x;
    //! INS velocity sensor lever arm y-offset
    double vsm_y;
    //! INS velocity sensor lever arm z-offset
    double vsm_z;
    //! Attitude offset determination in longitudinal direction
    double heading_offset;
    //! Attitude offset determination in latitudinal direction
    double pitch_offset;
    //! INS multiantenna
    bool multi_antenna;
    //! INS solution reference point
    bool ins_use_poi;
    //! For heading computation when unit is powered-cycled
    std::string ins_initial_heading;
    //! Attitude deviation mask
    float att_std_dev;
    //! Position deviation mask
    float pos_std_dev;
    //! RTK corrections settings
    Rtk rtk;
    //! OSNMA settings
    Osnma osnma;
    //! Whether or not to publish the GGA message
    bool publish_gpgga;
    //! Whether or not to publish the RMC message
    bool publish_gprmc;
    //! Whether or not to publish the GSA message
    bool publish_gpgsa;
    //! Whether or not to publish the GSV message
    bool publish_gpgsv;
    //! Whether or not to publish the MeasEpoch message
    bool publish_measepoch;
    //! Whether or not to publish the RFStatus and AIMPlusStatus message and
    //! diagnostics
    bool publish_aimplusstatus;
    //! Whether or not to publish the GALAuthStatus message and diagnostics
    bool publish_galauthstatus;
    //! Whether or not to publish the PVTCartesianMsg
    //! message
    bool publish_pvtcartesian;
    //! Whether or not to publish the PVTGeodeticMsg message
    bool publish_pvtgeodetic;
    //! Whether or not to publish the BaseVectorCartMsg
    //! message
    bool publish_basevectorcart;
    //! Whether or not to publish the BaseVectorGeodMsg message
    bool publish_basevectorgeod;
    //! Whether or not to publish the PosCovCartesianMsg
    //! message
    bool publish_poscovcartesian;
    //! Whether or not to publish the PosCovGeodeticMsg
    //! message
    bool publish_poscovgeodetic;
    //! Whether or not to publish the VelCovCartesianMsg
    //! message
    bool publish_velcovcartesian;
    //! Whether or not to publish the VelCovGeodeticMsg
    //! message
    bool publish_velcovgeodetic;
    //! Whether or not to publish the AttEulerMsg message
    bool publish_atteuler;
    //! Whether or not to publish the AttCovEulerMsg message
    bool publish_attcoveuler;
    //! Whether or not to publish the INSNavCartMsg message
    bool publish_insnavcart;
    //! Whether or not to publish the INSNavGeodMsg message
    bool publish_insnavgeod;
    //! Whether or not to publish the IMUSetupMsg message
    bool publish_imusetup;
    //! Whether or not to publish the VelSensorSetupMsg message
    bool publish_velsensorsetup;
    //! Whether or not to publish the ExtEventINSNavGeodMsg message
    bool publish_exteventinsnavgeod;
    //! Whether or not to publish the ExtEventINSNavCartMsg message
    bool publish_exteventinsnavcart;
    //! Whether or not to publish the ExtSensorMeasMsg message
    bool publish_extsensormeas;
    //! Whether or not to publish the TimeReferenceMsg message with GPST
    bool publish_gpst;
    //! Whether or not to publish the NavSatFixMsg message
    bool publish_navsatfix;
    //! Whether or not to publish the GpsFixMsg message
    bool publish_gpsfix;
    //! Whether or not to publish the PoseWithCovarianceStampedMsg message
    bool publish_pose;
    //! Whether or not to publish the DiagnosticArrayMsg message
    bool publish_diagnostics;
    //! Whether or not to publish the ImuMsg message
    bool publish_imu;
    //! Whether or not to publish the LocalizationMsg message
    bool publish_localization;
    //! Whether or not to publish the LocalizationMsg message
    bool publish_localization_ecef;
    //! Whether or not to publish the TwistWithCovarianceStampedMsg message
    bool publish_twist;
    //! Whether or not to publish the tf of the localization
    bool publish_tf;
    //! Whether or not to publish the tf of the localization
    bool publish_tf_ecef;
    //! Wether local frame should be inserted into tf
    bool insert_local_frame = false;
    //! Frame id of the local frame to be inserted
    std::string local_frame_id;
    //! Septentrio receiver type, either "gnss" or "ins"
    std::string septentrio_receiver_type;
    //! If true, the ROS message headers' unix time field is constructed from the TOW
    //! (in the SBF case) and UTC (in the NMEA case) data. If false, times are
    //! constructed within the driver via time(NULL) of the \<ctime\> library.
    bool use_gnss_time;
    //! Wether processing latency shall be compensated for in ROS timestamp
    bool latency_compensation;
    //! The frame ID used in the header of every published ROS message
    std::string frame_id;
    //! The frame ID used in the header of published ROS Imu message
    std::string imu_frame_id;
    //! The frame ID used in the header of published ROS Localization message if poi
    //! is used
    std::string poi_frame_id;
    //! The frame ID of the velocity sensor
    std::string vsm_frame_id;
    //! The frame ID of the aux1 antenna
    std::string aux1_frame_id;
    //! The frame ID of the vehicle frame
    std::string vehicle_frame_id;
    //! Wether the UTM zone of the localization is locked
    bool lock_utm_zone;
    //! The number of leap seconds that have been inserted into the UTC time
    int32_t leap_seconds = -128;
    //! Whether or not we are reading from an SBF file
    bool read_from_sbf_log = false;
    //! Whether or not we are reading from a PCAP file
    bool read_from_pcap = false;
    //! VSM source for INS
    std::string ins_vsm_ros_source;
    //! Whether or not to use individual elements of 3D velocity (v_x, v_y, v_z)
    std::vector<bool> ins_vsm_ros_config = {false, false, false};
    //! Whether or not to use variance defined by ROS parameter
    bool ins_vsm_ros_variances_by_parameter = false;
    //! Variances of the 3D velocity (var_x, var_y, var_z)
    std::vector<double> ins_vsm_ros_variances = {-1.0, -1.0, -1.0};
    //! VSM IP server id
    std::string ins_vsm_ip_server_id;
    //! VSM tcp port
    uint32_t ins_vsm_ip_server_port;
    //! Wether VSM shall be kept open om shutdown
    bool ins_vsm_ip_server_keep_open;
    //! VSM serial port
    std::string ins_vsm_serial_port;
    //! VSM serial baud rate
    uint32_t ins_vsm_serial_baud_rate;
    //! Wether VSM shall be kept open om shutdown
    bool ins_vsm_serial_keep_open;
};

//! Capabilities struct
struct Capabilities
{
    //! Wether Rx is INS
    bool is_ins = false;
    //! Wether Rx has heading
    bool has_heading = false;
    //! Wether Rx has improved VSM handling
    bool has_improved_vsm_handling = false;
};