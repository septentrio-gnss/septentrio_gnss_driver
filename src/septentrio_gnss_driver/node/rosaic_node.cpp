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
// ****************************************************************************

// Boost includes
#include <boost/regex.hpp>

#include <septentrio_gnss_driver/node/rosaic_node.hpp>

/**
 * @file rosaic_node.cpp
 * @date 22/08/20
 * @brief The heart of the ROSaic driver: The ROS node that represents it
 */

rosaic_node::ROSaicNode::ROSaicNode() :
    pNh_(new ros::NodeHandle("~")),
    IO_(pNh_)
{
    ROS_DEBUG("Called ROSaicNode() constructor..");

    // Parameters must be set before initializing IO
    connected_ = false;
    getROSParams();

    // Initializes Connection
    initializeIO();

    // Subscribes to all requested Rx messages by adding entries to the C++ multimap
    // storing the callback handlers and publishes ROS messages
    IO_.defineMessages(settings_);

    // Sends commands to the Rx regarding which SBF/NMEA messages it should output
    // and sets all its necessary corrections-related parameters
    if (!g_read_from_sbf_log && !g_read_from_pcap)
    {
        boost::mutex::scoped_lock lock(connection_mutex_);
        connection_condition_.wait(lock, [this]() { return connected_; });
        IO_.configureRx(settings_);
    }

    // Since we already have a ros::Spin() elsewhere, we use waitForShutdown() here
    ros::waitForShutdown();
    ROS_DEBUG("Leaving ROSaicNode() constructor..");
}

void rosaic_node::ROSaicNode::getROSParams()
{
    pNh_->param("use_gnss_time", g_use_gnss_time, true);
    pNh_->param("frame_id", g_frame_id, (std::string) "gnss");
    pNh_->param("publish/gpst", g_publish_gpst, true);
    pNh_->param("publish/navsatfix", g_publish_navsatfix, true);
    pNh_->param("publish/gpsfix", g_publish_gpsfix, true);
    pNh_->param("publish/pose", g_publish_pose, true);
    pNh_->param("publish/diagnostics", g_publish_diagnostics, true);
    getROSInt(pNh_, "leap_seconds", g_leap_seconds,
                           static_cast<uint32_t>(18));

    // Communication parameters
    pNh_->param("device", settings_.device, std::string("/dev/ttyACM0"));
    getROSInt(pNh_, "serial/baudrate", settings_.baudrate, static_cast<uint32_t>(115200));
    pNh_->param("serial/hw_flow_control", settings_.hw_flow_control, std::string("off"));
    pNh_->param("serial/rx_serial_port", settings_.rx_serial_port, std::string("USB1"));
    reconnect_delay_s_ = 2.0f; // Removed from ROS parameter list.
    pNh_->param("receiver_type", septentrio_receiver_type_, std::string("gnss"));
	
    // Polling period parameters
    getROSInt(pNh_, "polling_period/pvt", settings_.polling_period_pvt,
              static_cast<uint32_t>(1000));
    if (settings_.polling_period_pvt != 10 && settings_.polling_period_pvt != 20 &&
        settings_.polling_period_pvt != 50 && settings_.polling_period_pvt != 100 &&
        settings_.polling_period_pvt != 200 && settings_.polling_period_pvt != 250 &&
        settings_.polling_period_pvt != 500 && settings_.polling_period_pvt != 1000 &&
        settings_.polling_period_pvt != 2000 && settings_.polling_period_pvt != 5000 &&
        settings_.polling_period_pvt != 10000)
    {
        ROS_ERROR(
            "Please specify a valid polling period for PVT-related SBF blocks and NMEA messages.");
    }
    getROSInt(pNh_, "polling_period/rest", settings_.polling_period_rest,
              static_cast<uint32_t>(1000));
    if (settings_.polling_period_rest != 10 && settings_.polling_period_rest != 20 &&
        settings_.polling_period_rest != 50 && settings_.polling_period_rest != 100 &&
        settings_.polling_period_rest != 200 && settings_.polling_period_rest != 250 &&
        settings_.polling_period_rest != 500 && settings_.polling_period_rest != 1000 &&
        settings_.polling_period_rest != 2000 && settings_.polling_period_rest != 5000 &&
        settings_.polling_period_rest != 10000)
    {
        ROS_ERROR(
            "Please specify a valid polling period for PVT-unrelated SBF blocks and NMEA messages.");
    }

    // Datum and marker-to-ARP offset
    pNh_->param("datum", settings_.datum, std::string("ETRS89"));
    pNh_->param("ant_type", settings_.ant_type, std::string("Unknown"));
    pNh_->param("ant_aux1_type", settings_.ant_aux1_type, std::string("Unknown"));
    pNh_->param("ant_serial_nr", settings_.ant_serial_nr, std::string("Unknown"));
    pNh_->param("ant_aux1_serial_nr", settings_.ant_aux1_serial_nr, std::string("Unknown"));
    pNh_->param("poi_to_arp/delta_e", settings_.delta_e, 0.0f);
    pNh_->param("poi_to_arp/delta_n", settings_.delta_n, 0.0f);
    pNh_->param("poi_to_arp/delta_u", settings_.delta_u, 0.0f);
    pNh_->param("poi_to_aux1_arp/delta_e", settings_.delta_aux1_e, 0.0f);
    pNh_->param("poi_to_aux1_arp/delta_n", settings_.delta_aux1_n, 0.0f);
    pNh_->param("poi_to_aux1_arp/delta_u", settings_.delta_aux1_u, 0.0f);
	
	// INS Spatial Configuration
    // IMU orientation parameter
    pNh_->param("ins_spatial_config/imu_orientation/theta_x", settings_.theta_x, 0.0f);
    pNh_->param("ins_spatial_config/imu_orientation/theta_y", settings_.theta_y_, 0.0f);
    pNh_->param("ins_spatial_config/imu_orientation/theta_z", settings_.theta_z, 0.0f);
	
    // INS antenna lever arm offset parameter
    pNh_->param("ins_spatial_config/ant_lever_arm/x", settings_.ant_lever_x, 0.0f);
    pNh_->param("ins_spatial_config/ant_lever_arm/y", settings_.ant_lever_y, 0.0f);
    pNh_->param("ins_spatial_config/ant_lever_arm/z", settings_.ant_lever_z, 0.0f);

    // INS POI ofset paramter
    pNh_->param("ins_spatial_config/poi_to_imu/delta_x", settings_.poi_x, 0.0f);
    pNh_->param("ins_spatial_config/poi_to_imu/delta_y", settings_.poi_y, 0.0f);
    pNh_->param("ins_spatial_config/poi_to_imu/delta_z", settings_.poi_z, 0.0f);

    // INS velocity sensor lever arm offset parameter
    pNh_->param("ins_spatial_config/vel_sensor_lever_arm/vsm_x", settings_.vsm_x, 0.0f);
    pNh_->param("ins_spatial_config/vel_sensor_lever_arm/vsm_y", settings_.vsm_y, 0.0f);
    pNh_->param("ins_spatial_config/vel_sensor_lever_arm/vsm_z", settings_.vsm_z, 0.0f);
    
    // Attitude Determination parameter
    pNh_->param("ins_spatial_config/att_offset/heading", settings_.heading_offset, 0.0f);
    pNh_->param("ins_spatial_config/att_offset/pitch", settings_.pitch_offset, 0.0f);

    // ins_initial_heading param
    pNh_->param("ins_initial_heading", settings_.ins_initial_heading, std::string("auto"));

    // ins_std_dev_mask
    pNh_->param("ins_std_dev_mask/att_std_dev", settings_.att_std_dev, 0.0f);
    pNh_->param("ins_std_dev_mask/pos_std_dev", settings_.pos_std_dev, 0.0f);

    // INS solution reference point
    pNh_->param("ins_use_poi", settings_.ins_use_poi, false);

    // Correction service parameters
    pNh_->param("ntrip_settings/mode", settings_.ntrip_mode, std::string("off"));
    pNh_->param("ntrip_settings/caster", settings_.caster, std::string());
    getROSInt(pNh_, "ntrip_settings/caster_port", settings_.caster_port, static_cast<uint32_t>(0));
    pNh_->param("ntrip_settings/username", settings_.ntrip_username, std::string());
    pNh_->param("ntrip_settings/password", settings_.ntrip_password, std::string());
    if (settings_.ntrip_password.empty())
    {
        uint32_t pwd_tmp;
        getROSInt(pNh_, "ntrip_settings/password", pwd_tmp, static_cast<uint32_t>(0));
        settings_.ntrip_password = std::to_string(pwd_tmp);
    }
    pNh_->param("ntrip_settings/mountpoint", settings_.mountpoint, std::string());
    pNh_->param("ntrip_settings/ntrip_version", settings_.ntrip_version, std::string("v2"));
    pNh_->param("ntrip_settings/send_gga", settings_.send_gga, std::string("auto"));
    pNh_->param("ntrip_settings/rx_has_internet", settings_.rx_has_internet, false);
    pNh_->param("ntrip_settings/rtcm_version", settings_.rtcm_version, std::string("RTCMv3"));
    getROSInt(pNh_, "ntrip_settings/rx_input_corrections_tcp", settings_.rx_input_corrections_tcp,
              static_cast<uint32_t>(28785));
    pNh_->param("ntrip_settings/rx_input_corrections_serial",
                settings_.rx_input_corrections_serial, std::string("USB2"));

    // Publishing parameters, the others remained global since they need to be
    // accessed in the callbackhandlers.hpp file
    pNh_->param("publish/gpgga", settings_.publish_gpgga, true);
    pNh_->param("publish/gprmc", settings_.publish_gprmc, true);
    pNh_->param("publish/gpgsa", settings_.publish_gpgsa, true);
    pNh_->param("publish/gpgsv", settings_.publish_gpgsv, true);
    pNh_->param("publish/pvtcartesian", settings_.publish_pvtcartesian, true);
    pNh_->param("publish/pvtgeodetic", settings_.publish_pvtgeodetic, true);
    pNh_->param("publish/poscovcartesian", settings_.publish_poscovcartesian, true);
    pNh_->param("publish/poscovgeodetic", settings_.publish_poscovgeodetic, true);
	pNh_->param("publish/velcovgeodetic", settings_.publish_velcovgeodetic, true);
    pNh_->param("publish/atteuler", settings_.publish_atteuler, true);
    pNh_->param("publish/attcoveuler", settings_.publish_attcoveuler, true);
    pNh_->param("publish/insnavcart", settings_.publish_insnavcart, true);
    pNh_->param("publish/insnavgeod", settings_.publish_insnavgeod, true);
    pNh_->param("publish/imusetup", settings_.publish_imusetup, true);
    pNh_->param("publish/velsensorsetup", settings_.publish_velsensorsetup, true);
    pNh_->param("publish/exteventinsnavgeod", settings_.publish_exteventinsnavgeod, true);
    pNh_->param("publish/exteventinsnavcart", settings_.publish_exteventinsnavcart, true);
    pNh_->param("publish/extsensormeas", settings_.publish_extsensormeas, true);
	
    // To be implemented: RTCM, raw data settings, PPP, SBAS ...
    ROS_DEBUG("Finished getROSParams() method");
};

void rosaic_node::ROSaicNode::initializeIO()
{
    ROS_DEBUG("Called initializeIO() method");
    boost::smatch match;
    // In fact: smatch is a typedef of match_results<string::const_iterator>
    if (boost::regex_match(settings_.device, match, boost::regex("(tcp)://(.+):(\\d+)")))
    // C++ needs \\d instead of \d: Both mean decimal.
    // Note that regex_match can be used with a smatch object to store results, or
    // without. In any case, true is returned if and only if it matches the
    // !complete! string.
    {
        // The first sub_match (index 0) contained in a match_result always
        // represents the full match within a target sequence made by a regex, and
        // subsequent sub_matches represent sub-expression matches corresponding in
        // sequence to the left parenthesis delimiting the sub-expression in the
        // regex, i.e. $n Perl is equivalent to m[n] in boost regex.
        tcp_host_ = match[2];
        tcp_port_ = match[3];

        serial_ = false;
        g_read_from_sbf_log = false;
        g_read_from_pcap = false;
        boost::thread temporary_thread(boost::bind(&ROSaicNode::connect, this));
        temporary_thread.detach();
    } else if (boost::regex_match(settings_.device, match,
                                  boost::regex("(file_name):(/|(?:/[\\w-]+)+.sbf)")))
    {
        serial_ = false;
        g_read_from_sbf_log = true;
        g_read_from_pcap = false;
        boost::thread temporary_thread(
            boost::bind(&ROSaicNode::prepareSBFFileReading, this, match[2]));
        temporary_thread.detach();

    } else if (boost::regex_match(
                   settings_.device, match,
                   boost::regex("(file_name):(/|(?:/[\\w-]+)+.pcap)")))
    {
        serial_ = false;
        g_read_from_sbf_log = false;
        g_read_from_pcap = true;
        boost::thread temporary_thread(
            boost::bind(&ROSaicNode::preparePCAPFileReading, this, match[2]));
        temporary_thread.detach();

    } else if (boost::regex_match(settings_.device, match, boost::regex("(serial):(.+)")))
    {
        serial_ = true;
        g_read_from_sbf_log = false;
        g_read_from_pcap = false;
        std::string proto(match[2]);
        std::stringstream ss;
        ss << "Searching for serial port" << proto;
		settings_.device = proto;
        ROS_DEBUG("%s", ss.str().c_str());
        boost::thread temporary_thread(boost::bind(&ROSaicNode::connect, this));
        temporary_thread.detach();
    } else
    {
        std::stringstream ss;
        ss << "Device is unsupported. Perhaps you meant 'tcp://host:port' or 'file_name:xxx.sbf' or 'serial:/path/to/device'?";
        ROS_ERROR("%s", ss.str().c_str());
    }
    ROS_DEBUG("Leaving initializeIO() method");
}

void rosaic_node::ROSaicNode::prepareSBFFileReading(std::string file_name)
{
    try
    {
        std::stringstream ss;
        ss << "Setting up everything needed to read from" << file_name;
        ROS_DEBUG("%s", ss.str().c_str());
        IO_.initializeSBFFileReading(file_name);
    } catch (std::runtime_error& e)
    {
        std::stringstream ss;
        ss << "Comm_IO::initializeSBFFileReading() failed for SBF File" << file_name
           << " due to: " << e.what();
        ROS_ERROR("%s", ss.str().c_str());
    }
}

void rosaic_node::ROSaicNode::preparePCAPFileReading(std::string file_name)
{
    try
    {
        std::stringstream ss;
        ss << "Setting up everything needed to read from " << file_name;
        ROS_DEBUG("%s", ss.str().c_str());
        IO_.initializePCAPFileReading(file_name);
    } catch (std::runtime_error& e)
    {
        std::stringstream ss;
        ss << "CommIO::initializePCAPFileReading() failed for SBF File " << file_name
           << " due to: " << e.what();
        ROS_ERROR("%s", ss.str().c_str());
    }
}

void rosaic_node::ROSaicNode::connect()
{
    ROS_DEBUG("Called connect() method");
    ROS_DEBUG(
        "Setting ROS timer for calling reconnect() method until connection succeeds");
    reconnect_timer_ = pNh_->createTimer(ros::Duration(reconnect_delay_s_),
                                         &ROSaicNode::reconnect, this);
    reconnect_timer_.start();
    ROS_DEBUG(
        "Started ROS timer for calling reconnect() method until connection succeeds");
    ros::spin();
    ROS_DEBUG("Leaving connect() method"); // This will never be output since
                                           // ros::spin() is on the line above.
}

//! In serial mode (not USB, since the Rx port is then called USB1 or USB2), please
//! ensure that you are connected to the Rx's COM1, COM2 or COM3 port, !if! you
//! employ UART hardware flow control.
void rosaic_node::ROSaicNode::reconnect(const ros::TimerEvent& event)
{
    ROS_DEBUG("Called reconnect() method");
    if (connected_ == true)
    {
        reconnect_timer_.stop();
        ROS_DEBUG("Stopped ROS timer since successully connected.");
    } else
    {
        if (serial_)
        {
            bool initialize_serial_return = false;
            try
            {
                ROS_INFO("Connecting serially to device %s, targeted baudrate: %u",
                         settings_.device.c_str(), settings_.baudrate);
                initialize_serial_return =
                    IO_.initializeSerial(settings_.device, settings_.baudrate, settings_.hw_flow_control);
            } catch (std::runtime_error& e)
            {
                {
                    std::stringstream ss;
                    ss << "IO_.initializeSerial() failed for device " << settings_.device
                       << " due to: " << e.what();
                    ROS_ERROR("%s", ss.str().c_str());
                }
            }
            if (initialize_serial_return)
            {
                boost::mutex::scoped_lock lock(connection_mutex_);
                connected_ = true;
                lock.unlock();
                connection_condition_.notify_one();
            }
        } else
        {
            bool initialize_tcp_return = false;
            try
            {
                ROS_INFO("Connecting to tcp://%s:%s ...", tcp_host_.c_str(),
                         tcp_port_.c_str());
                initialize_tcp_return = IO_.initializeTCP(tcp_host_, tcp_port_);
            } catch (std::runtime_error& e)
            {
                {
                    std::stringstream ss;
                    ss << "IO_.initializeTCP() failed for host " << tcp_host_
                       << " on port " << tcp_port_ << " due to: " << e.what();
                    ROS_ERROR("%s", ss.str().c_str());
                }
            }
            if (initialize_tcp_return)
            {
                boost::mutex::scoped_lock lock(connection_mutex_);
                connected_ = true;
                lock.unlock();
                connection_condition_.notify_one();
            }
        }
    }
    ROS_DEBUG("Leaving reconnect() method");
}

//! If true, the ROS message headers' unix time field is constructed from the TOW (in
//! the SBF case) and UTC (in the NMEA case) data. If false, times are constructed
//! within the driver via time(NULL) of the \<ctime\> library.
bool g_use_gnss_time;
//! Whether or not to publish the sensor_msgs::TimeReference message with GPST
bool g_publish_gpst;
//! Whether or not to publish the sensor_msgs::NavSatFix message
bool g_publish_navsatfix;
//! Whether or not to publish the gps_common::GPSFix message
bool g_publish_gpsfix;
//! Whether or not to publish the geometry_msgs::PoseWithCovarianceStamped message
bool g_publish_pose;
//! Whether or not to publish the diagnostic_msgs::DiagnosticArray message
bool g_publish_diagnostics;
//! The frame ID used in the header of every published ROS message
std::string g_frame_id;
//! The number of leap seconds that have been inserted into the UTC time
uint32_t g_leap_seconds;
//! Rx TCP port, e.g. IP10 or IP11, to which ROSaic is connected to
std::string g_rx_tcp_port;
//! Since after SSSSSSSSSSS we need to wait for second connection descriptor, we have
//! to count the connection descriptors
uint32_t g_cd_count;
//! Whether or not we are reading from an SBF file
bool g_read_from_sbf_log;
//! Whether or not we are reading from a PCAP file
bool g_read_from_pcap;
//! A C++ map for keeping track of the SBF blocks necessary to construct the GPSFix
//! ROS message
std::map<std::string, uint32_t> g_GPSFixMap;
//! A C++ map for keeping track of the SBF blocks necessary to construct the
//! NavSatFix ROS message
std::map<std::string, uint32_t> g_NavSatFixMap;
//! A C++ map for keeping track of SBF blocks necessary to construct the
//! PoseWithCovarianceStamped ROS message
std::map<std::string, uint32_t> g_PoseWithCovarianceStampedMap;
//! A C++ map for keeping track of SBF blocks necessary to construct the
//! DiagnosticArray ROS message
std::map<std::string, uint32_t> g_DiagnosticArrayMap;
//! Queue size for ROS publishers
const uint32_t g_ROS_QUEUE_SIZE = 1;
//! Septentrio receiver type, either "gnss" or "ins"
std::string septentrio_receiver_type_;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "septentrio_gnss");
  	
    // The info logging level seems to be default, hence we modify log level
    // momentarily.. The following is the C++ version of
    // rospy.init_node('my_ros_node', log_level=rospy.DEBUG)
    if (ros::console::set_logger_level(
            ROSCONSOLE_DEFAULT_NAME,
            ros::console::levels::Debug)) // debug is lowest level, shows everything
        ros::console::notifyLoggerLevelsChanged();

    rosaic_node::ROSaicNode
        rx_node; // This launches everything we need, in theory :)
    return 0;
}