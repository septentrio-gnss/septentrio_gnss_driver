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

#include <septentrio_gnss_driver/node/rosaic_node.hpp>

/**
 * @file rosaic_node.cpp
 * @date 22/08/20
 * @brief The heart of the ROSaic driver: The ROS node that represents it
 */

rosaic_node::ROSaicNode::ROSaicNode() :
    IO_(this, &settings_)
{
    this->log(LogLevel::DEBUG, "Called ROSaicNode() constructor..");

    // Parameters must be set before initializing IO
    getROSParams();

    // Initializes Connection
    IO_.initializeIO();

    // Subscribes to all requested Rx messages by adding entries to the C++ multimap
    // storing the callback handlers and publishes ROS messages
    IO_.defineMessages();

    // Sends commands to the Rx regarding which SBF/NMEA messages it should output
    // and sets all its necessary corrections-related parameters
    if (!settings_.read_from_sbf_log && !settings_.read_from_pcap)
    {
        IO_.configureRx();
    }

    this->log(LogLevel::DEBUG, "Leaving ROSaicNode() constructor..");
}

void rosaic_node::ROSaicNode::getROSParams()
{
    param("use_gnss_time", settings_.use_gnss_time, true);
    param("frame_id", settings_.frame_id, (std::string) "gnss");
    param("publish/gpst", settings_.publish_gpst, true);
    param("publish/navsatfix", settings_.publish_navsatfix, true);
    param("publish/gpsfix", settings_.publish_gpsfix, true);
    param("publish/pose", settings_.publish_pose, true);
    param("publish/diagnostics", settings_.publish_diagnostics, true);
    getIntParam("leap_seconds", settings_.leap_seconds,
                           static_cast<uint32_t>(18));

    // Communication parameters
    param("device", settings_.device, std::string("/dev/ttyACM0"));
    getIntParam("serial/baudrate", settings_.baudrate, static_cast<uint32_t>(115200));
    param("serial/hw_flow_control", settings_.hw_flow_control, std::string("off"));
    param("serial/rx_serial_port", settings_.rx_serial_port, std::string("USB1"));
    settings_.reconnect_delay_s = 2.0f; // Removed from ROS parameter list.
    param("receiver_type", settings_.septentrio_receiver_type, std::string("gnss"));
	
    // Polling period parameters
    getIntParam("polling_period/pvt", settings_.polling_period_pvt,
              static_cast<uint32_t>(1000));
    if (settings_.polling_period_pvt != 10 && settings_.polling_period_pvt != 20 &&
        settings_.polling_period_pvt != 50 && settings_.polling_period_pvt != 100 &&
        settings_.polling_period_pvt != 200 && settings_.polling_period_pvt != 250 &&
        settings_.polling_period_pvt != 500 && settings_.polling_period_pvt != 1000 &&
        settings_.polling_period_pvt != 2000 && settings_.polling_period_pvt != 5000 &&
        settings_.polling_period_pvt != 10000)
    {
        this->log(LogLevel::ERROR,
            "Please specify a valid polling period for PVT-related SBF blocks and NMEA messages.");
    }
    getIntParam("polling_period/rest", settings_.polling_period_rest,
              static_cast<uint32_t>(1000));
    if (settings_.polling_period_rest != 10 && settings_.polling_period_rest != 20 &&
        settings_.polling_period_rest != 50 && settings_.polling_period_rest != 100 &&
        settings_.polling_period_rest != 200 && settings_.polling_period_rest != 250 &&
        settings_.polling_period_rest != 500 && settings_.polling_period_rest != 1000 &&
        settings_.polling_period_rest != 2000 && settings_.polling_period_rest != 5000 &&
        settings_.polling_period_rest != 10000)
    {
        this->log(LogLevel::ERROR,
            "Please specify a valid polling period for PVT-unrelated SBF blocks and NMEA messages.");
    }

    // Datum and marker-to-ARP offset
    param("datum", settings_.datum, std::string("ETRS89"));
    param("ant_type", settings_.ant_type, std::string("Unknown"));
    param("ant_aux1_type", settings_.ant_aux1_type, std::string("Unknown"));
    param("ant_serial_nr", settings_.ant_serial_nr, std::string("Unknown"));
    param("ant_aux1_serial_nr", settings_.ant_aux1_serial_nr, std::string("Unknown"));
    param("poi_to_arp/delta_e", settings_.delta_e, 0.0f);
    param("poi_to_arp/delta_n", settings_.delta_n, 0.0f);
    param("poi_to_arp/delta_u", settings_.delta_u, 0.0f);
    param("poi_to_aux1_arp/delta_e", settings_.delta_aux1_e, 0.0f);
    param("poi_to_aux1_arp/delta_n", settings_.delta_aux1_n, 0.0f);
    param("poi_to_aux1_arp/delta_u", settings_.delta_aux1_u, 0.0f);
	
	// INS Spatial Configuration
    // IMU orientation parameter
    param("ins_spatial_config/imu_orientation/theta_x", settings_.theta_x, 0.0f);
    param("ins_spatial_config/imu_orientation/theta_y", settings_.theta_y, 0.0f);
    param("ins_spatial_config/imu_orientation/theta_z", settings_.theta_z, 0.0f);
	
    // INS antenna lever arm offset parameter
    param("ins_spatial_config/ant_lever_arm/x", settings_.ant_lever_x, 0.0f);
    param("ins_spatial_config/ant_lever_arm/y", settings_.ant_lever_y, 0.0f);
    param("ins_spatial_config/ant_lever_arm/z", settings_.ant_lever_z, 0.0f);

    // INS POI ofset paramter
    param("ins_spatial_config/poi_to_imu/delta_x", settings_.poi_x, 0.0f);
    param("ins_spatial_config/poi_to_imu/delta_y", settings_.poi_y, 0.0f);
    param("ins_spatial_config/poi_to_imu/delta_z", settings_.poi_z, 0.0f);

    // INS velocity sensor lever arm offset parameter
    param("ins_spatial_config/vel_sensor_lever_arm/vsm_x", settings_.vsm_x, 0.0f);
    param("ins_spatial_config/vel_sensor_lever_arm/vsm_y", settings_.vsm_y, 0.0f);
    param("ins_spatial_config/vel_sensor_lever_arm/vsm_z", settings_.vsm_z, 0.0f);
    
    // Attitude Determination parameter
    param("ins_spatial_config/att_offset/heading", settings_.heading_offset, 0.0f);
    param("ins_spatial_config/att_offset/pitch", settings_.pitch_offset, 0.0f);

    // ins_initial_heading param
    param("ins_initial_heading", settings_.ins_initial_heading, std::string("auto"));

    // ins_std_dev_mask
    param("ins_std_dev_mask/att_std_dev", settings_.att_std_dev, 0.0f);
    param("ins_std_dev_mask/pos_std_dev", settings_.pos_std_dev, 0.0f);

    // INS solution reference point
    param("ins_use_poi", settings_.ins_use_poi, false);

    // Correction service parameters
    param("ntrip_settings/mode", settings_.ntrip_mode, std::string("off"));
    param("ntrip_settings/caster", settings_.caster, std::string());
    getIntParam("ntrip_settings/caster_port", settings_.caster_port, static_cast<uint32_t>(0));
    param("ntrip_settings/username", settings_.ntrip_username, std::string());
    param("ntrip_settings/password", settings_.ntrip_password, std::string());
    if (settings_.ntrip_password.empty())
    {
        uint32_t pwd_tmp;
        getIntParam("ntrip_settings/password", pwd_tmp, static_cast<uint32_t>(0));
        settings_.ntrip_password = std::to_string(pwd_tmp);
    }
    param("ntrip_settings/mountpoint", settings_.mountpoint, std::string());
    param("ntrip_settings/ntrip_version", settings_.ntrip_version, std::string("v2"));
    param("ntrip_settings/send_gga", settings_.send_gga, std::string("auto"));
    param("ntrip_settings/rx_has_internet", settings_.rx_has_internet, false);
    param("ntrip_settings/rtcm_version", settings_.rtcm_version, std::string("RTCMv3"));
    getIntParam("ntrip_settings/rx_input_corrections_tcp", settings_.rx_input_corrections_tcp,
              static_cast<uint32_t>(28785));
    param("ntrip_settings/rx_input_corrections_serial",
                settings_.rx_input_corrections_serial, std::string("USB2"));

    // Publishing parameters, the others remained global since they need to be
    // accessed in the callbackhandlers.hpp file
    param("publish/gpgga", settings_.publish_gpgga, true);
    param("publish/gprmc", settings_.publish_gprmc, true);
    param("publish/gpgsa", settings_.publish_gpgsa, true);
    param("publish/gpgsv", settings_.publish_gpgsv, true);
    param("publish/pvtcartesian", settings_.publish_pvtcartesian, true);
    param("publish/pvtgeodetic", settings_.publish_pvtgeodetic, true);
    param("publish/poscovcartesian", settings_.publish_poscovcartesian, true);
    param("publish/poscovgeodetic", settings_.publish_poscovgeodetic, true);
	param("publish/velcovgeodetic", settings_.publish_velcovgeodetic, true);
    param("publish/atteuler", settings_.publish_atteuler, true);
    param("publish/attcoveuler", settings_.publish_attcoveuler, true);
    param("publish/insnavcart", settings_.publish_insnavcart, true);
    param("publish/insnavgeod", settings_.publish_insnavgeod, true);
    param("publish/imusetup", settings_.publish_imusetup, true);
    param("publish/velsensorsetup", settings_.publish_velsensorsetup, true);
    param("publish/exteventinsnavgeod", settings_.publish_exteventinsnavgeod, true);
    param("publish/exteventinsnavcart", settings_.publish_exteventinsnavcart, true);
    param("publish/extsensormeas", settings_.publish_extsensormeas, true);
	
    // To be implemented: RTCM, raw data settings, PPP, SBAS ...
    this->log(LogLevel::DEBUG ,"Finished getROSParams() method");
};

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
    ros::spin();
    
    return 0;
}