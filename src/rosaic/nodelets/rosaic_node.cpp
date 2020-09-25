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

#include <rosaic/nodelets/rosaic_node.hpp>

/**
 * @file rosaic_node.cpp
 * @date 22/08/20
 * @brief The heart of the ROSaic driver: The ROS node that represents it
 */
 
rosaic_node::ROSaicNode::ROSaicNode()
{
	ROS_DEBUG("Called ROSaicNode() constructor..");
	
	// Parameters must be set before initializing IO
	GetROSParams();
	StringValues_Initialize();
	ROS_DEBUG("About to call InitializeIO() method");
	InitializeIO();
	// Subscribes to all requested mosaic messages and publish them raw or in composite form (e.g. NavSatFix)
    DefineMessages();
	// Communicates to mosaic which SBF/NMEA messages it should output and sets all its necessary corrections-related parameters
	boost::mutex::scoped_lock lock(connection_mutex_);
	connection_condition_.wait(lock, [](){return connected;});
	ConfigureMosaic();
	
	ros::waitForShutdown();	
}

//! The Send() method of AsyncManager class is paramount for this purpose.
//! Note that std::to_string() is from C++11 onwards only.
//! Since this driver can be launched before booting mosaic, we have to watch out for escape characters that are sent 
//! by mosaic to indicate that it is in upgrade mode. Those characters would then be mingled with the first command 
//! we send to it in this method and could result in an invalid command. Hence the first command we send to mosaic is an unneccesary "grc".
void rosaic_node::ROSaicNode::ConfigureMosaic()
{
	ROS_DEBUG("Called ConfigureMosaic() method");
	
	// It is imperative to hold a lock on the mutex "response_mutex" while modifying the variable "response_received".
	boost::mutex::scoped_lock lock(response_mutex);
	
	IO.Send("grc \x0D");
	response_condition.wait(lock, [](){return response_received;});
	response_received = false;
	
	// Turning off all current SBF/NMEA output 
	IO.Send("sso, all, none, none, off \x0D");
	response_condition.wait(lock, [](){return response_received;});
	response_received = false;
	IO.Send("sno, all, none, none, off \x0D");
	response_condition.wait(lock, [](){return response_received;});
	response_received = false;
	
	// Setting datum to be used by mosaic (not mosaic's NMEA output though, which only provides MSL and undulation (by default with respect to WGS84), but not ellipsoidal height)
	{
		std::stringstream ss;
		ss << "sgd, " << datum_ << "\x0D"; 
		IO.Send(ss.str());
	}
	response_condition.wait(lock, [](){return response_received;});
	response_received = false;
	
	// Setting the marker-to-ARP offsets
	{
		std::stringstream ss;
		ss << "sao, Main, " << string_utilities::TrimString(std::to_string(delta_e_)) << ", " << string_utilities::TrimString(std::to_string(delta_n_)) << ", " << string_utilities::TrimString(std::to_string(delta_u_)) << ", \"" << ant_type_ << "\", \"" << ant_serial_nr_ << "\", 0 \x0D"; 
		IO.Send(ss.str());
	}
	response_condition.wait(lock, [](){return response_received;});
	response_received = false;
	
	// Configuring the NTRIP connection
	// Disabling any existing NTRIP connection on NTR1
	{
		std::stringstream ss;
		ss << "snts, NTR1, off \x0D"; 
		IO.Send(ss.str());
	}
	response_condition.wait(lock, [](){return response_received;});
	response_received = false;
	if (mode_ == "off")
	{
	}
	else if (mode_ == "Client")
	{
		{
			std::stringstream ss;
			ss << "snts, NTR1, " << mode_ << ", " << caster_ << ", " << std::to_string(ntrip_port_) << ", " << username_ << ", " << password_ << ", " << mountpoint_ << ", " << version_ << ", " << send_gga_ << " \x0D"; 
			IO.Send(ss.str());
		}
		response_condition.wait(lock, [](){return response_received;});
		response_received = false;
	}
	else if (mode_ == "Client-Sapcorda")
	{
		{
			std::stringstream ss;
			ss << "snts, NTR1, Client-Sapcorda, , , , , , , , \x0D"; 
			IO.Send(ss.str());
		}
		response_condition.wait(lock, [](){return response_received;});
		response_received = false;
	}
	else 
	{
		ROS_ERROR("Invalid mode specified for NTRIP settings.");
	}
	
	// Setting SBF/NMEA output of mosaic
	unsigned stream = 1;
	boost::smatch match;
	boost::regex_match(device_, match, boost::regex("(tcp|udp)://(.+):(\\d+)"));
	std::string proto(match[1]);
	std::string mosaic_port;
	if (proto == "tcp") 
	{
		mosaic_port = mosaic_tcp_port;
	}
	else
	{
		mosaic_port = mosaic_serial_port_;
	}
	if (publish_gpgga == true)
	{
		std::stringstream ss;
		ss << "sno, Stream" << std::to_string(stream) << ", " << mosaic_port << ", GGA, sec" << std::to_string(polling_period_rest_) << "\x0D"; 
		IO.Send(ss.str());
		++stream;
		response_condition.wait(lock, [](){return response_received;});
		response_received = false;
	}
	if (publish_pvtcartesian == true)
	{
		std::stringstream ss;
		ss << "sso, Stream" << std::to_string(stream) << ", " << mosaic_port << ", PVTCartesian, sec" << std::to_string(polling_period_pvt_) << "\x0D";
		IO.Send(ss.str());
		++stream;
		response_condition.wait(lock, [](){return response_received;});
		response_received = false;
	}
	if (publish_pvtgeodetic == true)
	{
		std::stringstream ss;
		ss << "sso, Stream" << std::to_string(stream) << ", " << mosaic_port << ", PVTGeodetic, sec" << std::to_string(polling_period_pvt_) << "\x0D";
		IO.Send(ss.str());
		++stream;
		response_condition.wait(lock, [](){return response_received;});
		response_received = false;
	}
	if (publish_poscovgeodetic == true)
	{
		std::stringstream ss;
		ss << "sso, Stream" << std::to_string(stream) << ", " << mosaic_port << ", PosCovGeodetic, sec" << std::to_string(polling_period_pvt_) << "\x0D";
		IO.Send(ss.str());
		++stream;
		response_condition.wait(lock, [](){return response_received;});
		response_received = false;
	}
	if (publish_atteuler == true)
	{
		std::stringstream ss;
		ss << "sso, Stream" << std::to_string(stream) << ", " << mosaic_port << ", AttEuler, sec" << std::to_string(polling_period_rest_) << "\x0D";
		IO.Send(ss.str());
		++stream;
		response_condition.wait(lock, [](){return response_received;});
		response_received = false;
	}
	if (publish_attcoveuler == true)
	{
		std::stringstream ss;
		ss << "sso, Stream" << std::to_string(stream) << ", " << mosaic_port << ", AttCovEuler, sec" << std::to_string(polling_period_rest_) << "\x0D";
		IO.Send(ss.str());
		++stream;
		response_condition.wait(lock, [](){return response_received;});
		response_received = false;
	}
	if (publish_gpsfix == true)
	{
		std::stringstream ss;
		ss << "sso, Stream" << std::to_string(stream) << ", " << mosaic_port << ", ChannelStatus, sec" << std::to_string(polling_period_rest_) << "\x0D";
		IO.Send(ss.str());
		response_condition.wait(lock, [](){return response_received;});
		response_received = false;
		++stream;
		ss.str(std::string()); // avoids invoking the std::string constructor
		ss << "sso, Stream" << std::to_string(stream) << ", " << mosaic_port << ", MeasEpoch, sec" << std::to_string(polling_period_rest_) << "\x0D";
		IO.Send(ss.str());
		response_condition.wait(lock, [](){return response_received;});
		response_received = false;
		++stream;
		ss.str(std::string()); 
		ss << "sso, Stream" << std::to_string(stream) << ", " << mosaic_port << ", DOP, sec" << std::to_string(polling_period_rest_) << "\x0D";
		IO.Send(ss.str());
		response_condition.wait(lock, [](){return response_received;});
		response_received = false;
		++stream;
		ss.str(std::string()); 
		ss << "sso, Stream" << std::to_string(stream) << ", " << mosaic_port << ", VelCovGeodetic, sec" << std::to_string(polling_period_rest_) << "\x0D";
		IO.Send(ss.str());
		++stream;
	}
	ROS_DEBUG("Finished ConfigureMosaic() method");
}
void rosaic_node::ROSaicNode::GetROSParams() 
{
	// Communication parameters
	nh->param("device", device_, std::string("/dev/ttyACM0"));
	GetROSInt("serial/baudrate", baudrate_, static_cast<uint32_t>(115200));
	nh->param("reconnect_delay_s", reconnect_delay_s_, 4.0f);
	nh->param("serial/mosaic_serial_port", mosaic_serial_port_, std::string("USB1"));
	
	// Polling period parameters
	GetROSInt("polling_period/pvt", polling_period_pvt_, static_cast<unsigned>(1));
	GetROSInt("polling_period/rest", polling_period_rest_, static_cast<unsigned>(1));
	
	// Datum and marker-to-ARP offset
	nh->param("datum", datum_, std::string("ETRS89"));
	nh->param("ant_type", ant_type_, std::string("Unknown"));
	nh->param("ant_serial_nr", ant_serial_nr_, std::string("Unknown"));
	nh->param("marker_to_arp/delta_e", delta_e_, 0.0f);
	nh->param("marker_to_arp/delta_n", delta_n_, 0.0f);
	nh->param("marker_to_arp/delta_u", delta_u_, 0.0f);
	
	// Correction service parameters
	nh->param("ntrip_settings/mode", mode_, std::string("off"));
	nh->param("ntrip_settings/caster", caster_, std::string());
	GetROSInt("ntrip_settings/port", ntrip_port_, static_cast<uint32_t>(0));
	nh->param("ntrip_settings/username", username_, std::string());
	nh->param("ntrip_settings/password", password_, std::string());
	nh->param("ntrip_settings/mountpoint", mountpoint_, std::string());
	nh->param("ntrip_settings/version", version_, std::string("v2"));
	nh->param("ntrip_settings/send_gga", send_gga_, std::string("auto"));

	// To be implemented: RTCM, setting datum, raw data settings, PPP, SBAS, fix mode...
	ROS_DEBUG("Finished GetROSParams() method");

}

void rosaic_node::ROSaicNode::InitializeIO() 
{
	ROS_DEBUG("Called InitializeIO() method");
	boost::smatch match;
	// In fact: smatch is a typedef of match_results<string::const_iterator>
	if (boost::regex_match(device_, match, boost::regex("(tcp)://(.+):(\\d+)"))) 
	// \d means decimal, however, in the regular expression, the \ is a special character, which needs to be escaped on its own as well..
	// Note that regex_match can be used with a smatch object to store results, or without. In any case, true is returned if and only if it matches the !complete! string.
	{
		// The first sub_match (index 0) contained in a match_result always represents the full match within a target sequence made by a regex, 
		// and subsequent sub_matches represent sub-expression matches corresponding in sequence to the left parenthesis delimiting the sub-expression in the regex,
		// i.e. $n Perl is equivalent to m[n] in boost regex.
		std::string proto(match[1]);
		if (proto == "tcp") 
		{
			tcp_host_ = match[2];
			tcp_port_ = match[3];
			
			serial_ = false;
			boost::thread temporary_thread(boost::bind(&ROSaicNode::Connect, this));
			temporary_thread.detach();
		} 
		else 
		{
			{
				std::stringstream ss;
				ss << "Protocol '" << proto << "' is unsupported.";
				ROS_ERROR("%s", ss.str().c_str());
			}
		}
	} 
	else 
	{
		serial_ = true;
		boost::thread temporary_thread(boost::bind(&ROSaicNode::Connect, this));
		temporary_thread.detach();
	}
	ROS_DEBUG("Leaving InitializeIO() method");
}


		
void rosaic_node::ROSaicNode::Connect() 
{
	ROS_DEBUG("Called Connect() method");
	ROS_DEBUG("Setting ROS timer for calling Reconnect() method until connection succeds");
	reconnect_timer_ = nh->createTimer(ros::Duration(reconnect_delay_s_), &ROSaicNode::Reconnect, this);
	reconnect_timer_.start();
	ROS_DEBUG("Started ROS timer for calling Reconnect() method until connection succeds"); //not printed if error in callback of course
	ros::spin(); // otherwise callback will never be called, with ros::spin i cannot leave InitializeIO(), yet with ros::spinOnce cannot enter reconnect() even once
	ROS_DEBUG("Leaving Connect() method"); // will never be called
}

//! Please ensure that the USB is connected to mosaic's COM1, COM2 or COM3 port, since we employ UART hardware flow control.
void rosaic_node::ROSaicNode::Reconnect(const ros::TimerEvent& event) 
{
	ROS_DEBUG("Called Reconnect() method");
	if (connected == true)
	{
		reconnect_timer_.stop();
		ROS_DEBUG("Ended timer");
	}
	else
	{
		if (serial_)
		{
			bool initialize_serial_return = false;
			try
			{
				ROS_INFO("Connecting serially to device %s, targeted baudrate: %u", device_.c_str(), baudrate_);
				initialize_serial_return = IO.InitializeSerial(device_, baudrate_, "RTS|CTS");
			}
			catch (std::runtime_error& e)
			{
				{
					std::stringstream ss;
					ss << "IO.InitializeSerial() failed for device " << device_ << " due to: " << e.what();
					ROS_ERROR("%s", ss.str().c_str());
				}
			}
			if (initialize_serial_return)
			{
				boost::mutex::scoped_lock lock(connection_mutex_);
				connected = true;
				lock.unlock();
				connection_condition_.notify_one();
			}
		}
		else
		{
			bool initialize_tcp_return = false;
			try
			{
				ROS_INFO("Connecting to tcp://%s:%s ...", tcp_host_.c_str(), tcp_port_.c_str());
				initialize_tcp_return = IO.InitializeTCP(tcp_host_, tcp_port_);
			}
			catch (std::runtime_error& e)
			{
				{
					std::stringstream ss;
					ss << "IO.InitializeTCP() failed for host " << tcp_host_ << " on port " << tcp_port_ << " due to: " << e.what();
					ROS_ERROR("%s", ss.str().c_str());
				}
			}
			if (initialize_tcp_return)
			{
				boost::mutex::scoped_lock lock(connection_mutex_);
				connected = true;
				lock.unlock();
				connection_condition_.notify_one();
			}
		}
	}
	ROS_DEBUG("Leaving Reconnect() method");
}

//! InitializeSerial is not self-contained: The for loop in Callbackhandlers' handle method would never open a specific handler unless 
//! the handler is added (=inserted) to the C++ map via this function. This way, the specific handler can be called, in which in turn 
//! mosaicMessage's read() method is called, thereby "message" occupied, and func_ (the handler we insert in this function) called with this "message".
void rosaic_node::ROSaicNode::DefineMessages() 
{
	ROS_DEBUG("Called DefineMessages() method");

	if (publish_gpgga == true)
	{
		IO.handlers_.callbackmap_ = IO.GetHandlers().Insert<rosaic::Gpgga>("$GPGGA", boost::bind(Publish<rosaic::Gpgga>, _1, "/gpgga"));
		//std::multimap<std::string, boost::shared_ptr<io_comm_mosaic::AbstractCallbackHandler> >::key_type key = "$GPGGA";
		//ROS_DEBUG("Back to DefineMessages() method: The element exists in our map: %u", (unsigned int) IO.handlers_.callbackmap_.count(key));
	}
	if (publish_pvtcartesian == true)
	{
		IO.handlers_.callbackmap_ = IO.GetHandlers().Insert<rosaic::PVTCartesian>("4006", boost::bind(Publish<rosaic::PVTCartesian>, _1, "/pvtcartesian"));
	}
	if (publish_pvtgeodetic == true)
	{
		IO.handlers_.callbackmap_ = IO.GetHandlers().Insert<rosaic::PVTGeodetic>("4007", boost::bind(Publish<rosaic::PVTGeodetic>, _1, "/pvtgeodetic"));
	}
	if (publish_poscovgeodetic == true)
	{
		IO.handlers_.callbackmap_ = IO.GetHandlers().Insert<rosaic::PosCovGeodetic>("5906", boost::bind(Publish<rosaic::PosCovGeodetic>, _1, "/poscovgeodetic"));
	}
	if (publish_atteuler == true)
	{
		IO.handlers_.callbackmap_ = IO.GetHandlers().Insert<rosaic::AttEuler>("5938", boost::bind(Publish<rosaic::AttEuler>, _1, "/atteuler"));
	}
	if (publish_attcoveuler == true)
	{
		IO.handlers_.callbackmap_ = IO.GetHandlers().Insert<rosaic::AttCovEuler>("5939", boost::bind(Publish<rosaic::AttCovEuler>, _1, "/attcoveuler"));
	}
	if (publish_navsatfix == true)
	{
		IO.handlers_.callbackmap_ = IO.GetHandlers().Insert<sensor_msgs::NavSatFix>("NavSatFix", boost::bind(Publish<sensor_msgs::NavSatFix>, _1, "/navsatfix"));
	}
	if (publish_gpsfix == true)
	{
		typedef boost::function<void(int32_t)> Empty; 
		IO.handlers_.callbackmap_ = IO.GetHandlers().Insert<gps_common::GPSFix>("GPSFix", boost::bind(Publish<gps_common::GPSFix>, _1, "/gpsfix"));
		// The following blocks are never published, yet are needed for the construction of the GPSFix message, hence we have empty callbacks.
		IO.handlers_.callbackmap_ = IO.GetHandlers().Insert<int32_t>("4013", Empty()); // ChannelStatus block
		IO.handlers_.callbackmap_ = IO.GetHandlers().Insert<int32_t>("4027", Empty()); // MeasEpoch block
		IO.handlers_.callbackmap_ = IO.GetHandlers().Insert<int32_t>("4001", Empty()); // DOP block
		IO.handlers_.callbackmap_ = IO.GetHandlers().Insert<int32_t>("5908", Empty()); // VelCovGeodetic block
	}
	
	ROS_DEBUG("Leaving DefineMessages() method");
	// and so on
}


// Declaring global variables (to be modified, since their use is not recommended)...
//! If true, the ROS message headers' unix time field is constructed from the TOW (in the SBF case) and UTC (in the NMEA case) data. 
//! If false, times are constructed within the driver via time(NULL) of the <ctime> library.
bool use_GNSS_time;
//! Driver debugging level (not the ROS logging level), as specified in rover.yaml or equivalent files
int io_comm_mosaic::debug; 
//! Whether or not to publish the GGA message
bool publish_gpgga;
//! Whether or not to publish the rosaic::PVTCartesian message
bool publish_pvtcartesian;
//! Whether or not to publish the rosaic::PVTGeodetic message
bool publish_pvtgeodetic;
//! Whether or not to publish the rosaic::PosCovGeodetic message
bool publish_poscovgeodetic;
//! Whether or not to publish the rosaic::AttEuler message
bool publish_atteuler;
//! Whether or not to publish the rosaic::AttCovEuler message
bool publish_attcoveuler;
//! Whether or not to publish the sensor_msgs::NavSatFix message
bool publish_navsatfix;
//! Whether or not to publish the gps_common::GPSFix message
bool publish_gpsfix;
//! The frame ID used in the header of every published ROS message
std::string frame_id;
//! The number of leap seconds that have been inserted into the UTC time
uint32_t leap_seconds;
//! The get_handlers() method of the Comm_IO class forces us to make this mutex static, since otherwise, 
//! the method would need to construct-by-copy the mutex, which is strictly prohibited.
boost::mutex io_comm_mosaic::CallbackHandlers::callback_mutex_;
boost::mutex response_mutex;
bool response_received;
boost::condition_variable response_condition;
bool connected;
//! Whether or not we still want to read the connection descriptor, which we only want in the very beginning to know whether it is IP10, IP11 etc.
bool read_cd;
//! Mosaic TCP port, e.g. IP10 or IP11, to which ROSaic is connected to
std::string mosaic_tcp_port;

int main(int argc, char** argv) 
{
	//rosaic_node::nh->param("something", node_name, default_node_name); 
	ros::init(argc, argv, "mosaic_gnss");
	rosaic_node::nh.reset(new ros::NodeHandle("~")); // Note that nh was initialized in the header file already.
	rosaic_node::GetROSInt("debug", io_comm_mosaic::debug, 1); 
	rosaic_node::nh->param("use_GNSS_time", use_GNSS_time, true);
	rosaic_node::nh->param("frame_id", frame_id, (std::string) "gnss"); 
	rosaic_node::nh->param("publish/gpgga", publish_gpgga, true);
	rosaic_node::nh->param("publish/pvtcartesian", publish_pvtcartesian, true);
	rosaic_node::nh->param("publish/pvtgeodetic", publish_pvtgeodetic, true);
	rosaic_node::nh->param("publish/poscovgeodetic", publish_poscovgeodetic, true);
	rosaic_node::nh->param("publish/atteuler", publish_atteuler, true);
	rosaic_node::nh->param("publish/attcoveuler", publish_attcoveuler, true);
	rosaic_node::nh->param("publish/navsatfix", publish_navsatfix, true);
	rosaic_node::nh->param("publish/gpsfix", publish_gpsfix, true);
	rosaic_node::GetROSInt("leap_seconds", leap_seconds, static_cast<uint32_t>(18));
	connected = false;
	read_cd = true;
	
	response_received = false;
	
	//To be implemented: Let nh subscribe to RTCM topic...

	if(io_comm_mosaic::debug) //yields true if >= 1
	{
		// The info logging level seems to be default, hence we modify log level momentarily..
		// The following is the C++ version of rospy.init_node('my_ros_node', log_level=rospy.DEBUG)
		if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                       ros::console::levels::Debug)) //debug is lowest level, shows everything
		ros::console::notifyLoggerLevelsChanged();

	}
	ROS_DEBUG("Calling ROSaicNode constructor");
	rosaic_node::ROSaicNode mosaic_node; // This launches everything we need, in theory :)
	ROS_DEBUG("Leaving int main.");
	return 0;
}
