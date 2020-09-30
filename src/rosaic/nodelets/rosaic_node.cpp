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
	connected_ = false;
	GetROSParams();
	StringValues_Initialize();
	
	// Initializes Connection
	ROS_DEBUG("About to call InitializeIO() method");
	InitializeIO();
	
	// Subscribes to all requested mosaic messages by adding entries to the C++ multimap storing the callback handlers and publishes ROS messages
    DefineMessages();
	
	// Sends commands to mosaic regarding which SBF/NMEA messages it should output and sets all its necessary corrections-related parameters
	boost::mutex::scoped_lock lock(connection_mutex_);
	connection_condition_.wait(lock, [this](){return connected_;});
	ConfigureMosaic();
	
	// Since we already have a ros::Spin() elsewhere, we use waitForShutdown() here
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
	
	// It is imperative to hold a lock on the mutex "response_mutex" while modifying the variable "response_received". Same for "cd_mutex" and "cd_received".
	boost::mutex::scoped_lock lock(response_mutex);
	boost::mutex::scoped_lock lock_cd(cd_mutex);
	
	IO.Send("\x0DSSSSSSSSSSSSSSSSSSS\x0D\x0D"); // Escape sequence (escape from correction mode), ensuring that we can send our real commands afterwards...
	// We have to wait for the connection descriptor before we send another command, otherwise the latter would not be processed.
	cd_condition.wait(lock_cd, [](){return cd_received;}); 
	cd_received = false;
	
	// Authentication, leaving anonymous mode
	IO.Send("login, Tibor, Tibor \x0D");
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
	uint32_t mosaic_period_pvt = parsing_utilities::UserPeriodToMosaicPeriod(polling_period_pvt_);
	uint32_t mosaic_period_rest = parsing_utilities::UserPeriodToMosaicPeriod(polling_period_rest_);
	std::string pvt_sec_or_msec;
	std::string rest_sec_or_msec;
	if (polling_period_pvt_ == 1000 || polling_period_pvt_ == 2000 || polling_period_pvt_ == 5000 || polling_period_pvt_ == 10000) pvt_sec_or_msec = "sec";
	else pvt_sec_or_msec = "msec";
	if (polling_period_rest_ == 1000 || polling_period_rest_ == 2000 || polling_period_rest_ == 5000 || polling_period_rest_ == 10000) rest_sec_or_msec = "sec";
	else rest_sec_or_msec = "msec";
	
	
	if (publish_gpgga_ == true)
	{
		std::stringstream ss;
		
		ss << "sno, Stream" << std::to_string(stream) << ", " << mosaic_port << ", GGA, " << pvt_sec_or_msec << std::to_string(mosaic_period_pvt) << "\x0D"; 
		IO.Send(ss.str());
		++stream;
		response_condition.wait(lock, [](){return response_received;});
		response_received = false;
	}
	if (publish_gprmc_ == true)
	{
		std::stringstream ss;
		
		ss << "sno, Stream" << std::to_string(stream) << ", " << mosaic_port << ", RMC, " << pvt_sec_or_msec << std::to_string(mosaic_period_pvt) << "\x0D"; 
		IO.Send(ss.str());
		++stream;
		response_condition.wait(lock, [](){return response_received;});
		response_received = false;
	}
	if (publish_gpgsa_ == true)
	{
		std::stringstream ss;
		
		ss << "sno, Stream" << std::to_string(stream) << ", " << mosaic_port << ", GSA, " << pvt_sec_or_msec << std::to_string(mosaic_period_pvt) << "\x0D"; 
		IO.Send(ss.str());
		++stream;
		response_condition.wait(lock, [](){return response_received;});
		response_received = false;
	}
	if (publish_gpgsv_ == true)
	{
		std::stringstream ss;
		
		ss << "sno, Stream" << std::to_string(stream) << ", " << mosaic_port << ", GSV, " << rest_sec_or_msec << std::to_string(mosaic_period_rest) << "\x0D"; 
		IO.Send(ss.str());
		++stream;
		response_condition.wait(lock, [](){return response_received;});
		response_received = false;
	}
	if (publish_pvtcartesian_ == true)
	{
		std::stringstream ss;
		ss << "sso, Stream" << std::to_string(stream) << ", " << mosaic_port << ", PVTCartesian, " << pvt_sec_or_msec << std::to_string(mosaic_period_pvt) << "\x0D";
		IO.Send(ss.str());
		++stream;
		response_condition.wait(lock, [](){return response_received;});
		response_received = false;
	}
	if (publish_pvtgeodetic_ == true)
	{
		std::stringstream ss;
		ss << "sso, Stream" << std::to_string(stream) << ", " << mosaic_port << ", PVTGeodetic, " << pvt_sec_or_msec << std::to_string(mosaic_period_pvt) << "\x0D";
		IO.Send(ss.str());
		++stream;
		response_condition.wait(lock, [](){return response_received;});
		response_received = false;
	}
	if (publish_poscovcartesian_ == true)
	{
		std::stringstream ss;
		ss << "sso, Stream" << std::to_string(stream) << ", " << mosaic_port << ", PosCovCartesian, " << pvt_sec_or_msec << std::to_string(mosaic_period_pvt) << "\x0D";
		IO.Send(ss.str());
		++stream;
		response_condition.wait(lock, [](){return response_received;});
		response_received = false;
	}
	if (publish_poscovgeodetic_ == true)
	{
		std::stringstream ss;
		ss << "sso, Stream" << std::to_string(stream) << ", " << mosaic_port << ", PosCovGeodetic, " << pvt_sec_or_msec << std::to_string(mosaic_period_pvt) << "\x0D";
		IO.Send(ss.str());
		++stream;
		response_condition.wait(lock, [](){return response_received;});
		response_received = false;
	}
	if (publish_atteuler_ == true)
	{
		std::stringstream ss;
		ss << "sso, Stream" << std::to_string(stream) << ", " << mosaic_port << ", AttEuler, " << rest_sec_or_msec << std::to_string(mosaic_period_rest) << "\x0D";
		IO.Send(ss.str());
		++stream;
		response_condition.wait(lock, [](){return response_received;});
		response_received = false;
	}
	if (publish_attcoveuler_ == true)
	{
		std::stringstream ss;
		ss << "sso, Stream" << std::to_string(stream) << ", " << mosaic_port << ", AttCovEuler, " << rest_sec_or_msec << std::to_string(mosaic_period_rest) << "\x0D";
		IO.Send(ss.str());
		++stream;
		response_condition.wait(lock, [](){return response_received;});
		response_received = false;
	}
	if (publish_gpsfix == true)
	{
		std::stringstream ss;
		ss << "sso, Stream" << std::to_string(stream) << ", " << mosaic_port << ", ChannelStatus, " << rest_sec_or_msec << std::to_string(mosaic_period_rest) << "\x0D";
		IO.Send(ss.str());
		response_condition.wait(lock, [](){return response_received;});
		response_received = false;
		++stream;
		ss.str(std::string()); // avoids invoking the std::string constructor
		ss << "sso, Stream" << std::to_string(stream) << ", " << mosaic_port << ", MeasEpoch, " << rest_sec_or_msec << std::to_string(mosaic_period_rest) << "\x0D";
		IO.Send(ss.str());
		response_condition.wait(lock, [](){return response_received;});
		response_received = false;
		++stream;
		ss.str(std::string()); 
		ss << "sso, Stream" << std::to_string(stream) << ", " << mosaic_port << ", DOP, " << rest_sec_or_msec << std::to_string(mosaic_period_rest) << "\x0D";
		IO.Send(ss.str());
		response_condition.wait(lock, [](){return response_received;});
		response_received = false;
		++stream;
		ss.str(std::string()); 
		ss << "sso, Stream" << std::to_string(stream) << ", " << mosaic_port << ", VelCovGeodetic, " << rest_sec_or_msec << std::to_string(mosaic_period_rest) << "\x0D";
		IO.Send(ss.str());
		++stream;
	}
	
	// Configuring the NTRIP connection
	// First disable any existing NTRIP connection on NTR1
	{
		std::stringstream ss;
		ss << "snts, NTR1, off \x0D"; 
		IO.Send(ss.str());
	}
	response_condition.wait(lock, [](){return response_received;});
	response_received = false;
	if (mosaic_has_internet_)
	{
		if (mode_ == "off")
		{
		}
		else if (mode_ == "Client")
		{
			{
				std::stringstream ss;
				ss << "snts, NTR1, " << mode_ << ", " << caster_ << ", " << std::to_string(caster_port_) << ", " << username_ << ", " << password_ << ", " << mountpoint_ << ", " << ntrip_version_ << ", " << send_gga_ << " \x0D"; 
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
	}
	else // Since mosaic does not have internet (and you will not be able to share it via USB), we need to forward the corrections ourselves, though not on the same port.
	{
		if (proto == "tcp")
		{
			{
				std::stringstream ss;
				ss << "siss, IPS1, " << std::to_string(mosaic_input_corrections_tcp_) << ", TCP2Way \x0D"; // We assume that IPS1 is free.
				IO.Send(ss.str());
			}
			response_condition.wait(lock, [](){return response_received;});
			response_received = false;
			{
				std::stringstream ss;
				ss << "sno, Stream" << std::to_string(stream) << ", IPS1, GGA, " << pvt_sec_or_msec << std::to_string(mosaic_period_pvt) << " \x0D"; // We assume that IPS1 is free.
				++stream;
				IO.Send(ss.str());
			}
			response_condition.wait(lock, [](){return response_received;});
			response_received = false;
		}
		{
			std::stringstream ss;
			if (proto == "tcp")
			{
				ss << "sdio, IPS1, " << rtcm_version_ << ", +SBF+NMEA \x0D"; 
			}
			else
			{
				ss << "sdio, " << mosaic_input_corrections_serial_ << ", " << rtcm_version_ << ", +SBF+NMEA \x0D"; 
			}
			IO.Send(ss.str());
		}
	}
	ROS_DEBUG("Finished ConfigureMosaic() method");
}
void rosaic_node::ROSaicNode::GetROSParams() 
{
	// Communication parameters
	nh->param("device", device_, std::string("/dev/ttyACM0"));
	GetROSInt("serial/baudrate", baudrate_, static_cast<uint32_t>(115200));
	nh->param("serial/hw_flow_control", hw_flow_control_, std::string("off"));
	nh->param("serial/mosaic_serial_port", mosaic_serial_port_, std::string("USB1"));
	
	nh->param("reconnect_delay_s", reconnect_delay_s_, 4.0f);
	
	// Polling period parameters
	GetROSInt("polling_period/pvt", polling_period_pvt_, static_cast<uint32_t>(1000));
	if (polling_period_pvt_ != 10 && polling_period_pvt_ != 20 && polling_period_pvt_ != 50 && polling_period_pvt_ != 100 
	&& polling_period_pvt_ != 200 && polling_period_pvt_ != 250 && polling_period_pvt_ != 500 && polling_period_pvt_ != 1000 
	&& polling_period_pvt_ != 2000 && polling_period_pvt_ != 5000 && polling_period_pvt_ != 10000)
	{
		ROS_ERROR("Please specify a valid polling period for PVT-related SBF blocks.");
	}
	GetROSInt("polling_period/rest", polling_period_rest_, static_cast<uint32_t>(1000));
	if (polling_period_rest_ != 10 && polling_period_rest_ != 20 && polling_period_rest_ != 50 && polling_period_rest_ != 100 
	&& polling_period_rest_ != 200 && polling_period_rest_ != 250 && polling_period_rest_ != 500 && polling_period_rest_ != 1000 
	&& polling_period_rest_ != 2000 && polling_period_rest_ != 5000 && polling_period_rest_ != 10000)
	{
		ROS_ERROR("Please specify a valid polling period for PVT-unrelated SBF blocks and NMEA messages.");
	}
	
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
	GetROSInt("ntrip_settings/caster_port", caster_port_, static_cast<uint32_t>(0));
	nh->param("ntrip_settings/username", username_, std::string());
	nh->param("ntrip_settings/password", password_, std::string());
	nh->param("ntrip_settings/mountpoint", mountpoint_, std::string());
	nh->param("ntrip_settings/ntrip_version", ntrip_version_, std::string("v2"));
	nh->param("ntrip_settings/send_gga", send_gga_, std::string("auto"));
	nh->param("ntrip_settings/mosaic_has_internet", mosaic_has_internet_, false);
	nh->param("ntrip_settings/rtcm_version", rtcm_version_, std::string("RTCMv3"));
	GetROSInt("ntrip_settings/mosaic_input_corrections_tcp", mosaic_input_corrections_tcp_, static_cast<uint32_t>(28785));
	nh->param("ntrip_settings/mosaic_input_corrections_serial", mosaic_input_corrections_serial_, std::string("USB2"));
	
	// Publising parameters, the others remained global since they need to be accessed in the callbackhandlers.hpp file
	rosaic_node::nh->param("publish/gpgga", publish_gpgga_, true);
	rosaic_node::nh->param("publish/gprmc", publish_gprmc_, true);
	rosaic_node::nh->param("publish/gpgsa", publish_gpgsa_, true);
	rosaic_node::nh->param("publish/gpgsv", publish_gpgsv_, true);
	rosaic_node::nh->param("publish/pvtcartesian", publish_pvtcartesian_, true);
	rosaic_node::nh->param("publish/pvtgeodetic", publish_pvtgeodetic_, true);
	rosaic_node::nh->param("publish/poscovcartesian", publish_poscovcartesian_, true);
	rosaic_node::nh->param("publish/poscovgeodetic", publish_poscovgeodetic_, true);
	rosaic_node::nh->param("publish/atteuler", publish_atteuler_, true);
	rosaic_node::nh->param("publish/attcoveuler", publish_attcoveuler_, true);

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
	ROS_DEBUG("Started ROS timer for calling Reconnect() method until connection succeds"); 
	ros::spin();
	ROS_DEBUG("Leaving Connect() method"); // This will never be output since ros::spin() is on the line above.
}

//! In serial mode (not USB, since mosaic port is then called USB1 or USB2), please ensure that you are 
//! connected to mosaic's COM1, COM2 or COM3 port, !if! you employ UART hardware flow control.
void rosaic_node::ROSaicNode::Reconnect(const ros::TimerEvent& event) 
{
	ROS_DEBUG("Called Reconnect() method");
	if (connected_ == true)
	{
		reconnect_timer_.stop();
		ROS_DEBUG("Stopped ROS timer since successully connected.");
	}
	else
	{
		if (serial_)
		{
			bool initialize_serial_return = false;
			try
			{
				ROS_INFO("Connecting serially to device %s, targeted baudrate: %u", device_.c_str(), baudrate_);
				initialize_serial_return = IO.InitializeSerial(device_, baudrate_, hw_flow_control_);
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
				connected_ = true;
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
				connected_ = true;
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

	if (publish_gpgga_ == true)
	{
		IO.handlers_.callbackmap_ = IO.GetHandlers().Insert<rosaic::Gpgga>("$GPGGA", boost::bind(Publish<rosaic::Gpgga>, _1, "/gpgga"));
		//std::multimap<std::string, boost::shared_ptr<io_comm_mosaic::AbstractCallbackHandler> >::key_type key = "$GPGGA";
		//ROS_DEBUG("Back to DefineMessages() method: The element exists in our map: %u", (unsigned int) IO.handlers_.callbackmap_.count(key));
	}
	if (publish_gprmc_ == true)
	{
		IO.handlers_.callbackmap_ = IO.GetHandlers().Insert<rosaic::Gprmc>("$GPRMC", boost::bind(Publish<rosaic::Gprmc>, _1, "/gprmc"));
	}
	if (publish_gpgsa_ == true)
	{
		IO.handlers_.callbackmap_ = IO.GetHandlers().Insert<rosaic::Gpgsa>("$GPGSA", boost::bind(Publish<rosaic::Gpgsa>, _1, "/gpgsa"));
	}
	if (publish_gpgsv_ == true)
	{
		IO.handlers_.callbackmap_ = IO.GetHandlers().Insert<rosaic::Gpgsv>("$GPGSV", boost::bind(Publish<rosaic::Gpgsv>, _1, "/gpgsv"));
		IO.handlers_.callbackmap_ = IO.GetHandlers().Insert<rosaic::Gpgsv>("$GLGSV", boost::bind(Publish<rosaic::Gpgsv>, _1, "/gpgsv"));
		IO.handlers_.callbackmap_ = IO.GetHandlers().Insert<rosaic::Gpgsv>("$GAGSV", boost::bind(Publish<rosaic::Gpgsv>, _1, "/gpgsv"));
		IO.handlers_.callbackmap_ = IO.GetHandlers().Insert<rosaic::Gpgsv>("$GBGSV", boost::bind(Publish<rosaic::Gpgsv>, _1, "/gpgsv"));
	}
	if (publish_pvtcartesian_ == true)
	{
		IO.handlers_.callbackmap_ = IO.GetHandlers().Insert<rosaic::PVTCartesian>("4006", boost::bind(Publish<rosaic::PVTCartesian>, _1, "/pvtcartesian"));
	}
	if (publish_pvtgeodetic_ == true)
	{
		IO.handlers_.callbackmap_ = IO.GetHandlers().Insert<rosaic::PVTGeodetic>("4007", boost::bind(Publish<rosaic::PVTGeodetic>, _1, "/pvtgeodetic"));
	}
	if (publish_poscovcartesian_ == true)
	{
		IO.handlers_.callbackmap_ = IO.GetHandlers().Insert<rosaic::PosCovCartesian>("5905", boost::bind(Publish<rosaic::PosCovCartesian>, _1, "/poscovcartesian"));
	}
	if (publish_poscovgeodetic_ == true)
	{
		IO.handlers_.callbackmap_ = IO.GetHandlers().Insert<rosaic::PosCovGeodetic>("5906", boost::bind(Publish<rosaic::PosCovGeodetic>, _1, "/poscovgeodetic"));
	}
	if (publish_atteuler_ == true)
	{
		IO.handlers_.callbackmap_ = IO.GetHandlers().Insert<rosaic::AttEuler>("5938", boost::bind(Publish<rosaic::AttEuler>, _1, "/atteuler"));
	}
	if (publish_attcoveuler_ == true)
	{
		IO.handlers_.callbackmap_ = IO.GetHandlers().Insert<rosaic::AttCovEuler>("5939", boost::bind(Publish<rosaic::AttCovEuler>, _1, "/attcoveuler"));
	}
	if (publish_gpst == true)
	{
		IO.handlers_.callbackmap_ = IO.GetHandlers().Insert<sensor_msgs::TimeReference>("GPST", boost::bind(Publish<sensor_msgs::TimeReference>, _1, "/gpst"));
	}
	if (publish_navsatfix == true)
	{
		if (publish_pvtgeodetic_ == false || publish_poscovgeodetic_ == false)
		{
			ROS_ERROR("For a proper NavSatFix message, please set the publish/pvtgeodetic and the publish/poscovgeodetic ROSaic parameters both to true.");
		}
		IO.handlers_.callbackmap_ = IO.GetHandlers().Insert<sensor_msgs::NavSatFix>("NavSatFix", boost::bind(Publish<sensor_msgs::NavSatFix>, _1, "/navsatfix"));
	}
	if (publish_gpsfix == true)
	{
		if (publish_pvtgeodetic_ == false || publish_poscovgeodetic_ == false)
		{
			ROS_ERROR("For a proper GPSFix message, please set the publish/pvtgeodetic and the publish/poscovgeodetic ROSaic parameters both to true.");
		}
		typedef boost::function<void(int32_t)> Empty; 
		IO.handlers_.callbackmap_ = IO.GetHandlers().Insert<gps_common::GPSFix>("GPSFix", boost::bind(Publish<gps_common::GPSFix>, _1, "/gpsfix"));
		// The following blocks are never published, yet are needed for the construction of the GPSFix message, hence we have empty callbacks.
		IO.handlers_.callbackmap_ = IO.GetHandlers().Insert<int32_t>("4013", Empty()); // ChannelStatus block
		IO.handlers_.callbackmap_ = IO.GetHandlers().Insert<int32_t>("4027", Empty()); // MeasEpoch block
		IO.handlers_.callbackmap_ = IO.GetHandlers().Insert<int32_t>("4001", Empty()); // DOP block
		IO.handlers_.callbackmap_ = IO.GetHandlers().Insert<int32_t>("5908", Empty()); // VelCovGeodetic block
	}
	if (publish_posewithcovariancestamped == true)
	{
		if (publish_pvtgeodetic_ == false || publish_poscovgeodetic_ == false || publish_atteuler_ == false || publish_attcoveuler_ == false)
		{
			ROS_ERROR("For a proper PoseWithCovarianceStamped message, please set the publish/pvtgeodetic, publish/poscovgeodetic, publish_atteuler and publish_attcoveuler ROSaic parameters all to true.");
		}
		IO.handlers_.callbackmap_ = IO.GetHandlers().Insert<geometry_msgs::PoseWithCovarianceStamped>("PoseWithCovarianceStamped", boost::bind(Publish<geometry_msgs::PoseWithCovarianceStamped>, _1, "/posewithcovariancestamped"));
	}
	// so on and so forth...
	ROS_DEBUG("Leaving DefineMessages() method");
}


//! If true, the ROS message headers' unix time field is constructed from the TOW (in the SBF case) and UTC (in the NMEA case) data. 
//! If false, times are constructed within the driver via time(NULL) of the <ctime> library.
bool use_GNSS_time;
//! Whether or not to publish the sensor_msgs::TimeReference message with GPST
bool publish_gpst;
//! Whether or not to publish the sensor_msgs::NavSatFix message
bool publish_navsatfix;
//! Whether or not to publish the gps_common::GPSFix message
bool publish_gpsfix;
//! Whether or not to publish the geometry_msgs::PoseWithCovarianceStamped message
bool publish_posewithcovariancestamped;
//! The frame ID used in the header of every published ROS message
std::string frame_id;
//! The number of leap seconds that have been inserted into the UTC time
uint32_t leap_seconds;
//! The get_handlers() method of the Comm_IO class forces us to make this mutex static, since otherwise, 
//! the method would need to construct-by-copy the mutex, which is strictly prohibited.
boost::mutex io_comm_mosaic::CallbackHandlers::callback_mutex_;
//! Mutex to control changes of global variable "response_received"
boost::mutex response_mutex;
//! Determines whether a command reply was received from mosaic
bool response_received;
//! Condition variable complementing "response_mutex"
boost::condition_variable response_condition;
//! Mutex to control changes of global variable "cd_received"
boost::mutex cd_mutex;
//! Determines whether the connection descriptor was received from mosaic
bool cd_received;
//! Condition variable complementing "cd_mutex"
boost::condition_variable cd_condition;
//! Whether or not we still want to read the connection descriptor, which we only want in the very beginning to know whether it is IP10, IP11 etc.
bool read_cd;
//! Mosaic TCP port, e.g. IP10 or IP11, to which ROSaic is connected to
std::string mosaic_tcp_port;
//! Since after SSSSSSSSSSS we need to wait for second connection descriptor, we have to count the connection descriptors
uint32_t cd_count;

int main(int argc, char** argv) 
{
	ros::init(argc, argv, "mosaic_gnss");
	rosaic_node::nh.reset(new ros::NodeHandle("~")); // Note that nh was initialized in the header file already.
	rosaic_node::nh->param("use_GNSS_time", use_GNSS_time, true);
	rosaic_node::nh->param("frame_id", frame_id, (std::string) "gnss"); 
	rosaic_node::nh->param("publish/gpst", publish_gpst, true);
	rosaic_node::nh->param("publish/navsatfix", publish_navsatfix, true);
	rosaic_node::nh->param("publish/gpsfix", publish_gpsfix, true);
	rosaic_node::nh->param("publish/posewithcovariancestamped", publish_posewithcovariancestamped, true);
	rosaic_node::GetROSInt("leap_seconds", leap_seconds, static_cast<uint32_t>(18));
	
	response_received = false;
	cd_received = false;
	read_cd = true;
	cd_count = 0;
	
	// The info logging level seems to be default, hence we modify log level momentarily..
	// The following is the C++ version of rospy.init_node('my_ros_node', log_level=rospy.DEBUG)
	if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
								   ros::console::levels::Debug)) //debug is lowest level, shows everything
	ros::console::notifyLoggerLevelsChanged();

	ROS_DEBUG("Calling ROSaicNode constructor");
	rosaic_node::ROSaicNode mosaic_node; // This launches everything we need, in theory :)
	ROS_DEBUG("Leaving int main.");
	return 0;
}
