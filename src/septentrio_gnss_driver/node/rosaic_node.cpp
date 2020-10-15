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
 
rosaic_node::ROSaicNode::ROSaicNode()
{
	ROS_DEBUG("Called ROSaicNode() constructor..");
	
	// Parameters must be set before initializing IO
	connected_ = false;
	getROSParams();
	
	// Initializes Connection
	initializeIO();
	
	// Subscribes to all requested Rx messages by adding entries to the C++ multimap 
	// storing the callback handlers and publishes ROS messages
    defineMessages();
	
	// Sends commands to the Rx regarding which SBF/NMEA messages it should output and sets all 
	// its necessary corrections-related parameters
	boost::mutex::scoped_lock lock(connection_mutex_);
	connection_condition_.wait(lock, [this](){return connected_;});
	configureRx();
	
	// Since we already have a ros::Spin() elsewhere, we use waitForShutdown() here
	ros::waitForShutdown();	
	ROS_DEBUG("Leaving ROSaicNode() constructor..");
}

//! The send() method of AsyncManager class is paramount for this purpose.
//! Note that std::to_string() is from C++11 onwards only.
//! Since ROSaic can be launched before booting the Rx, we have to watch out for escape characters that are sent 
//! by the Rx to indicate that it is in upgrade mode. Those characters would then be mingled with the first command 
//! we send to it in this method and could result in an invalid command. Hence we first enter command mode via "SSSSSSSSSS".
void rosaic_node::ROSaicNode::configureRx()
{
	ROS_DEBUG("Called configureRx() method");
	
	// It is imperative to hold a lock on the mutex "g_response_mutex" while modifying the variable 
	// "g_response_received". Same for "g_cd_mutex" and "g_cd_received".
	boost::mutex::scoped_lock lock(g_response_mutex);
	boost::mutex::scoped_lock lock_cd(g_cd_mutex);
	
	// Escape sequence (escape from correction mode), ensuring that we can send our real commands afterwards...
	IO.send("\x0DSSSSSSSSSSSSSSSSSSS\x0D\x0D"); 
	// We wait for the connection descriptor before we send another command, otherwise the latter would not be processed.
	g_cd_condition.wait(lock_cd, [](){return g_cd_received;}); 
	g_cd_received = false;
	
	// Turning off all current SBF/NMEA output 
	IO.send("sso, all, none, none, off \x0D");
	g_response_condition.wait(lock, [](){return g_response_received;});
	g_response_received = false;
	IO.send("sno, all, none, none, off \x0D");
	g_response_condition.wait(lock, [](){return g_response_received;});
	g_response_received = false;
	
	// Setting the datum to be used by the Rx (not the NMEA output though, which only provides MSL and undulation 
	// (by default with respect to WGS84), but not ellipsoidal height)
	{
		std::stringstream ss;
		ss << "sgd, " << datum_ << "\x0D"; 
		IO.send(ss.str());
	}
	g_response_condition.wait(lock, [](){return g_response_received;});
	g_response_received = false;
	
	// Setting SBF/NMEA output of Rx
	unsigned stream = 1;
	boost::smatch match;
	boost::regex_match(device_, match, boost::regex("(tcp|udp)://(.+):(\\d+)"));
	std::string proto(match[1]);
	std::string rx_port;
	if (proto == "tcp") 
	{
		rx_port = g_rx_tcp_port;
	}
	else
	{
		rx_port = rx_serial_port_;
	}
	uint32_t rx_period_pvt = parsing_utilities::convertUserPeriodToRxCommand(polling_period_pvt_);
	uint32_t rx_period_rest = parsing_utilities::convertUserPeriodToRxCommand(polling_period_rest_);
	std::string pvt_sec_or_msec;
	std::string rest_sec_or_msec;
	if (polling_period_pvt_ == 1000 || polling_period_pvt_ == 2000 || polling_period_pvt_ == 5000 || 
		polling_period_pvt_ == 10000) pvt_sec_or_msec = "sec";
	else pvt_sec_or_msec = "msec";
	if (polling_period_rest_ == 1000 || polling_period_rest_ == 2000 || polling_period_rest_ == 5000 || 
		polling_period_rest_ == 10000) rest_sec_or_msec = "sec";
	else rest_sec_or_msec = "msec";
	
	
	if (publish_gpgga_ == true)
	{
		std::stringstream ss;
		
		ss << "sno, Stream" << std::to_string(stream) << ", " << rx_port << ", GGA, " << pvt_sec_or_msec << 
			std::to_string(rx_period_pvt) << "\x0D"; 
		IO.send(ss.str());
		++stream;
		g_response_condition.wait(lock, [](){return g_response_received;});
		g_response_received = false;
	}
	if (publish_gprmc_ == true)
	{
		std::stringstream ss;
		
		ss << "sno, Stream" << std::to_string(stream) << ", " << rx_port << ", RMC, " << pvt_sec_or_msec << 
			std::to_string(rx_period_pvt) << "\x0D"; 
		IO.send(ss.str());
		++stream;
		g_response_condition.wait(lock, [](){return g_response_received;});
		g_response_received = false;
	}
	if (publish_gpgsa_ == true)
	{
		std::stringstream ss;
		
		ss << "sno, Stream" << std::to_string(stream) << ", " << rx_port << ", GSA, " << pvt_sec_or_msec << 
			std::to_string(rx_period_pvt) << "\x0D"; 
		IO.send(ss.str());
		++stream;
		g_response_condition.wait(lock, [](){return g_response_received;});
		g_response_received = false;
	}
	if (publish_gpgsv_ == true)
	{
		std::stringstream ss;
		
		ss << "sno, Stream" << std::to_string(stream) << ", " << rx_port << ", GSV, " << rest_sec_or_msec << 
			std::to_string(rx_period_rest) << "\x0D"; 
		IO.send(ss.str());
		++stream;
		g_response_condition.wait(lock, [](){return g_response_received;});
		g_response_received = false;
	}
	if (publish_pvtcartesian_ == true)
	{
		std::stringstream ss;
		ss << "sso, Stream" << std::to_string(stream) << ", " << rx_port << ", PVTCartesian, " << pvt_sec_or_msec << 
			std::to_string(rx_period_pvt) << "\x0D";
		IO.send(ss.str());
		++stream;
		g_response_condition.wait(lock, [](){return g_response_received;});
		g_response_received = false;
	}
	if (publish_pvtgeodetic_ == true)
	{
		std::stringstream ss;
		ss << "sso, Stream" << std::to_string(stream) << ", " << rx_port << ", PVTGeodetic, " << pvt_sec_or_msec << 
			std::to_string(rx_period_pvt) << "\x0D";
		IO.send(ss.str());
		++stream;
		g_response_condition.wait(lock, [](){return g_response_received;});
		g_response_received = false;
	}
	if (publish_poscovcartesian_ == true)
	{
		std::stringstream ss;
		ss << "sso, Stream" << std::to_string(stream) << ", " << rx_port << ", PosCovCartesian, " << pvt_sec_or_msec << 
			std::to_string(rx_period_pvt) << "\x0D";
		IO.send(ss.str());
		++stream;
		g_response_condition.wait(lock, [](){return g_response_received;});
		g_response_received = false;
	}
	if (publish_poscovgeodetic_ == true)
	{
		std::stringstream ss;
		ss << "sso, Stream" << std::to_string(stream) << ", " << rx_port << ", PosCovGeodetic, " << pvt_sec_or_msec << 
			std::to_string(rx_period_pvt) << "\x0D";
		IO.send(ss.str());
		++stream;
		g_response_condition.wait(lock, [](){return g_response_received;});
		g_response_received = false;
	}
	if (publish_atteuler_ == true)
	{
		std::stringstream ss;
		ss << "sso, Stream" << std::to_string(stream) << ", " << rx_port << ", AttEuler, " << rest_sec_or_msec << 
			std::to_string(rx_period_rest) << "\x0D";
		IO.send(ss.str());
		++stream;
		g_response_condition.wait(lock, [](){return g_response_received;});
		g_response_received = false;
	}
	if (publish_attcoveuler_ == true)
	{
		std::stringstream ss;
		ss << "sso, Stream" << std::to_string(stream) << ", " << rx_port << ", AttCovEuler, " << rest_sec_or_msec << 
			std::to_string(rx_period_rest) << "\x0D";
		IO.send(ss.str());
		++stream;
		g_response_condition.wait(lock, [](){return g_response_received;});
		g_response_received = false;
	}
	if (g_publish_gpsfix == true)
	{
		std::stringstream ss;
		ss << "sso, Stream" << std::to_string(stream) << ", " << rx_port << ", ChannelStatus, " << pvt_sec_or_msec << 
			std::to_string(rx_period_pvt) << "\x0D";
		IO.send(ss.str());
		++stream;
		g_response_condition.wait(lock, [](){return g_response_received;});
		g_response_received = false;
		ss.str(std::string()); // avoids invoking the std::string constructor
		ss << "sso, Stream" << std::to_string(stream) << ", " << rx_port << ", MeasEpoch, " << pvt_sec_or_msec << 
			std::to_string(rx_period_pvt) << "\x0D";
		IO.send(ss.str());
		++stream;
		g_response_condition.wait(lock, [](){return g_response_received;});
		g_response_received = false;
		ss.str(std::string()); 
		ss << "sso, Stream" << std::to_string(stream) << ", " << rx_port << ", DOP, " << pvt_sec_or_msec << 
			std::to_string(rx_period_pvt) << "\x0D";
		IO.send(ss.str());
		++stream;
		g_response_condition.wait(lock, [](){return g_response_received;});
		g_response_received = false;
		ss.str(std::string()); 
		ss << "sso, Stream" << std::to_string(stream) << ", " << rx_port << ", VelCovGeodetic, " << pvt_sec_or_msec << 
			std::to_string(rx_period_pvt) << "\x0D";
		IO.send(ss.str());
		++stream;
		g_response_condition.wait(lock, [](){return g_response_received;});
		g_response_received = false;
	}
	if (g_publish_diagnostics == true)
	{
		std::stringstream ss;
		ss << "sso, Stream" << std::to_string(stream) << ", " << rx_port << ", ReceiverStatus, " << rest_sec_or_msec << 
			std::to_string(rx_period_rest) << "\x0D";
		IO.send(ss.str());
		++stream;
		g_response_condition.wait(lock, [](){return g_response_received;});
		g_response_received = false;
		ss.str(std::string()); 
		ss << "sso, Stream" << std::to_string(stream) << ", " << rx_port << ", QualityInd, " << rest_sec_or_msec << 
			std::to_string(rx_period_rest) << "\x0D";
		IO.send(ss.str());
		++stream;
		g_response_condition.wait(lock, [](){return g_response_received;});
		g_response_received = false;
		ss.str(std::string()); 
		ss << "sso, Stream" << std::to_string(stream) << ", " << rx_port << ", ReceiverSetup, " << rest_sec_or_msec << 
			std::to_string(rx_period_rest) << "\x0D";
		IO.send(ss.str());
		++stream;
		g_response_condition.wait(lock, [](){return g_response_received;});
		g_response_received = false;
	}
	
	// Setting the marker-to-ARP offsets. This comes after the "sso, ..., ReceiverSetup, ..." command, 
	// since the latter is only generated when a user-command is entered to change one or more values in the block.
	{
		std::stringstream ss;
		ss << "sao, Main, " << string_utilities::trimString(std::to_string(delta_e_)) << ", " << 
			string_utilities::trimString(std::to_string(delta_n_)) << ", " << 
			string_utilities::trimString(std::to_string(delta_u_)) << ", \"" << ant_type_ << "\", \"" << ant_serial_nr_ << 
			"\", 0 \x0D"; 
		IO.send(ss.str());
	}
	g_response_condition.wait(lock, [](){return g_response_received;});
	g_response_received = false;
	
	// Configuring the NTRIP connection
	// First disable any existing NTRIP connection on NTR1
	{
		std::stringstream ss;
		ss << "snts, NTR1, off \x0D"; 
		IO.send(ss.str());
	}
	g_response_condition.wait(lock, [](){return g_response_received;});
	g_response_received = false;
	if (rx_has_internet_)
	{
		if (mode_ == "off")
		{
		}
		else if (mode_ == "Client")
		{
			{
				std::stringstream ss;
				ss << "snts, NTR1, " << mode_ << ", " << caster_ << ", " << std::to_string(caster_port_) << ", " << 
					username_ << ", " << password_ << ", " << mountpoint_ << ", " << ntrip_version_ << ", " << send_gga_ << " \x0D"; 
				IO.send(ss.str());
			}
			g_response_condition.wait(lock, [](){return g_response_received;});
			g_response_received = false;
		}
		else if (mode_ == "Client-Sapcorda")
		{
			{
				std::stringstream ss;
				ss << "snts, NTR1, Client-Sapcorda, , , , , , , , \x0D"; 
				IO.send(ss.str());
			}
			g_response_condition.wait(lock, [](){return g_response_received;});
			g_response_received = false;
		}
		else 
		{
			ROS_ERROR("Invalid mode specified for NTRIP settings.");
		}
	}
	else 	// Since the Rx does not have internet (and you will not be able to share it via USB), 
			//we need to forward the corrections ourselves, though not on the same port.
	{
		if (proto == "tcp")
		{
			{
				std::stringstream ss;
				// In case IPS1 was used before, old configuration is lost of course.
				ss << "siss, IPS1, " << std::to_string(rx_input_corrections_tcp_) << ", TCP2Way \x0D"; 
				IO.send(ss.str());
			}
			g_response_condition.wait(lock, [](){return g_response_received;});
			g_response_received = false;
			{
				std::stringstream ss;
				ss << "sno, Stream" << std::to_string(stream) << ", IPS1, GGA, " << pvt_sec_or_msec << 
					std::to_string(rx_period_pvt) << " \x0D";
				++stream;
				IO.send(ss.str());
			}
			g_response_condition.wait(lock, [](){return g_response_received;});
			g_response_received = false;
		}
		{
			std::stringstream ss;
			if (proto == "tcp")
			{
				ss << "sdio, IPS1, " << rtcm_version_ << ", +SBF+NMEA \x0D"; 
			}
			else
			{
				ss << "sdio, " << rx_input_corrections_serial_ << ", " << rtcm_version_ << ", +SBF+NMEA \x0D"; 
			}
			IO.send(ss.str());
		}
	}
	ROS_DEBUG("Leaving configureRx() method");
}
void rosaic_node::ROSaicNode::getROSParams() 
{
	// Communication parameters
	g_nh->param("device", device_, std::string("/dev/ttyACM0"));
	getROSInt("serial/baudrate", baudrate_, static_cast<uint32_t>(115200));
	g_nh->param("serial/hw_flow_control", hw_flow_control_, std::string("off"));
	g_nh->param("serial/rx_serial_port", rx_serial_port_, std::string("USB1"));
	
	g_nh->param("reconnect_delay_s", reconnect_delay_s_, 4.0f);
	
	// Polling period parameters
	getROSInt("polling_period/pvt", polling_period_pvt_, static_cast<uint32_t>(1000));
	if (polling_period_pvt_ != 10 && polling_period_pvt_ != 20 && polling_period_pvt_ != 50 && polling_period_pvt_ != 100 
	&& polling_period_pvt_ != 200 && polling_period_pvt_ != 250 && polling_period_pvt_ != 500 && polling_period_pvt_ != 1000 
	&& polling_period_pvt_ != 2000 && polling_period_pvt_ != 5000 && polling_period_pvt_ != 10000)
	{
		ROS_ERROR("Please specify a valid polling period for PVT-related SBF blocks and NMEA messages.");
	}
	getROSInt("polling_period/rest", polling_period_rest_, static_cast<uint32_t>(1000));
	if (polling_period_rest_ != 10 && polling_period_rest_ != 20 && polling_period_rest_ != 50 && polling_period_rest_ != 100 
	&& polling_period_rest_ != 200 && polling_period_rest_ != 250 && polling_period_rest_ != 500 && polling_period_rest_ != 1000 
	&& polling_period_rest_ != 2000 && polling_period_rest_ != 5000 && polling_period_rest_ != 10000)
	{
		ROS_ERROR("Please specify a valid polling period for PVT-unrelated SBF blocks and NMEA messages.");
	}
	
	// Datum and marker-to-ARP offset
	g_nh->param("datum", datum_, std::string("ETRS89"));
	g_nh->param("ant_type", ant_type_, std::string("Unknown"));
	g_nh->param("ant_serial_nr", ant_serial_nr_, std::string("Unknown"));
	g_nh->param("marker_to_arp/delta_e", delta_e_, 0.0f);
	g_nh->param("marker_to_arp/delta_n", delta_n_, 0.0f);
	g_nh->param("marker_to_arp/delta_u", delta_u_, 0.0f);
	
	// Correction service parameters
	g_nh->param("ntrip_settings/mode", mode_, std::string("off"));
	g_nh->param("ntrip_settings/caster", caster_, std::string());
	getROSInt("ntrip_settings/caster_port", caster_port_, static_cast<uint32_t>(0));
	g_nh->param("ntrip_settings/username", username_, std::string());
	g_nh->param("ntrip_settings/password", password_, std::string());
	g_nh->param("ntrip_settings/mountpoint", mountpoint_, std::string());
	g_nh->param("ntrip_settings/ntrip_version", ntrip_version_, std::string("v2"));
	g_nh->param("ntrip_settings/send_gga", send_gga_, std::string("auto"));
	g_nh->param("ntrip_settings/rx_has_internet", rx_has_internet_, false);
	g_nh->param("ntrip_settings/rtcm_version", rtcm_version_, std::string("RTCMv3"));
	getROSInt("ntrip_settings/rx_input_corrections_tcp", rx_input_corrections_tcp_, static_cast<uint32_t>(28785));
	g_nh->param("ntrip_settings/rx_input_corrections_serial", rx_input_corrections_serial_, std::string("USB2"));
	
	// Publishing parameters, the others remained global since they need to be accessed in the callbackhandlers.hpp file
	g_nh->param("publish/gpgga", publish_gpgga_, true);
	g_nh->param("publish/gprmc", publish_gprmc_, true);
	g_nh->param("publish/gpgsa", publish_gpgsa_, true);
	g_nh->param("publish/gpgsv", publish_gpgsv_, true);
	g_nh->param("publish/pvtcartesian", publish_pvtcartesian_, true);
	g_nh->param("publish/pvtgeodetic", publish_pvtgeodetic_, true);
	g_nh->param("publish/poscovcartesian", publish_poscovcartesian_, true);
	g_nh->param("publish/poscovgeodetic", publish_poscovgeodetic_, true);
	g_nh->param("publish/atteuler", publish_atteuler_, true);
	g_nh->param("publish/attcoveuler", publish_attcoveuler_, true);

	// To be implemented: RTCM, setting datum, raw data settings, PPP, SBAS, fix mode...
	ROS_DEBUG("Finished getROSParams() method");

}

void rosaic_node::ROSaicNode::initializeIO() 
{
	ROS_DEBUG("Called initializeIO() method");
	boost::smatch match;
	// In fact: smatch is a typedef of match_results<string::const_iterator>
	if (boost::regex_match(device_, match, boost::regex("(tcp)://(.+):(\\d+)"))) 
	// \d means decimal, however, in the regular expression, the \ is a special character, which needs 
	// to be escaped on its own as well..
	// Note that regex_match can be used with a smatch object to store results, or without. In any case, 
	// true is returned if and only if it matches the !complete! string.
	{
		// The first sub_match (index 0) contained in a match_result always represents the full match 
		// within a target sequence made by a regex, and subsequent sub_matches represent sub-expression 
		// matches corresponding in sequence to the left parenthesis delimiting the sub-expression in the regex,
		// i.e. $n Perl is equivalent to m[n] in boost regex.
		std::string proto(match[1]);
		if (proto == "tcp") 
		{
			tcp_host_ = match[2];
			tcp_port_ = match[3];
			
			serial_ = false;
			boost::thread temporary_thread(boost::bind(&ROSaicNode::connect, this));
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
		boost::thread temporary_thread(boost::bind(&ROSaicNode::connect, this));
		temporary_thread.detach();
	}
	ROS_DEBUG("Leaving initializeIO() method");
}

void rosaic_node::ROSaicNode::connect() 
{
	ROS_DEBUG("Called connect() method");
	ROS_DEBUG("Setting ROS timer for calling reconnect() method until connection succeds");
	reconnect_timer_ = g_nh->createTimer(ros::Duration(reconnect_delay_s_), &ROSaicNode::reconnect, this);
	reconnect_timer_.start();
	ROS_DEBUG("Started ROS timer for calling reconnect() method until connection succeds"); 
	ros::spin();
	ROS_DEBUG("Leaving connect() method"); // This will never be output since ros::spin() is on the line above.
}

//! In serial mode (not USB, since the Rx port is then called USB1 or USB2), please ensure that you are 
//! connected to the Rx's COM1, COM2 or COM3 port, !if! you employ UART hardware flow control.
void rosaic_node::ROSaicNode::reconnect(const ros::TimerEvent& event) 
{
	ROS_DEBUG("Called reconnect() method");
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
				initialize_serial_return = IO.initializeSerial(device_, baudrate_, hw_flow_control_);
			}
			catch (std::runtime_error& e)
			{
				{
					std::stringstream ss;
					ss << "IO.initializeSerial() failed for device " << device_ << " due to: " << e.what();
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
				initialize_tcp_return = IO.initializeTCP(tcp_host_, tcp_port_);
			}
			catch (std::runtime_error& e)
			{
				{
					std::stringstream ss;
					ss << "IO.initializeTCP() failed for host " << tcp_host_ << " on port " << tcp_port_ << 
						" due to: " << e.what();
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

//! initializeSerial is not self-contained: The for loop in Callbackhandlers' handle method would 
//! never open a specific handler unless the handler is added (=inserted) to the C++ map via this 
//! function. This way, the specific handler can be called, in which in turn RxMessage's read() method is 
//! called, thereby "message" occupied, and func_ (the handler we insert in this function) called with 
//! this "message".
void rosaic_node::ROSaicNode::defineMessages() 
{
	ROS_DEBUG("Called defineMessages() method");
	
	if (publish_gpgga_ == true)
	{
		IO.handlers_.callbackmap_ = IO.getHandlers().insert<septentrio_gnss_driver::Gpgga>("$GPGGA");
	}
	if (publish_gprmc_ == true)
	{
		IO.handlers_.callbackmap_ = IO.getHandlers().insert<septentrio_gnss_driver::Gprmc>("$GPRMC");
	}
	if (publish_gpgsa_ == true)
	{
		IO.handlers_.callbackmap_ = IO.getHandlers().insert<septentrio_gnss_driver::Gpgsa>("$GPGSA");
	}
	if (publish_gpgsv_ == true)
	{
		IO.handlers_.callbackmap_ = IO.getHandlers().insert<septentrio_gnss_driver::Gpgsv>("$GPGSV");
		IO.handlers_.callbackmap_ = IO.getHandlers().insert<septentrio_gnss_driver::Gpgsv>("$GLGSV");
		IO.handlers_.callbackmap_ = IO.getHandlers().insert<septentrio_gnss_driver::Gpgsv>("$GAGSV");
		IO.handlers_.callbackmap_ = IO.getHandlers().insert<septentrio_gnss_driver::Gpgsv>("$GBGSV");
	}
	if (publish_pvtcartesian_ == true)
	{
		IO.handlers_.callbackmap_ = IO.getHandlers().insert<septentrio_gnss_driver::PVTCartesian>("4006");
	}
	if (publish_pvtgeodetic_ == true)
	{
		IO.handlers_.callbackmap_ = IO.getHandlers().insert<septentrio_gnss_driver::PVTGeodetic>("4007");
	}
	if (publish_poscovcartesian_ == true)
	{
		IO.handlers_.callbackmap_ = IO.getHandlers().insert<septentrio_gnss_driver::PosCovCartesian>("5905");
	}
	if (publish_poscovgeodetic_ == true)
	{
		IO.handlers_.callbackmap_ = IO.getHandlers().insert<septentrio_gnss_driver::PosCovGeodetic>("5906");
	}
	if (publish_atteuler_ == true)
	{
		IO.handlers_.callbackmap_ = IO.getHandlers().insert<septentrio_gnss_driver::AttEuler>("5938");
	}
	if (publish_attcoveuler_ == true)
	{
		IO.handlers_.callbackmap_ = IO.getHandlers().insert<septentrio_gnss_driver::AttCovEuler>("5939");
	}
	if (g_publish_gpst == true)
	{
		IO.handlers_.callbackmap_ = IO.getHandlers().insert<int32_t>("GPST");
	}
	if (g_publish_navsatfix == true)
	{
		if (publish_pvtgeodetic_ == false || publish_poscovgeodetic_ == false)
		{
			ROS_ERROR("For a proper NavSatFix message, please set the publish/pvtgeodetic and the publish/poscovgeodetic ROSaic parameters both to true.");
		}
		IO.handlers_.callbackmap_ = IO.getHandlers().insert<sensor_msgs::NavSatFix>("NavSatFix");
	}
	if (g_publish_gpsfix == true)
	{
		if (publish_pvtgeodetic_ == false || publish_poscovgeodetic_ == false)
		{
			ROS_ERROR("For a proper GPSFix message, please set the publish/pvtgeodetic and the publish/poscovgeodetic ROSaic parameters both to true.");
		}
		IO.handlers_.callbackmap_ = IO.getHandlers().insert<gps_common::GPSFix>("GPSFix");
		// The following blocks are never published, yet are needed for the construction of the GPSFix message, hence we have empty callbacks.
		IO.handlers_.callbackmap_ = IO.getHandlers().insert<int32_t>("4013"); // ChannelStatus block
		IO.handlers_.callbackmap_ = IO.getHandlers().insert<int32_t>("4027"); // MeasEpoch block
		IO.handlers_.callbackmap_ = IO.getHandlers().insert<int32_t>("4001"); // DOP block
		IO.handlers_.callbackmap_ = IO.getHandlers().insert<int32_t>("5908"); // VelCovGeodetic block
	}
	if (g_publish_pose == true)
	{
		if (publish_pvtgeodetic_ == false || publish_poscovgeodetic_ == false || publish_atteuler_ == false || 
			publish_attcoveuler_ == false)
		{
			ROS_ERROR("For a proper PoseWithCovarianceStamped message, please set the publish/pvtgeodetic, publish/poscovgeodetic, publish_atteuler and publish_attcoveuler ROSaic parameters all to true.");
		}
		IO.handlers_.callbackmap_ = IO.getHandlers().insert<geometry_msgs::PoseWithCovarianceStamped>("PoseWithCovarianceStamped");
	}
	if (g_publish_diagnostics == true)
	{
		IO.handlers_.callbackmap_ = IO.getHandlers().insert<diagnostic_msgs::DiagnosticArray>("DiagnosticArray");
		IO.handlers_.callbackmap_ = IO.getHandlers().insert<int32_t>("4014"); // ReceiverStatus block
		IO.handlers_.callbackmap_ = IO.getHandlers().insert<int32_t>("4082"); // QualityInd block
		IO.handlers_.callbackmap_ = IO.getHandlers().insert<int32_t>("5902"); // ReceiverSetup block
	}
	// so on and so forth...
	ROS_DEBUG("Leaving defineMessages() method");
}


//! If true, the ROS message headers' unix time field is constructed from the TOW (in the SBF case) 
//! and UTC (in the NMEA case) data. 
//! If false, times are constructed within the driver via time(NULL) of the <ctime> library.
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
//! Mutex to control changes of global variable "g_response_received"
boost::mutex g_response_mutex;
//! Determines whether a command reply was received from the Rx
bool g_response_received;
//! Condition variable complementing "g_response_mutex"
boost::condition_variable g_response_condition;
//! Mutex to control changes of global variable "g_cd_received"
boost::mutex g_cd_mutex;
//! Determines whether the connection descriptor was received from the Rx
bool g_cd_received;
//! Condition variable complementing "g_cd_mutex"
boost::condition_variable g_cd_condition;
//! Whether or not we still want to read the connection descriptor, which we only want in the very beginning to 
//! know whether it is IP10, IP11 etc.
bool g_read_cd;
//! Rx TCP port, e.g. IP10 or IP11, to which ROSaic is connected to
std::string g_rx_tcp_port;
//! Since after SSSSSSSSSSS we need to wait for second connection descriptor, we have to count the connection descriptors
uint32_t g_cd_count;
//! For GPSFix: Whether the ChannelStatus block of the current epoch has arrived or not
bool g_channelstatus_has_arrived_gpsfix;
//! For GPSFix: Whether the MeasEpoch block of the current epoch has arrived or not
bool g_measepoch_has_arrived_gpsfix;
//! For GPSFix: Whether the DOP block of the current epoch has arrived or not
bool g_dop_has_arrived_gpsfix;
//! For GPSFix: Whether the PVTGeodetic block of the current epoch has arrived or not
bool g_pvtgeodetic_has_arrived_gpsfix;
//! For GPSFix: Whether the PosCovGeodetic block of the current epoch has arrived or not
bool g_poscovgeodetic_has_arrived_gpsfix;
//! For GPSFix: Whether the VelCovGeodetic block of the current epoch has arrived or not
bool g_velcovgeodetic_has_arrived_gpsfix;
//! For GPSFix: Whether the AttEuler block of the current epoch has arrived or not
bool g_atteuler_has_arrived_gpsfix;
//! For GPSFix: Whether the AttCovEuler block of the current epoch has arrived or not
bool g_attcoveuler_has_arrived_gpsfix;
//! For NavSatFix: Whether the PVTGeodetic block of the current epoch has arrived or not
bool g_pvtgeodetic_has_arrived_navsatfix;
//! For NavSatFix: Whether the PosCovGeodetic block of the current epoch has arrived or not
bool g_poscovgeodetic_has_arrived_navsatfix;
//! For PoseWithCovarianceStamped: Whether the PVTGeodetic block of the current epoch has arrived or not
bool g_pvtgeodetic_has_arrived_pose;
//! For PoseWithCovarianceStamped: Whether the PosCovGeodetic block of the current epoch has arrived or not
bool g_poscovgeodetic_has_arrived_pose;
//! For PoseWithCovarianceStamped: Whether the AttEuler block of the current epoch has arrived or not
bool g_atteuler_has_arrived_pose;
//! For PoseWithCovarianceStamped: Whether the AttCovEuler block of the current epoch has arrived or not
bool g_attcoveuler_has_arrived_pose;
//! For DiagnosticArray: Whether the ReceiverStatus block of the current epoch has arrived or not
bool g_receiverstatus_has_arrived_diagnostics;
//! For DiagnosticArray: Whether the QualityInd block of the current epoch has arrived or not
bool g_qualityind_has_arrived_diagnostics;
//! A C++ map for keeping track of the SBF blocks necessary to construct the GPSFix ROS message
std::map<std::string, uint32_t> g_GPSFixMap;
//! A C++ map for keeping track of the SBF blocks necessary to construct the NavSatFix ROS message
std::map<std::string, uint32_t> g_NavSatFixMap;
//! A C++ map for keeping track of SBF blocks necessary to construct the PoseWithCovarianceStamped ROS message
std::map<std::string, uint32_t> g_PoseWithCovarianceStampedMap;
//! A C++ map for keeping track of SBF blocks necessary to construct the DiagnosticArray ROS message
std::map<std::string, uint32_t> g_DiagnosticArrayMap;
//! Node Handle for the ROSaic node
//! You must initialize the NodeHandle in the "main" function (or in any method called 
//! indirectly or directly by the main function). 
//! One can declare a pointer to the NodeHandle to be a global variable and then initialize it afterwards only...
boost::shared_ptr<ros::NodeHandle> g_nh;
//! Queue size for ROS publishers
const uint32_t g_ROS_QUEUE_SIZE = 1;

int main(int argc, char** argv) 
{
	ros::init(argc, argv, "septentrio_gnss");
	g_nh.reset(new ros::NodeHandle("~")); 
	g_nh->param("use_gnss_time", g_use_gnss_time, true);
	g_nh->param("frame_id", g_frame_id, (std::string) "gnss"); 
	g_nh->param("publish/gpst", g_publish_gpst, true);
	g_nh->param("publish/navsatfix", g_publish_navsatfix, true);
	g_nh->param("publish/gpsfix", g_publish_gpsfix, true);
	g_nh->param("publish/pose", g_publish_pose, true);
	g_nh->param("publish/diagnostics", g_publish_diagnostics, true);
	rosaic_node::getROSInt("leap_seconds", g_leap_seconds, static_cast<uint32_t>(18));
	
	g_response_received = false;
	g_cd_received = false;
	g_read_cd = true;
	g_cd_count = 0;
	g_channelstatus_has_arrived_gpsfix = false;
	g_measepoch_has_arrived_gpsfix = false;
	g_dop_has_arrived_gpsfix = false;
	g_pvtgeodetic_has_arrived_gpsfix = false;
	g_pvtgeodetic_has_arrived_navsatfix = false;
	g_pvtgeodetic_has_arrived_pose = false;
	g_poscovgeodetic_has_arrived_gpsfix = false;
	g_poscovgeodetic_has_arrived_navsatfix = false;
	g_poscovgeodetic_has_arrived_pose = false;
	g_velcovgeodetic_has_arrived_gpsfix = false;
	g_atteuler_has_arrived_gpsfix = false;
	g_atteuler_has_arrived_pose = false;
	g_attcoveuler_has_arrived_gpsfix = false;
	g_attcoveuler_has_arrived_pose = false;
	g_receiverstatus_has_arrived_diagnostics = false;
	g_qualityind_has_arrived_diagnostics = false;
	
	// The info logging level seems to be default, hence we modify log level momentarily..
	// The following is the C++ version of rospy.init_node('my_ros_node', log_level=rospy.DEBUG)
	if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
								   ros::console::levels::Debug)) //debug is lowest level, shows everything
	ros::console::notifyLoggerLevelsChanged();
	
	rosaic_node::ROSaicNode rx_node; // This launches everything we need, in theory :)
	return 0;
}
