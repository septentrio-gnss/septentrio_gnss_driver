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
	ROS_DEBUG("Entered ROSaicNode() constructor..");
	// Parameters must be set before initializing IO
	GetROSParams();
	StringValues_Initialize();
	ROS_DEBUG("About to call InitializeIO() method");
	try
	{
		InitializeIO();
	}
	catch (std::runtime_error& e)
	{
		ROS_ERROR("InitializeIO() failed: %s", e.what());
	}
	// Subscribe to all requested mosaic messages and publish them raw or in composite form (e.g. NavSatFix)
    DefineMessages();
	// Testing Send() method of AsyncManager class
	std::string cmd = "grc \x0D";
	IO.Send(cmd);
	ros::spin();
	
}


void rosaic_node::ROSaicNode::GetROSParams() 
{
	nh->param("device", device_, std::string("/dev/ttyACM0"));	
	// Serial params
	GetROSInt("serial/baudrate", baudrate_, (uint32_t) 115200);
	// To be implemented: RTCM, setting datum, raw data settings, PPP, SBAS, fix mode...
	ROS_DEBUG("Finished GetROSParams() method");

}	


void rosaic_node::ROSaicNode::InitializeIO() 
{
	ROS_DEBUG("Called InitializeIO() method");
	boost::smatch match;
	// In fact: smatch is a typedef of match_results<string::const_iterator>
	if (boost::regex_match(device_, match, boost::regex("(tcp|udp)://(.+):(\\d+)"))) 
	// \d means decimal, however, in the regular expression, the \ is a special character, which needs to be escaped on its own as well..
	// Note that regex_match can be used with a smatch object to store results, or without. In any case, true is returned if and only if it matches the !complete! string.
	{
		// The first sub_match (index 0) contained in a match_result always represents the full match within a target sequence made by a regex, 
		// and subsequent sub_matches represent sub-expression matches corresponding in sequence to the left parenthesis delimiting the sub-expression in the regex,
		// i.e. $n Perl is equivalent to m[n] in boost regex.
		std::string proto(match[1]);
		if (proto == "tcp") 
		{
			std::string host(match[2]);
			std::string port(match[3]);
			ROS_INFO("Connecting to %s://%s:%s ...", proto.c_str(), host.c_str(), port.c_str());
			try
			{
				IO.InitializeTCP(host, port);
			}
			catch (std::runtime_error& e)
			{
				throw std::runtime_error("IO.InitializeTCP() failed for host " + host + " on port " + port + " due to: " + e.what());
			}
		} 
		else 
		{
			throw std::runtime_error("Protocol '" + proto + "' is unsupported");
		}
	} 
	else 
	{
		// To be modified here, or clarified: how to use reconnect_delay_s properly? Is respawn (roslaunch parameter) enough?
		//ROS_DEBUG("Setting timer for calling InitializeSerial() method");
		//nh->param("reconnect_delay_s", reconnect_delay_s_, 0.5f);
		//reconnect_timer_ = nh->createTimer(ros::Duration(reconnect_delay_s_), &ROSaicNode::Reconnect, this);
		//reconnect_timer_.start();
		//ROS_DEBUG("Started timer"); //not printed if error in callback of course
		//ros::spin(); // otherwise callback will never be called, with ros::spin i cannot leave InitializeIO(), yet with ros::spinOnce can enter reconnect() even once
		try
		{
			IO.InitializeSerial(device_, baudrate_);
		}
		catch (std::runtime_error& e)
		{
			throw std::runtime_error("IO.InitializeSerial() failed for device " + device_ + " due to: " + e.what());
		}
	}
	ROS_DEBUG("Leaving InitializeIO()");
}

void rosaic_node::ROSaicNode::Reconnect(const ros::TimerEvent& event) 
{
	ROS_DEBUG("Inside reconnect");
	if(IO.InitializeSerial(device_, baudrate_))
	{
		connected_ = true;
	}
	if (connected_ == true)
	{
		reconnect_timer_.stop();
		ROS_DEBUG("Ended timer");
	}
	ROS_DEBUG("Leaving reconnect");
}

//! InitializeSerial is not self-contained: The for loop in Callbackhandlers' handle method would never open a specific handler unless 
//! the handler is added (=inserted) to the C++ map via this function. This way, the specific handler can be called, in which in turn 
//! mosaicMessage's read() method is called, thereby "message" occupied, and func_ (the handler we insert in this function) called with this "message".
void rosaic_node::ROSaicNode::DefineMessages() 
{
	ROS_DEBUG("Entered DefineMessages() method");

	if (publish_gpgga == true)
	{
		IO.handlers_.callbackmap_ = IO.GetHandlers().Insert<rosaic::Gpgga>("$GPGGA", boost::bind(Publish<rosaic::Gpgga>, _1, "/gpgga"));
		std::multimap<std::string, boost::shared_ptr<io_comm_mosaic::AbstractCallbackHandler> >::key_type key = "$GPGGA";
		ROS_DEBUG("Back to DefineMessages() method: The element exists in our map: %u", (unsigned int) IO.handlers_.callbackmap_.count(key));
	}
	if (publish_pvtcartesian == true)
	{
		IO.handlers_.callbackmap_ = IO.GetHandlers().Insert<rosaic::PVTCartesian>("4006", boost::bind(Publish<rosaic::PVTCartesian>, _1, "/pvtcartesian"));
		std::multimap<std::string, boost::shared_ptr<io_comm_mosaic::AbstractCallbackHandler> >::key_type key = "4006";
		ROS_DEBUG("Back to DefineMessages() method: The element exists in our map: %u", (unsigned int) IO.handlers_.callbackmap_.count(key));
	}
	if (publish_pvtgeodetic == true)
	{
		IO.handlers_.callbackmap_ = IO.GetHandlers().Insert<rosaic::PVTGeodetic>("4007", boost::bind(Publish<rosaic::PVTGeodetic>, _1, "/pvtgeodetic"));
		std::multimap<std::string, boost::shared_ptr<io_comm_mosaic::AbstractCallbackHandler> >::key_type key = "4007";
		ROS_DEBUG("Back to DefineMessages() method: The element exists in our map: %u", (unsigned int) IO.handlers_.callbackmap_.count(key));
	}
	if (publish_poscovgeodetic == true)
	{
		IO.handlers_.callbackmap_ = IO.GetHandlers().Insert<rosaic::PosCovGeodetic>("5906", boost::bind(Publish<rosaic::PosCovGeodetic>, _1, "/poscovgeodetic"));
		std::multimap<std::string, boost::shared_ptr<io_comm_mosaic::AbstractCallbackHandler> >::key_type key = "5906";
		ROS_DEBUG("Back to DefineMessages() method: The element exists in our map: %u", (unsigned int) IO.handlers_.callbackmap_.count(key));
	}
	if (publish_atteuler == true)
	{
		IO.handlers_.callbackmap_ = IO.GetHandlers().Insert<rosaic::AttEuler>("5938", boost::bind(Publish<rosaic::AttEuler>, _1, "/atteuler"));
		std::multimap<std::string, boost::shared_ptr<io_comm_mosaic::AbstractCallbackHandler> >::key_type key = "5938";
		ROS_DEBUG("Back to DefineMessages() method: The element exists in our map: %u", (unsigned int) IO.handlers_.callbackmap_.count(key));
	}
	if (publish_attcoveuler == true)
	{
		IO.handlers_.callbackmap_ = IO.GetHandlers().Insert<rosaic::AttCovEuler>("5939", boost::bind(Publish<rosaic::AttCovEuler>, _1, "/attcoveuler"));
		std::multimap<std::string, boost::shared_ptr<io_comm_mosaic::AbstractCallbackHandler> >::key_type key = "5939";
		ROS_DEBUG("Back to DefineMessages() method: The element exists in our map: %u", (unsigned int) IO.handlers_.callbackmap_.count(key));
	}
	if (publish_navsatfix == true)
	{
		IO.handlers_.callbackmap_ = IO.GetHandlers().Insert<sensor_msgs::NavSatFix>("NavSatFix", boost::bind(Publish<sensor_msgs::NavSatFix>, _1, "/navsatfix"));
		std::multimap<std::string, boost::shared_ptr<io_comm_mosaic::AbstractCallbackHandler> >::key_type key = "NavSatFix";
		ROS_DEBUG("Back to DefineMessages() method: The element exists in our map: %u", (unsigned int) IO.handlers_.callbackmap_.count(key));
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
//! The frame ID used in the header of every published ROS message
std::string frame_id;
//! The get_handlers() method of the Comm_IO class forces us to make this mutex static, since otherwise, 
//! the method would need to construct-by-copy the mutex, which is strictly prohibited.
boost::mutex io_comm_mosaic::CallbackHandlers::callback_mutex_;

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
