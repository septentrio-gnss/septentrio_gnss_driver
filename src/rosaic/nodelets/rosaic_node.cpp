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

std::string frame_id;

 
rosaic_node::ROSaicNode::ROSaicNode()
{
	ROS_DEBUG("Entered ROSaicNode() constructor..");
	// Params must be set before initializing IO
	GetROSParams();
	StringValues_Initialize();
	ROS_DEBUG("About to call InitializeIO() method");
	InitializeIO();
	// Subscribe to all requested mosaic messages and publish them
    Subscribe();
	ros::spin();
	
}


void rosaic_node::ROSaicNode::GetROSParams() 
{
	nh->param("device", device_, std::string("/dev/ttyACM0"));	
	// Serial params
	GetROSInt("serial/baudrate", baudrate_, 115200);
	// To be implemented: RTCM, setting datum, raw data settings, PPP, SBAS, fix mode...
	ROS_DEBUG("Finished GetROSParams() method");

}	


void rosaic_node::ROSaicNode::InitializeIO() 
{
	ROS_DEBUG("Called InitializeIO() method");
	boost::smatch match;
	// In fact: typedef match_results<string::const_iterator> smatch;
	if (boost::regex_match(device_, match,
                         boost::regex("(tcp|udp)://(.+):(\\d+)"))) 
	// regex_match can be used with a smatch object to store results, or without. In any case, true is returned if and only if it matches the !complete! string.
	{
		// The first sub_match (index 0) contained in a match_result always represents the full match within a target sequence made by a regex, 
		// and subsequent sub_matches represent sub-expression matches corresponding in sequence to the left parenthesis delimiting the sub-expression in the regex,
		// i.e. $n Perl is equivalent to m[n] in boost regex.
		std::string proto(match[1]);
		if (proto == "tcp") 
		{
			// To be written
			/*
			std::string host(match[2]);
			std::string port(match[3]);
			ROS_INFO("Connecting to %s://%s:%s ...", proto.str(), host.str(), port.str());
			IO.InitializeTCP(host, port);
			*/
		} 
		else 
		{
			throw std::runtime_error("Protocol '" + proto + "' is unsupported");
		}
	} 
	else 
	{
		// To be modified here, or clarified: how to use reconnect_delay_s properly? Is respawn (roslaunch parameter) enough?
		ROS_DEBUG("Setting timer for calling InitializeSerial() method");
		//nh->param("reconnect_delay_s", reconnect_delay_s_, 0.5f);
		//reconnect_timer_ = nh->createTimer(ros::Duration(reconnect_delay_s_), &ROSaicNode::Reconnect, this);
		//reconnect_timer_.start();
		//ros::spin(); // otherwise callback will never be called, with ros::spin i cannot leave InitializeIO(), yet with ros::spinOnce can enter reconnect() even once
		ROS_DEBUG("Current debug value before calling initializeserial() method is %u", io_comm_mosaic::debug);
		IO.InitializeSerial(device_, baudrate_);
		//ROS_DEBUG("Started timer"); //not printed if error in callback of course
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

void rosaic_node::ROSaicNode::Subscribe() 
{
	ROS_DEBUG("Entered subscribe() method");
	nh->param("publish/gpgga", publish_gpgga_, true);
	nh->param("publish/pvtcartesian", publish_pvtcartesian_, true);
	nh->param("publish/pvtgeodetic", publish_pvtgeodetic_, true);

	if (publish_gpgga_ == true)
	{
		IO.handlers_.callbacks_ = IO.get_handlers().insert<nmea_msgs::Gpgga>("$GPGGA", boost::bind(publish<nmea_msgs::Gpgga>, _1, "/gpgga"));
		
		//std::multimap<std::string, boost::shared_ptr<io_comm_mosaic::CallbackHandler> >::key_type key = "$GPGGA";
		//ROS_DEBUG("Back to subscribe() method: The element exists in our map: %u", (unsigned int) IO.handlers_.callbacks_.count(key));
	}
	if (publish_pvtcartesian_ == true)
	{
		IO.handlers_.callbacks_ = IO.get_handlers().insert<rosaic::PVTCartesian>("4006", boost::bind(publish<rosaic::PVTCartesian>, _1, "/pvtcartesian"));
		std::multimap<std::string, boost::shared_ptr<io_comm_mosaic::CallbackHandler> >::key_type key = "4006";
		ROS_DEBUG("Back to subscribe() method: The element exists in our map: %u", (unsigned int) IO.handlers_.callbacks_.count(key));
	}
	if (publish_pvtgeodetic_ == true)
	{
		IO.handlers_.callbacks_ = IO.get_handlers().insert<rosaic::PVTGeodetic>("4007", boost::bind(publish<rosaic::PVTGeodetic>, _1, "/pvtgeodetic"));
		std::multimap<std::string, boost::shared_ptr<io_comm_mosaic::CallbackHandler> >::key_type key = "4007";
		ROS_DEBUG("Back to subscribe() method: The element exists in our map: %u", (unsigned int) IO.handlers_.callbacks_.count(key));
	}
	ROS_DEBUG("Leaving subscribe() method");
	// and so on
}


// Declaring global variables..
bool use_GNSS_time;
//! Number of times the "read" method of the mosaicMessage class has been called
uint32_t read_count;
int io_comm_mosaic::debug; 
boost::mutex io_comm_mosaic::CallbackHandlers::callback_mutex_; 

int main(int argc, char** argv) 
{
	read_count = 0;
	ROS_DEBUG("About to call ROSaicNode constructor.."); // This will not be shown since info level seems to be default, hence modify momentarily..
	//rosaic_node::nh->param("?", node_name, default_node_name); 
	ros::init(argc, argv, "mosaic_gnss");
	ROS_DEBUG("Just called ROSaicNode constructor..");
	rosaic_node::nh.reset(new ros::NodeHandle("~")); // Note that nh was initialized in the header file already.
	rosaic_node::nh->param("debug", io_comm_mosaic::debug, 1); 
	rosaic_node::nh->param("use_GNSS_time", use_GNSS_time, true);
	rosaic_node::nh->param("frame_id", frame_id, (std::string) "gnss"); 
	ROS_DEBUG("Just loaded debug value to be %u from parameter server..", io_comm_mosaic::debug);

	// ros::NodeHandle param_nh("~");
	// std::string rtcm_topic;
	// param_nh.param("rtcm_topic", rtcm_topic, std::string("rtcm"));
	// subRTCM = nh->subscribe(rtcm_topic, 10, rtcmCallback);
  


	if(io_comm_mosaic::debug) //yields true if 1
	{
		ROS_DEBUG("Inside of if clause");
		// The following is the C++ version of rospy.init_node('my_ros_node', log_level=rospy.DEBUG)
		if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                       ros::console::levels::Debug)) //debug is lowest level, shows everything
		ros::console::notifyLoggerLevelsChanged();

	}
	ROS_DEBUG("Right before calling ROSaicNode constructor");
	rosaic_node::ROSaicNode mosaic_node; // This launches everything we need, in theory :)
	ROS_DEBUG("Leaving int main.");
	return 0;
}
