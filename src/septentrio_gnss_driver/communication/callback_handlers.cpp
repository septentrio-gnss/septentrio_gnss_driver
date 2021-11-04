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

#include <septentrio_gnss_driver/communication/callback_handlers.hpp>

/**
 * @file callback_handlers.cpp
 * @date 22/08/20
 * @brief Handles callbacks when reading NMEA/SBF messages
 */

std::pair<std::string, uint32_t> gpsfix_pairs[] = {
    std::make_pair("4013", 0), std::make_pair("4027", 1), std::make_pair("4001", 2),
    std::make_pair("4007", 3), std::make_pair("5906", 4), std::make_pair("5908", 5),
    std::make_pair("5938", 6), std::make_pair("5939", 7), std::make_pair("4226", 8)};

std::pair<std::string, uint32_t> navsatfix_pairs[] = {std::make_pair("4007", 0),
                                                      std::make_pair("5906", 1),
                                                      std::make_pair("4226",2)};

std::pair<std::string, uint32_t> pose_pairs[] = {
    std::make_pair("4007", 0), std::make_pair("5906", 1), std::make_pair("5938", 2),
    std::make_pair("5939", 3), std::make_pair("4226",4)};

std::pair<std::string, uint32_t> diagnosticarray_pairs[] = {
    std::make_pair("4014", 0), std::make_pair("4082", 1)};

namespace io_comm_rx {
    boost::mutex CallbackHandlers::callback_mutex_;

    CallbackHandlers::GPSFixMap 
	    CallbackHandlers::gpsfix_map(gpsfix_pairs, gpsfix_pairs + 9);
    CallbackHandlers::NavSatFixMap
        CallbackHandlers::navsatfix_map(navsatfix_pairs, navsatfix_pairs + 3);
    CallbackHandlers::PoseWithCovarianceStampedMap
        CallbackHandlers::pose_map(pose_pairs, pose_pairs + 5);
    CallbackHandlers::DiagnosticArrayMap
        CallbackHandlers::diagnosticarray_map(diagnosticarray_pairs,
                                              diagnosticarray_pairs + 2);

    std::string CallbackHandlers::do_gpsfix_ = "4007";
    std::string CallbackHandlers::do_navsatfix_ = "4007";
    std::string CallbackHandlers::do_pose_ = "4007";
    std::string CallbackHandlers::do_diagnostics_ = "4014";
    
    std::string CallbackHandlers::do_insgpsfix_ = "4226";
    std::string CallbackHandlers::do_insnavsatfix_ = "4226"; 
    std::string CallbackHandlers::do_inspose_ = "4226";  

    //! The for loop forwards to a ROS message specific handle if the latter was
    //! added via callbackmap_.insert at some earlier point.
    void CallbackHandlers::handle(RxMessage& rx_message)
    {
        // Find the ROS message callback handler for the equivalent Rx message
        // (SBF/NMEA) at hand & call it
        boost::mutex::scoped_lock lock(callback_mutex_);
        CallbackMap::key_type key = rx_message.messageID();
        std::string ID_temp = rx_message.messageID();
        if (!(ID_temp == "4013" || ID_temp == "4027" || ID_temp == "4001" ||
              ID_temp == "4014" || ID_temp == "4082" || ID_temp == "5902"))
        // We only want to handle ChannelStatus, MeasEpoch, DOP, ReceiverStatus, 
        // QualityInd and ReceiverSetup blocks in case GPSFix and DiagnosticArray 
        // messages are to be published, respectively, see few lines below.
        {
            for (CallbackMap::iterator callback = callbackmap_.lower_bound(key);
                 callback != callbackmap_.upper_bound(key); ++callback)
            {
                try
                {
                    callback->second->handle(rx_message, callback->first);
                } catch (std::runtime_error& e)
                {
                    throw std::runtime_error(e.what());
                }
            }
        }
        // Call NavSatFix callback function if it was added for GNSS 
		if (septentrio_receiver_type_ == "gnss")
		{
			if (g_publish_navsatfix)
			{
				CallbackMap::key_type key = "NavSatFix";
				std::string ID_temp = rx_message.messageID();
				if (ID_temp == do_navsatfix_)
				// The last incoming block PVTGeodetic triggers
				// the publishing of NavSatFix.
				{
					for (CallbackMap::iterator callback = callbackmap_.lower_bound(key);
						callback != callbackmap_.upper_bound(key); ++callback)
					{
						try
						{
							callback->second->handle(rx_message, callback->first);
						} catch (std::runtime_error& e)
						{
							throw std::runtime_error(e.what());
						}
					}
					do_navsatfix_ = std::string();
				}
			}
		}
        // Call NavSatFix callback function if it was added for INS
        if (septentrio_receiver_type_ == "ins")
        {
            if (g_publish_navsatfix)
            {
                CallbackMap::key_type key = "INSNavSatFix";
                std::string ID_temp = rx_message.messageID();
                if (ID_temp == do_insnavsatfix_)
                // The last incoming block INSNavGeod triggers
                // the publishing of NavSatFix.
                {
                    for (CallbackMap::iterator callback = callbackmap_.lower_bound(key);
                        callback != callbackmap_.upper_bound(key); ++callback)
                    {
                        try
                        {
                            callback->second->handle(rx_message, callback->first);
                        } catch (std::runtime_error& e)
                        {
                            throw std::runtime_error(e.what());
                        }
                    }
                    do_insnavsatfix_ = std::string();
                }
            }
        }
        // Call geometry_msgs::PoseWithCovarianceStamped callback function if it was
        // added for GNSS
        if (septentrio_receiver_type_ == "gnss")
        {
            if (g_publish_pose)
            {
                CallbackMap::key_type key = "PoseWithCovarianceStamped";
                std::string ID_temp = rx_message.messageID();
                if (ID_temp == do_pose_)
                // The last incoming block among PVTGeodetic, PosCovGeodetic, AttEuler
                // and AttCovEuler triggers the publishing of PoseWithCovarianceStamped.
                {
                    for (CallbackMap::iterator callback = callbackmap_.lower_bound(key);
                        callback != callbackmap_.upper_bound(key); ++callback)
                    {
                        try
                        {
                            callback->second->handle(rx_message, callback->first);
                        } catch (std::runtime_error& e)
                        {
                            throw std::runtime_error(e.what());
                        }
                    }
                    do_pose_ = std::string();
                }
            }
        }
        // Call geometry_msgs::PoseWithCovarianceStamped callback function if it was
        // added for INS
        if (septentrio_receiver_type_ == "ins")
        {
            if (g_publish_pose)
            {
                CallbackMap::key_type key = "INSPoseWithCovarianceStamped";
                std::string ID_temp = rx_message.messageID();
                if (ID_temp == do_inspose_)
                // The last incoming block INSNavGeod triggers the publishing of PoseWithCovarianceStamped.
                {
                    for (CallbackMap::iterator callback = callbackmap_.lower_bound(key);
                        callback != callbackmap_.upper_bound(key); ++callback)
                    {
                        try
                        {
                            callback->second->handle(rx_message, callback->first);
                        } catch (std::runtime_error& e)
                        {
                            throw std::runtime_error(e.what());
                        }
                    }
                    do_inspose_ = std::string();
                }
            }
        }
        // Call diagnostic_msgs::DiagnosticArray callback function if it was added
        // for the both type of receivers
		if (g_publish_diagnostics)
		{
			CallbackMap::key_type key1 = rx_message.messageID();
			std::string ID_temp = rx_message.messageID();
			if (ID_temp == "4014" || ID_temp == "4082" || ID_temp == "5902")
			{
				for (CallbackMap::iterator callback = callbackmap_.lower_bound(key1);
					callback != callbackmap_.upper_bound(key1); ++callback)
				{
					try
					{
						callback->second->handle(rx_message, callback->first);
					} catch (std::runtime_error& e)
					{
						throw std::runtime_error(e.what());
					}
				}
			}
			CallbackMap::key_type key2 = "DiagnosticArray";
			if (ID_temp == do_diagnostics_)
			// The last incoming block among ReceiverStatus, QualityInd and
			// ReceiverSetup triggers the publishing of DiagnosticArray.
			{
				for (CallbackMap::iterator callback = callbackmap_.lower_bound(key2);
					callback != callbackmap_.upper_bound(key2); ++callback)
				{
					try
					{
						callback->second->handle(rx_message, callback->first);
					} catch (std::runtime_error& e)
					{
						throw std::runtime_error(e.what());
					}
				}
				do_diagnostics_ = std::string();
			}
		}
        // Call sensor_msgs::TimeReference (with GPST) callback function if it was
        // added
        if (septentrio_receiver_type_ == "gnss")
        {
            if (g_publish_gpst)
            {
                CallbackMap::key_type key1 = "GPST";
                std::string ID_temp = rx_message.messageID();
                // If no new PVTGeodetic block is coming in, there is no need to publish
                // sensor_msgs::TimeReference (with GPST) anew.
                if (ID_temp == "4007")
                {
                    for (CallbackMap::iterator callback = callbackmap_.lower_bound(key1);
                        callback != callbackmap_.upper_bound(key1); ++callback)
                    {
                        try
                        {
                            callback->second->handle(rx_message, callback->first);
                        } catch (std::runtime_error& e)
                        {
                            throw std::runtime_error(e.what());
                        }
                    }
                }
            }
        }
        if (septentrio_receiver_type_ == "ins")
        {
            if (g_publish_gpst)
            {
                CallbackMap::key_type key1 = "GPST";
                std::string ID_temp = rx_message.messageID();
                // If no new INSNavGeod block is coming in, there is no need to publish
                // sensor_msgs::TimeReference (with GPST) anew.
                if (ID_temp == "4226")
                {
                    for (CallbackMap::iterator callback = callbackmap_.lower_bound(key1);
                        callback != callbackmap_.upper_bound(key1); ++callback)
                    {
                        try
                        {
                            callback->second->handle(rx_message, callback->first);
                        } catch (std::runtime_error& e)
                        {
                            throw std::runtime_error(e.what());
                        }
                    }
                }
            }
        }
        // Call GPSFix callback function if it was added for GNSS
        if (septentrio_receiver_type_ == "gnss")
        {
            if (g_publish_gpsfix)
            {
                std::string ID_temp = rx_message.messageID();
                CallbackMap::key_type key1 = rx_message.messageID();
                if (ID_temp == "4013" || ID_temp == "4027" || ID_temp == "4001")
                // Even though we are not interested in publishing ChannelStatus (4013),
                // MeasEpoch (4027), and DOP (4001) ROS messages, we have to save some 
                // contents of these incoming blocks in order to publish the GPSFix message.
                {
                    for (CallbackMap::iterator callback = callbackmap_.lower_bound(key1);
                        callback != callbackmap_.upper_bound(key1); ++callback)
                    {
                        try
                        {
                            callback->second->handle(rx_message, callback->first);
                        } catch (std::runtime_error& e)
                        {
                            throw std::runtime_error(e.what());
                        }
                    }
                }
                CallbackMap::key_type key2 = "GPSFix";
                if (ID_temp == do_gpsfix_)
                // The last incoming block among ChannelStatus (4013), MeasEpoch (4027),
                // and DOP (4001) triggers the publishing of GPSFix.
                {
                    for (CallbackMap::iterator callback = callbackmap_.lower_bound(key2);
                        callback != callbackmap_.upper_bound(key2); ++callback)
                    {
                        try
                        {
                            callback->second->handle(rx_message, callback->first);
                        } catch (std::runtime_error& e)
                        {
                            throw std::runtime_error(e.what());
                        }
                    }
                    do_gpsfix_ = std::string();
                }
            }
        }
        // Call GPSFix callback function if it was added for INS
        if (septentrio_receiver_type_ == "ins")
        {
            if (g_publish_gpsfix)
            {
                std::string ID_temp = rx_message.messageID();
                CallbackMap::key_type key1 = rx_message.messageID();
                if (ID_temp == "4013" || ID_temp == "4027" || ID_temp == "4001")
                // Even though we are not interested in publishing ChannelStatus (4013),
                // MeasEpoch (4027) and DOP (4001) ROS messages,
                // we have to save some contents of these incoming blocks in order to
                // publish the GPSFix message.
                {
                    for (CallbackMap::iterator callback = callbackmap_.lower_bound(key1);
                        callback != callbackmap_.upper_bound(key1); ++callback)
                    {
                        try
                        {
                            callback->second->handle(rx_message, callback->first);
                        } catch (std::runtime_error& e)
                        {
                            throw std::runtime_error(e.what());
                        }
                    }
                }
                CallbackMap::key_type key2 = "INSGPSFix";
                if (ID_temp == do_insgpsfix_)
                // The last incoming block among ChannelStatus (4013), MeasEpoch (4027) and
                // DOP (4001) triggers the publishing of
                // INSGPSFix.
                {
                    for (CallbackMap::iterator callback = callbackmap_.lower_bound(key2);
                        callback != callbackmap_.upper_bound(key2); ++callback)
                    {
                        try
                        {
                            callback->second->handle(rx_message, callback->first);
                        } catch (std::runtime_error& e)
                        {
                            throw std::runtime_error(e.what());
                        }
                    }
                    do_insgpsfix_ = std::string();
                }
            }
        }
    }

    void CallbackHandlers::readCallback(const uint8_t* data, std::size_t& size)
    {
        RxMessage rx_message(data, size);
        // Read !all! (there might be many) messages in the buffer
        while (rx_message.search() != rx_message.getEndBuffer() &&
               rx_message.found())
        {
            // Print the found message (if NMEA) or just show messageID (if SBF)..
            if (rx_message.isSBF())
            {
                std::size_t sbf_block_length;
                std::string ID_temp = rx_message.messageID();
                sbf_block_length =
                    static_cast<std::size_t>(rx_message.getBlockLength());
                ROS_DEBUG("ROSaic reading SBF block %s made up of %li bytes...",
                          ID_temp.c_str(), sbf_block_length);
                // If full message did not yet arrive, throw an error message.
                if (sbf_block_length > rx_message.getCount())
                {
                    ROS_DEBUG(
                        "Not a valid SBF block, parts of the SBF block are yet to be received. Ignore..");
                    throw(
                        static_cast<std::size_t>(rx_message.getPosBuffer() - data));
                }
                if (septentrio_receiver_type_ == "gnss")
                {
                    if (g_publish_gpsfix == true &&
                    (ID_temp == "4013" || ID_temp == "4027" || ID_temp == "4001" ||
                     ID_temp == "4007" || ID_temp == "5906" || ID_temp == "5908" ||
                     ID_temp == "5938" || ID_temp == "5939"))
                    {
                        std::vector<bool> gpsfix_vec = {
                            g_channelstatus_has_arrived_gpsfix,
                            g_measepoch_has_arrived_gpsfix,
                            g_dop_has_arrived_gpsfix,
                            g_pvtgeodetic_has_arrived_gpsfix,
                            g_poscovgeodetic_has_arrived_gpsfix,
                            g_velcovgeodetic_has_arrived_gpsfix,
                            g_atteuler_has_arrived_gpsfix,
                            g_attcoveuler_has_arrived_gpsfix};
                        gpsfix_vec.erase(gpsfix_vec.begin() + gpsfix_map[ID_temp]);
                        // Checks whether all entries in gpsfix_vec are true
                        if (std::all_of(gpsfix_vec.begin(), gpsfix_vec.end(),
                                        [](bool v) { return v; }) == true)
                        {
                            do_gpsfix_ = ID_temp;
                        }
                    }
                }
                if (septentrio_receiver_type_ == "ins")
                {
                    if (g_publish_gpsfix == true &&
                    (ID_temp == "4013" || ID_temp == "4027" || ID_temp == "4001" || ID_temp == "4226"))
                    {
                        std::vector<bool> gpsfix_vec = {
                            g_channelstatus_has_arrived_gpsfix,
                            g_measepoch_has_arrived_gpsfix,
                            g_dop_has_arrived_gpsfix,
                            g_insnavgeod_has_arrived_gpsfix};
                        gpsfix_vec.erase(gpsfix_vec.begin() + gpsfix_map[ID_temp]);
                        // Checks whether all entries in gpsfix_vec are true
                        if (std::all_of(gpsfix_vec.begin(), gpsfix_vec.end(),
                                        [](bool v) { return v; }) == true)
                        {
                            do_insgpsfix_ = ID_temp;
                        }
                    }
                }
                if (septentrio_receiver_type_ == "gnss")
                {
                    if (g_publish_navsatfix == true &&
                    (ID_temp == "4007" || ID_temp == "5906"))
                    {
                        std::vector<bool> navsatfix_vec = {
                            g_pvtgeodetic_has_arrived_navsatfix,
                            g_poscovgeodetic_has_arrived_navsatfix};
                        navsatfix_vec.erase(navsatfix_vec.begin() +
                                            navsatfix_map[ID_temp]);
                        // Checks whether all entries in navsatfix_vec are true
                        if (std::all_of(navsatfix_vec.begin(), navsatfix_vec.end(),
                                        [](bool v) { return v; }) == true)
                        {
                            do_navsatfix_ = ID_temp;
                        }
                    }
                }
                if (septentrio_receiver_type_ == "ins")
                {
                    if (g_publish_navsatfix == true &&
                    (ID_temp == "4226"))
                    {
                        std::vector<bool> navsatfix_vec = {
                            g_insnavgeod_has_arrived_navsatfix};
                        navsatfix_vec.erase(navsatfix_vec.begin() +
                                            navsatfix_map[ID_temp]);
                        // Checks whether all entries in navsatfix_vec are true
                        if (std::all_of(navsatfix_vec.begin(), navsatfix_vec.end(),
                                        [](bool v) { return v; }) == true)
                        {
                            do_insnavsatfix_ = ID_temp;
                        }
                    }
                }
                if (septentrio_receiver_type_ == "gnss")
                {
                    if (g_publish_pose == true &&
                    (ID_temp == "4007" || ID_temp == "5906" || ID_temp == "5938" ||
                     ID_temp == "5939"))
                    {
                        std::vector<bool> pose_vec = {g_pvtgeodetic_has_arrived_pose,
                                                    g_poscovgeodetic_has_arrived_pose,
                                                    g_atteuler_has_arrived_pose,
                                                    g_attcoveuler_has_arrived_pose,};
                        pose_vec.erase(pose_vec.begin() + pose_map[ID_temp]);
                        // Checks whether all entries in pose_vec are true
                        if (std::all_of(pose_vec.begin(), pose_vec.end(),
                                        [](bool v) { return v; }) == true)
                        {
                            do_pose_ = ID_temp;
                        }
                    }
                }
                if (septentrio_receiver_type_ == "ins")
                {
                    if (g_publish_pose == true &&
                    (ID_temp == "4226"))
                    {
                        std::vector<bool> pose_vec = {
                                                    g_insnavgeod_has_arrived_pose};
                        pose_vec.erase(pose_vec.begin() + pose_map[ID_temp]);
                        // Checks whether all entries in pose_vec are true
                        if (std::all_of(pose_vec.begin(), pose_vec.end(),
                                        [](bool v) { return v; }) == true)
                        {
                            do_inspose_ = ID_temp;
                        }
                    }
                }
				if (g_publish_diagnostics == true &&
				(ID_temp == "4014" || ID_temp == "4082"))
				{
					std::vector<bool> diagnostics_vec = {
						g_receiverstatus_has_arrived_diagnostics,
						g_qualityind_has_arrived_diagnostics};
					diagnostics_vec.erase(diagnostics_vec.begin() +
										diagnosticarray_map[ID_temp]);
					// Checks whether all entries in diagnostics_vec are true
					if (std::all_of(diagnostics_vec.begin(), diagnostics_vec.end(),
									[](bool v) { return v; }) == true)
					{
						do_diagnostics_ = ID_temp;
					}
				}
            }
            if (rx_message.isNMEA())
            {
                boost::char_separator<char> sep("\r"); // Carriage Return (CR)
                typedef boost::tokenizer<boost::char_separator<char>> tokenizer;
                std::size_t nmea_size = rx_message.messageSize();
                // Syntax: new_string_name (const char* s, size_t n); size_t is
                // either 2 or 8 bytes, depending on your system
                std::string block_in_string(
                    reinterpret_cast<const char*>(rx_message.getPosBuffer()),
                    nmea_size);
                tokenizer tokens(block_in_string, sep);
                ROS_DEBUG(
                    "The NMEA message contains %li bytes and is ready to be parsed. It reads: %s",
                    nmea_size, (*tokens.begin()).c_str());
            }
            if (rx_message.isResponse()) // If the response is not sent at once, only
                                         // first part is ROS_DEBUG-printed
            {
                std::size_t response_size = rx_message.messageSize();
                std::string block_in_string(
                    reinterpret_cast<const char*>(rx_message.getPosBuffer()),
                    response_size);
                ROS_DEBUG("The Rx's response contains %li bytes and reads:\n %s",
                          response_size, block_in_string.c_str());
                {
                    boost::mutex::scoped_lock lock(g_response_mutex);
                    g_response_received = true;
                    lock.unlock();
                    g_response_condition.notify_one();
                }
                if (rx_message.isErrorMessage())
                {
                    ROS_ERROR("Invalid command just sent to the Rx!");
                }
                continue;
            }
            if (rx_message.isConnectionDescriptor())
            {
                std::string cd(
                    reinterpret_cast<const char*>(rx_message.getPosBuffer()), 4);
                g_rx_tcp_port = cd;
                ROS_INFO_COND(
                    g_cd_count == 0,
                    "The connection descriptor for the TCP connection is %s",
                    cd.c_str());
                if (g_cd_count < 3)
                    ++g_cd_count;
                if (g_cd_count == 2)
                {
                    boost::mutex::scoped_lock lock(g_cd_mutex);
                    g_cd_received = true;
                    lock.unlock();
                    g_cd_condition.notify_one();
                }
                continue;
            }
            try
            {
                handle(rx_message);
            } catch (std::runtime_error& e)
            {
                ROS_DEBUG("Incomplete message: %s", e.what());
                throw(static_cast<std::size_t>(rx_message.getPosBuffer() - data));
            }
        }
    }
} // namespace io_comm_rx