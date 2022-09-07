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

std::pair<std::string, uint32_t> navsatfix_pairs[] = {
    std::make_pair("4007", 0), std::make_pair("5906", 1), std::make_pair("4226", 2)};

std::pair<std::string, uint32_t> pose_pairs[] = {
    std::make_pair("4007", 0), std::make_pair("5906", 1), std::make_pair("5938", 2),
    std::make_pair("5939", 3), std::make_pair("4226", 4)};

std::pair<std::string, uint32_t> diagnosticarray_pairs[] = {
    std::make_pair("4014", 0), std::make_pair("4082", 1)};

std::pair<std::string, uint32_t> imu_pairs[] = {std::make_pair("4226", 0),
                                                std::make_pair("4050", 1)};

std::pair<std::string, uint32_t> localization_pairs[] = {std::make_pair("4226", 0)};

namespace io_comm_rx {
    boost::mutex CallbackHandlers::callback_mutex_;

    CallbackHandlers::GPSFixMap CallbackHandlers::gpsfix_map(gpsfix_pairs,
                                                             gpsfix_pairs + 9);
    CallbackHandlers::NavSatFixMap
        CallbackHandlers::navsatfix_map(navsatfix_pairs, navsatfix_pairs + 3);
    CallbackHandlers::PoseWithCovarianceStampedMap
        CallbackHandlers::pose_map(pose_pairs, pose_pairs + 5);
    CallbackHandlers::DiagnosticArrayMap
        CallbackHandlers::diagnosticarray_map(diagnosticarray_pairs,
                                              diagnosticarray_pairs + 2);
    CallbackHandlers::ImuMap CallbackHandlers::imu_map(imu_pairs, imu_pairs + 2);

    CallbackHandlers::LocalizationMap
        CallbackHandlers::localization_map(localization_pairs,
                                           localization_pairs + 1);

    std::string CallbackHandlers::do_gpsfix_ = "4007";
    std::string CallbackHandlers::do_navsatfix_ = "4007";
    std::string CallbackHandlers::do_pose_ = "4007";
    std::string CallbackHandlers::do_diagnostics_ = "4014";

    std::string CallbackHandlers::do_insgpsfix_ = "4226";
    std::string CallbackHandlers::do_insnavsatfix_ = "4226";
    std::string CallbackHandlers::do_inspose_ = "4226";
    std::string CallbackHandlers::do_imu_ = "4226";
    std::string CallbackHandlers::do_inslocalization_ = "4226";

    //! The for loop forwards to a ROS message specific handle if the latter was
    //! added via callbackmap_.insert at some earlier point.
    void CallbackHandlers::handle()
    {
        // Find the ROS message callback handler for the equivalent Rx message
        // (SBF/NMEA) at hand & call it
        boost::mutex::scoped_lock lock(callback_mutex_);
        CallbackMap::key_type key = rx_message_.messageID();
        std::string ID_temp = rx_message_.messageID();
        if (!(ID_temp == "4013" || ID_temp == "4001" || ID_temp == "4014" ||
              ID_temp == "4082"))
        // We only want to handle ChannelStatus, MeasEpoch, DOP, ReceiverStatus,
        // QualityInd blocks in case GPSFix and DiagnosticArray
        // messages are to be published, respectively, see few lines below.
        {
            for (CallbackMap::iterator callback = callbackmap_.lower_bound(key);
                 callback != callbackmap_.upper_bound(key); ++callback)
            {
                try
                {
                    callback->second->handle(rx_message_, callback->first);
                } catch (std::runtime_error& e)
                {
                    throw std::runtime_error(e.what());
                }
            }
        }

        // Call NavSatFix callback function if it was added for GNSS
        if (settings_->septentrio_receiver_type == "gnss")
        {
            if (settings_->publish_navsatfix)
            {
                CallbackMap::key_type key = "NavSatFix";
                std::string ID_temp = rx_message_.messageID();
                if (ID_temp == do_navsatfix_)
                // The last incoming block PVTGeodetic triggers
                // the publishing of NavSatFix.
                {
                    for (CallbackMap::iterator callback =
                             callbackmap_.lower_bound(key);
                         callback != callbackmap_.upper_bound(key); ++callback)
                    {
                        try
                        {
                            callback->second->handle(rx_message_, callback->first);
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
        if (settings_->septentrio_receiver_type == "ins")
        {
            if (settings_->publish_navsatfix)
            {
                CallbackMap::key_type key = "INSNavSatFix";
                std::string ID_temp = rx_message_.messageID();
                if (ID_temp == do_insnavsatfix_)
                // The last incoming block INSNavGeod triggers
                // the publishing of NavSatFix.
                {
                    for (CallbackMap::iterator callback =
                             callbackmap_.lower_bound(key);
                         callback != callbackmap_.upper_bound(key); ++callback)
                    {
                        try
                        {
                            callback->second->handle(rx_message_, callback->first);
                        } catch (std::runtime_error& e)
                        {
                            throw std::runtime_error(e.what());
                        }
                    }
                    do_insnavsatfix_ = std::string();
                }
            }
        }
        // Call PoseWithCovarianceStampedMsg callback function if it was
        // added for GNSS
        if (settings_->septentrio_receiver_type == "gnss")
        {
            if (settings_->publish_pose)
            {
                CallbackMap::key_type key = "PoseWithCovarianceStamped";
                std::string ID_temp = rx_message_.messageID();
                if (ID_temp == do_pose_)
                // The last incoming block among PVTGeodetic, PosCovGeodetic,
                // AttEuler and AttCovEuler triggers the publishing of
                // PoseWithCovarianceStamped.
                {
                    for (CallbackMap::iterator callback =
                             callbackmap_.lower_bound(key);
                         callback != callbackmap_.upper_bound(key); ++callback)
                    {
                        try
                        {
                            callback->second->handle(rx_message_, callback->first);
                        } catch (std::runtime_error& e)
                        {
                            throw std::runtime_error(e.what());
                        }
                    }
                    do_pose_ = std::string();
                }
            }
        }
        // Call PoseWithCovarianceStampedMsg callback function if it was
        // added for INS
        if (settings_->septentrio_receiver_type == "ins")
        {
            if (settings_->publish_pose)
            {
                CallbackMap::key_type key = "INSPoseWithCovarianceStamped";
                std::string ID_temp = rx_message_.messageID();
                if (ID_temp == do_inspose_)
                // The last incoming block INSNavGeod triggers the publishing of
                // PoseWithCovarianceStamped.
                {
                    for (CallbackMap::iterator callback =
                             callbackmap_.lower_bound(key);
                         callback != callbackmap_.upper_bound(key); ++callback)
                    {
                        try
                        {
                            callback->second->handle(rx_message_, callback->first);
                        } catch (std::runtime_error& e)
                        {
                            throw std::runtime_error(e.what());
                        }
                    }
                    do_inspose_ = std::string();
                }
            }
        }
        // Call DiagnosticArrayMsg callback function if it was added
        // for the both type of receivers
        if (settings_->publish_diagnostics)
        {
            CallbackMap::key_type key1 = rx_message_.messageID();
            std::string ID_temp = rx_message_.messageID();
            if (ID_temp == "4014" || ID_temp == "4082" || ID_temp == "5902")
            {
                for (CallbackMap::iterator callback = callbackmap_.lower_bound(key1);
                     callback != callbackmap_.upper_bound(key1); ++callback)
                {
                    try
                    {
                        callback->second->handle(rx_message_, callback->first);
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
                        callback->second->handle(rx_message_, callback->first);
                    } catch (std::runtime_error& e)
                    {
                        throw std::runtime_error(e.what());
                    }
                }
                do_diagnostics_ = std::string();
            }
        }
        // Call ImuMsg callback function if it was
        // added for INS
        if (settings_->septentrio_receiver_type == "ins")
        {
            if (settings_->publish_imu)
            {
                CallbackMap::key_type key = "Imu";
                std::string ID_temp = rx_message_.messageID();
                if (ID_temp == do_imu_)
                // The last incoming block INSNavGeod triggers the publishing of
                // PoseWithCovarianceStamped.
                {
                    for (CallbackMap::iterator callback =
                             callbackmap_.lower_bound(key);
                         callback != callbackmap_.upper_bound(key); ++callback)
                    {
                        try
                        {
                            callback->second->handle(rx_message_, callback->first);
                        } catch (std::runtime_error& e)
                        {
                            throw std::runtime_error(e.what());
                        }
                    }
                    do_imu_ = std::string();
                }
            }
        }
        // Call LocalizationMsg callback function if it was
        // added for INS
        if (settings_->septentrio_receiver_type == "ins")
        {
            if (settings_->publish_localization || settings_->publish_tf)
            {
                CallbackMap::key_type key = "Localization";
                std::string ID_temp = rx_message_.messageID();
                if (ID_temp == do_inslocalization_)
                // The last incoming block INSNavGeod triggers the publishing of
                // PoseWithCovarianceStamped.
                {
                    for (CallbackMap::iterator callback =
                             callbackmap_.lower_bound(key);
                         callback != callbackmap_.upper_bound(key); ++callback)
                    {
                        try
                        {
                            callback->second->handle(rx_message_, callback->first);
                        } catch (std::runtime_error& e)
                        {
                            throw std::runtime_error(e.what());
                        }
                    }
                    do_inslocalization_ = std::string();
                }
            }
        }
        // Call TimeReferenceMsg (with GPST) callback function if it was
        // added
        if (settings_->septentrio_receiver_type == "gnss")
        {
            if (settings_->publish_gpst)
            {
                CallbackMap::key_type key1 = "GPST";
                std::string ID_temp = rx_message_.messageID();
                // If no new PVTGeodetic block is coming in, there is no need to
                // publish TimeReferenceMsg (with GPST) anew.
                if (ID_temp == "4007")
                {
                    for (CallbackMap::iterator callback =
                             callbackmap_.lower_bound(key1);
                         callback != callbackmap_.upper_bound(key1); ++callback)
                    {
                        try
                        {
                            callback->second->handle(rx_message_, callback->first);
                        } catch (std::runtime_error& e)
                        {
                            throw std::runtime_error(e.what());
                        }
                    }
                }
            }
        }
        if (settings_->septentrio_receiver_type == "ins")
        {
            if (settings_->publish_gpst)
            {
                CallbackMap::key_type key1 = "GPST";
                std::string ID_temp = rx_message_.messageID();
                // If no new INSNavGeod block is coming in, there is no need to
                // publish TimeReferenceMsg (with GPST) anew.
                if (ID_temp == "4226")
                {
                    for (CallbackMap::iterator callback =
                             callbackmap_.lower_bound(key1);
                         callback != callbackmap_.upper_bound(key1); ++callback)
                    {
                        try
                        {
                            callback->second->handle(rx_message_, callback->first);
                        } catch (std::runtime_error& e)
                        {
                            throw std::runtime_error(e.what());
                        }
                    }
                }
            }
        }
        // Call GPSFix callback function if it was added for GNSS
        if (settings_->septentrio_receiver_type == "gnss")
        {
            if (settings_->publish_gpsfix)
            {
                std::string ID_temp = rx_message_.messageID();
                CallbackMap::key_type key1 = rx_message_.messageID();
                if (ID_temp == "4013" || ID_temp == "4001")
                // Even though we are not interested in publishing ChannelStatus
                // (4013) and DOP (4001) ROS messages, we have to save some contents
                // of these incoming blocks in order to publish the GPSFix message.
                {
                    for (CallbackMap::iterator callback =
                             callbackmap_.lower_bound(key1);
                         callback != callbackmap_.upper_bound(key1); ++callback)
                    {
                        try
                        {
                            callback->second->handle(rx_message_, callback->first);
                        } catch (std::runtime_error& e)
                        {
                            throw std::runtime_error(e.what());
                        }
                    }
                }
                CallbackMap::key_type key2 = "GPSFix";
                if (ID_temp == do_gpsfix_)
                // The last incoming block among ChannelStatus (4013), MeasEpoch
                // (4027), and DOP (4001) triggers the publishing of GPSFix.
                {
                    for (CallbackMap::iterator callback =
                             callbackmap_.lower_bound(key2);
                         callback != callbackmap_.upper_bound(key2); ++callback)
                    {
                        try
                        {
                            callback->second->handle(rx_message_, callback->first);
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
        if (settings_->septentrio_receiver_type == "ins")
        {
            if (settings_->publish_gpsfix)
            {
                std::string ID_temp = rx_message_.messageID();
                CallbackMap::key_type key1 = rx_message_.messageID();
                if (ID_temp == "4013" || ID_temp == "4001")
                // Even though we are not interested in publishing ChannelStatus
                // (4013) and DOP (4001) ROS messages, we have to save some contents
                // of these incoming blocks in order to publish the GPSFix message.
                {
                    for (CallbackMap::iterator callback =
                             callbackmap_.lower_bound(key1);
                         callback != callbackmap_.upper_bound(key1); ++callback)
                    {
                        try
                        {
                            callback->second->handle(rx_message_, callback->first);
                        } catch (std::runtime_error& e)
                        {
                            throw std::runtime_error(e.what());
                        }
                    }
                }
                CallbackMap::key_type key2 = "INSGPSFix";
                if (ID_temp == do_insgpsfix_)
                // The last incoming block among ChannelStatus (4013), MeasEpoch
                // (4027) and DOP (4001) triggers the publishing of INSGPSFix.
                {
                    for (CallbackMap::iterator callback =
                             callbackmap_.lower_bound(key2);
                         callback != callbackmap_.upper_bound(key2); ++callback)
                    {
                        try
                        {
                            callback->second->handle(rx_message_, callback->first);
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

    void CallbackHandlers::readCallback(Timestamp recvTimestamp, const uint8_t* data,
                                        std::size_t& size)
    {
        rx_message_.newData(recvTimestamp, data, size);
        // Read !all! (there might be many) messages in the buffer
        while (rx_message_.search() != rx_message_.getEndBuffer() &&
               rx_message_.found())
        {
            // Print the found message (if NMEA) or just show messageID (if SBF)..
            if (rx_message_.isSBF())
            {
                std::size_t sbf_block_length;
                std::string ID_temp = rx_message_.messageID();
                sbf_block_length =
                    static_cast<std::size_t>(rx_message_.getBlockLength());
                node_->log(LogLevel::DEBUG,
                           "ROSaic reading SBF block " + ID_temp + " made up of " +
                               std::to_string(sbf_block_length) + " bytes...");
                // If full message did not yet arrive, throw an error message.
                if (sbf_block_length > rx_message_.getCount())
                {
                    node_->log(
                        LogLevel::DEBUG,
                        "Not a valid SBF block, parts of the SBF block are yet to be received. Ignore..");
                    throw(
                        static_cast<std::size_t>(rx_message_.getPosBuffer() - data));
                }
                if (settings_->septentrio_receiver_type == "gnss")
                {
                    if (settings_->publish_gpsfix == true &&
                        (ID_temp == "4013" || ID_temp == "4027" ||
                         ID_temp == "4001" || ID_temp == "4007" ||
                         ID_temp == "5906" || ID_temp == "5908" ||
                         ID_temp == "5938" || ID_temp == "5939"))
                    {
                        if (rx_message_.gnss_gpsfix_complete(gpsfix_map[ID_temp]))
                        {
                            do_gpsfix_ = ID_temp;
                        }
                    }
                }
                if (settings_->septentrio_receiver_type == "ins")
                {
                    if (settings_->publish_gpsfix == true &&
                        (ID_temp == "4013" || ID_temp == "4027" ||
                         ID_temp == "4001" || ID_temp == "4226"))
                    {
                        if (rx_message_.ins_gpsfix_complete(gpsfix_map[ID_temp]))
                        {
                            do_insgpsfix_ = ID_temp;
                        }
                    }
                }
                if (settings_->septentrio_receiver_type == "gnss")
                {
                    if (settings_->publish_navsatfix == true &&
                        (ID_temp == "4007" || ID_temp == "5906"))
                    {
                        if (rx_message_.gnss_navsatfix_complete(
                                navsatfix_map[ID_temp]))
                        {
                            do_navsatfix_ = ID_temp;
                        }
                    }
                }
                if (settings_->septentrio_receiver_type == "ins")
                {
                    if (settings_->publish_navsatfix == true && (ID_temp == "4226"))
                    {
                        if (rx_message_.ins_navsatfix_complete(
                                navsatfix_map[ID_temp]))
                        {
                            do_insnavsatfix_ = ID_temp;
                        }
                    }
                }
                if (settings_->septentrio_receiver_type == "gnss")
                {
                    if (settings_->publish_pose == true &&
                        (ID_temp == "4007" || ID_temp == "5906" ||
                         ID_temp == "5938" || ID_temp == "5939"))
                    {
                        if (rx_message_.gnss_pose_complete(pose_map[ID_temp]))
                        {
                            do_pose_ = ID_temp;
                        }
                    }
                }
                if (settings_->septentrio_receiver_type == "ins")
                {
                    if (settings_->publish_pose == true && (ID_temp == "4226"))
                    {
                        if (rx_message_.ins_pose_complete(pose_map[ID_temp]))
                        {
                            do_inspose_ = ID_temp;
                        }
                    }
                }
                if (settings_->publish_diagnostics == true &&
                    (ID_temp == "4014" || ID_temp == "4082"))
                {
                    if (rx_message_.diagnostics_complete(
                            diagnosticarray_map[ID_temp]))
                    {
                        do_diagnostics_ = ID_temp;
                    }
                }
                if ((settings_->publish_localization || settings_->publish_tf) &&
                    (ID_temp == "4226"))
                {
                    if (rx_message_.ins_localization_complete(
                            localization_map[ID_temp]))
                    {
                        do_inslocalization_ = ID_temp;
                    }
                }
            }
            if (rx_message_.isNMEA())
            {
                boost::char_separator<char> sep("\r"); // Carriage Return (CR)
                typedef boost::tokenizer<boost::char_separator<char>> tokenizer;
                std::size_t nmea_size = rx_message_.messageSize();
                // Syntax: new_string_name (const char* s, size_t n); size_t is
                // either 2 or 8 bytes, depending on your system
                std::string block_in_string(
                    reinterpret_cast<const char*>(rx_message_.getPosBuffer()),
                    nmea_size);
                tokenizer tokens(block_in_string, sep);
                node_->log(LogLevel::DEBUG,
                           "The NMEA message contains " + std::to_string(nmea_size) +
                               " bytes and is ready to be parsed. It reads: " +
                               *tokens.begin());
            }
            if (rx_message_.isResponse()) // If the response is not sent at once,
                                          // only first part is ROS_DEBUG-printed
            {
                std::size_t response_size = rx_message_.messageSize();
                std::string block_in_string(
                    reinterpret_cast<const char*>(rx_message_.getPosBuffer()),
                    response_size);
                node_->log(LogLevel::DEBUG, "The Rx's response contains " +
                                                std::to_string(response_size) +
                                                " bytes and reads:\n " +
                                                block_in_string);
                {
                    boost::mutex::scoped_lock lock(g_response_mutex);
                    g_response_received = true;
                    lock.unlock();
                    g_response_condition.notify_one();
                }
                if (rx_message_.isErrorMessage())
                {
                    node_->log(
                        LogLevel::ERROR,
                        "Invalid command just sent to the Rx! The Rx's response contains " +
                            std::to_string(response_size) + " bytes and reads:\n " +
                            block_in_string);
                }
                continue;
            }
            if (rx_message_.isConnectionDescriptor())
            {
                std::string cd(
                    reinterpret_cast<const char*>(rx_message_.getPosBuffer()), 4);
                g_rx_tcp_port = cd;
                if (g_cd_count == 0)
                {
                    node_->log(
                        LogLevel::INFO,
                        "The connection descriptor for the TCP connection is " + cd);
                }
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
                handle();
            } catch (std::runtime_error& e)
            {
                node_->log(LogLevel::DEBUG,
                           "Incomplete message: " + std::string(e.what()));
                throw(static_cast<std::size_t>(rx_message_.getPosBuffer() - data));
            }
        }
    }
} // namespace io_comm_rx