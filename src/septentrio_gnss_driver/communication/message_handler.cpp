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

#include <GeographicLib/UTMUPS.hpp>
#include <boost/tokenizer.hpp>
#include <septentrio_gnss_driver/communication/message_handler.hpp>
#include <thread>

/**
 * The position_covariance array is populated in row-major order, where the basis of
 * the correspond matrix is (E, N, U, Roll, Pitch, Heading). Important: The Euler
 * angles (Roll, Pitch, Heading) are with respect to a vehicle-fixed (e.g. for
 * mosaic-x5 in moving base mode via the command setAntennaLocation, ...) !local! NED
 * frame or ENU frame if use_ros_axis_directions is set true Thus the orientation
 * is !not! given with respect to the same frame as the position is given in. The
 * cross-covariances are hence (apart from the fact that e.g. mosaic receivers do
 * not calculate these quantities) set to zero. The position and the partial
 * (with 2 antennas) or full (for INS receivers) orientation have covariance matrices
 * available e.g. in the PosCovGeodetic or AttCovEuler blocks, yet those are separate
 * computations.
 */

using parsing_utilities::convertEulerToQuaternionMsg;
using parsing_utilities::deg2rad;
using parsing_utilities::deg2radSq;
using parsing_utilities::rad2deg;
using parsing_utilities::square;

namespace io {
    void MessageHandler::assemblePoseWithCovarianceStamped()
    {
        if (!settings_->publish_pose)
            return;

        static auto last_ins_tow = last_insnavgeod_.block_header.tow;

        PoseWithCovarianceStampedMsg msg;
        if (settings_->septentrio_receiver_type == "ins")
        {
            if (!validValue(last_insnavgeod_.block_header.tow) ||
                (last_insnavgeod_.block_header.tow == last_ins_tow))
                return;
            last_ins_tow = last_insnavgeod_.block_header.tow;

            msg.header = last_insnavgeod_.header;

            msg.pose.pose.position.x = rad2deg(last_insnavgeod_.longitude);
            msg.pose.pose.position.y = rad2deg(last_insnavgeod_.latitude);
            msg.pose.pose.position.z = last_insnavgeod_.height;

            // Filling in the pose data
            if ((last_insnavgeod_.sb_list & 1) != 0)
            {
                // Pos autocov
                msg.pose.covariance[0] = square(last_insnavgeod_.longitude_std_dev);
                msg.pose.covariance[7] = square(last_insnavgeod_.latitude_std_dev);
                msg.pose.covariance[14] = square(last_insnavgeod_.height_std_dev);
            } else
            {
                msg.pose.covariance[0] = -1.0;
                msg.pose.covariance[7] = -1.0;
                msg.pose.covariance[14] = -1.0;
            }
            if ((last_insnavgeod_.sb_list & 2) != 0)
            {
                double yaw = 0.0;
                if (validValue(last_insnavgeod_.heading))
                    yaw = last_insnavgeod_.heading;
                double pitch = 0.0;
                if (validValue(last_insnavgeod_.pitch))
                    pitch = last_insnavgeod_.pitch;
                double roll = 0.0;
                if (validValue(last_insnavgeod_.roll))
                    roll = last_insnavgeod_.roll;
                // Attitude
                msg.pose.pose.orientation = convertEulerToQuaternionMsg(
                    deg2rad(roll), deg2rad(pitch), deg2rad(yaw));
            } else
            {
                msg.pose.pose.orientation.w =
                    std::numeric_limits<double>::quiet_NaN();
                msg.pose.pose.orientation.x =
                    std::numeric_limits<double>::quiet_NaN();
                msg.pose.pose.orientation.y =
                    std::numeric_limits<double>::quiet_NaN();
                msg.pose.pose.orientation.z =
                    std::numeric_limits<double>::quiet_NaN();
            }
            if ((last_insnavgeod_.sb_list & 4) != 0)
            {
                // Attitude autocov
                if (validValue(last_insnavgeod_.roll_std_dev))
                    msg.pose.covariance[21] =
                        square(deg2rad(last_insnavgeod_.roll_std_dev));
                else
                    msg.pose.covariance[21] = -1.0;
                if (validValue(last_insnavgeod_.pitch_std_dev))
                    msg.pose.covariance[28] =
                        square(deg2rad(last_insnavgeod_.pitch_std_dev));
                else
                    msg.pose.covariance[28] = -1.0;
                if (validValue(last_insnavgeod_.heading_std_dev))
                    msg.pose.covariance[35] =
                        square(deg2rad(last_insnavgeod_.heading_std_dev));
                else
                    msg.pose.covariance[35] = -1.0;
            } else
            {
                msg.pose.covariance[21] = -1.0;
                msg.pose.covariance[28] = -1.0;
                msg.pose.covariance[35] = -1.0;
            }
            if ((last_insnavgeod_.sb_list & 32) != 0)
            {
                // Pos cov
                msg.pose.covariance[1] = last_insnavgeod_.latitude_longitude_cov;
                msg.pose.covariance[2] = last_insnavgeod_.longitude_height_cov;
                msg.pose.covariance[6] = last_insnavgeod_.latitude_longitude_cov;
                msg.pose.covariance[8] = last_insnavgeod_.latitude_height_cov;
                msg.pose.covariance[12] = last_insnavgeod_.longitude_height_cov;
                msg.pose.covariance[13] = last_insnavgeod_.latitude_height_cov;
            }
            if ((last_insnavgeod_.sb_list & 64) != 0)
            {
                // Attitude cov
                msg.pose.covariance[22] = deg2radSq(last_insnavgeod_.pitch_roll_cov);
                msg.pose.covariance[23] =
                    deg2radSq(last_insnavgeod_.heading_roll_cov);
                msg.pose.covariance[27] = deg2radSq(last_insnavgeod_.pitch_roll_cov);

                msg.pose.covariance[29] =
                    deg2radSq(last_insnavgeod_.heading_pitch_cov);
                msg.pose.covariance[33] =
                    deg2radSq(last_insnavgeod_.heading_roll_cov);
                msg.pose.covariance[34] =
                    deg2radSq(last_insnavgeod_.heading_pitch_cov);
            }
        } else
        {
            if ((!validValue(last_pvtgeodetic_.block_header.tow)) ||
                (last_pvtgeodetic_.block_header.tow !=
                 last_atteuler_.block_header.tow) ||
                (last_pvtgeodetic_.block_header.tow !=
                 last_poscovgeodetic_.block_header.tow) ||
                (last_pvtgeodetic_.block_header.tow !=
                 last_attcoveuler_.block_header.tow))
                return;

            msg.header = last_pvtgeodetic_.header;

            // Filling in the pose data
            double yaw = 0.0;
            if (validValue(last_atteuler_.heading))
                yaw = last_atteuler_.heading;
            double pitch = 0.0;
            if (validValue(last_atteuler_.pitch))
                pitch = last_atteuler_.pitch;
            double roll = 0.0;
            if (validValue(last_atteuler_.roll))
                roll = last_atteuler_.roll;
            msg.pose.pose.orientation = convertEulerToQuaternionMsg(
                deg2rad(roll), deg2rad(pitch), deg2rad(yaw));
            msg.pose.pose.position.x = rad2deg(last_pvtgeodetic_.longitude);
            msg.pose.pose.position.y = rad2deg(last_pvtgeodetic_.latitude);
            msg.pose.pose.position.z = last_pvtgeodetic_.height;
            // Filling in the covariance data in row-major order
            msg.pose.covariance[0] = last_poscovgeodetic_.cov_lonlon;
            msg.pose.covariance[1] = last_poscovgeodetic_.cov_latlon;
            msg.pose.covariance[2] = last_poscovgeodetic_.cov_lonhgt;
            msg.pose.covariance[6] = last_poscovgeodetic_.cov_latlon;
            msg.pose.covariance[7] = last_poscovgeodetic_.cov_latlat;
            msg.pose.covariance[8] = last_poscovgeodetic_.cov_lathgt;
            msg.pose.covariance[12] = last_poscovgeodetic_.cov_lonhgt;
            msg.pose.covariance[13] = last_poscovgeodetic_.cov_lathgt;
            msg.pose.covariance[14] = last_poscovgeodetic_.cov_hgthgt;
            msg.pose.covariance[21] = deg2radSq(last_attcoveuler_.cov_rollroll);
            msg.pose.covariance[22] = deg2radSq(last_attcoveuler_.cov_pitchroll);
            msg.pose.covariance[23] = deg2radSq(last_attcoveuler_.cov_headroll);
            msg.pose.covariance[27] = deg2radSq(last_attcoveuler_.cov_pitchroll);
            msg.pose.covariance[28] = deg2radSq(last_attcoveuler_.cov_pitchpitch);
            msg.pose.covariance[29] = deg2radSq(last_attcoveuler_.cov_headpitch);
            msg.pose.covariance[33] = deg2radSq(last_attcoveuler_.cov_headroll);
            msg.pose.covariance[34] = deg2radSq(last_attcoveuler_.cov_headpitch);
            msg.pose.covariance[35] = deg2radSq(last_attcoveuler_.cov_headhead);
        }
        publish<PoseWithCovarianceStampedMsg>("pose", msg);
    };

    void MessageHandler::assembleDiagnosticArray(
        const std::shared_ptr<Telegram>& telegram)
    {
        if (!settings_->publish_diagnostics)
            return;

        DiagnosticArrayMsg msg;
        if (!validValue(last_receiverstatus_.block_header.tow) ||
            (last_receiverstatus_.block_header.tow !=
             last_qualityind_.block_header.tow))
            return;
        std::string serialnumber;
        if (validValue(last_receiversetup_.block_header.tow))
            serialnumber = last_receiversetup_.rx_serial_number;
        else
            serialnumber = "unknown";
        DiagnosticStatusMsg gnss_status;
        // Constructing the "level of operation" field
        uint16_t indicators_type_mask = static_cast<uint16_t>(255);
        uint16_t indicators_value_mask = static_cast<uint16_t>(3840);
        uint16_t qualityind_pos;
        for (uint16_t i = static_cast<uint16_t>(0);
             i < last_qualityind_.indicators.size(); ++i)
        {
            if ((last_qualityind_.indicators[i] & indicators_type_mask) ==
                static_cast<uint16_t>(0))
            {
                qualityind_pos = i;
                if (((last_qualityind_.indicators[i] & indicators_value_mask) >>
                     8) == static_cast<uint16_t>(0))
                {
                    gnss_status.level = DiagnosticStatusMsg::STALE;
                } else if (((last_qualityind_.indicators[i] &
                             indicators_value_mask) >>
                            8) == static_cast<uint16_t>(1) ||
                           ((last_qualityind_.indicators[i] &
                             indicators_value_mask) >>
                            8) == static_cast<uint16_t>(2))
                {
                    gnss_status.level = DiagnosticStatusMsg::WARN;
                } else
                {
                    gnss_status.level = DiagnosticStatusMsg::OK;
                }
                break;
            }
        }
        // If the ReceiverStatus's RxError field is not 0, then at least one error
        // has been detected.
        if (last_receiverstatus_.rx_error != static_cast<uint32_t>(0))
        {
            gnss_status.level = DiagnosticStatusMsg::ERROR;
        }
        // Creating an array of values associated with the GNSS status
        gnss_status.values.resize(static_cast<uint16_t>(last_qualityind_.n - 1));
        for (uint16_t i = static_cast<uint16_t>(0);
             i != static_cast<uint16_t>(last_qualityind_.n); ++i)
        {
            if (i == qualityind_pos)
            {
                continue;
            }
            if ((last_qualityind_.indicators[i] & indicators_type_mask) ==
                static_cast<uint16_t>(1))
            {
                gnss_status.values[i].key = "GNSS Signals, Main Antenna";
                gnss_status.values[i].value = std::to_string(
                    (last_qualityind_.indicators[i] & indicators_value_mask) >> 8);
            } else if ((last_qualityind_.indicators[i] & indicators_type_mask) ==
                       static_cast<uint16_t>(2))
            {
                gnss_status.values[i].key = "GNSS Signals, Aux1 Antenna";
                gnss_status.values[i].value = std::to_string(
                    (last_qualityind_.indicators[i] & indicators_value_mask) >> 8);
            } else if ((last_qualityind_.indicators[i] & indicators_type_mask) ==
                       static_cast<uint16_t>(11))
            {
                gnss_status.values[i].key = "RF Power, Main Antenna";
                gnss_status.values[i].value = std::to_string(
                    (last_qualityind_.indicators[i] & indicators_value_mask) >> 8);
            } else if ((last_qualityind_.indicators[i] & indicators_type_mask) ==
                       static_cast<uint16_t>(12))
            {
                gnss_status.values[i].key = "RF Power, Aux1 Antenna";
                gnss_status.values[i].value = std::to_string(
                    (last_qualityind_.indicators[i] & indicators_value_mask) >> 8);
            } else if ((last_qualityind_.indicators[i] & indicators_type_mask) ==
                       static_cast<uint16_t>(21))
            {
                gnss_status.values[i].key = "CPU Headroom";
                gnss_status.values[i].value = std::to_string(
                    (last_qualityind_.indicators[i] & indicators_value_mask) >> 8);
            } else if ((last_qualityind_.indicators[i] & indicators_type_mask) ==
                       static_cast<uint16_t>(25))
            {
                gnss_status.values[i].key = "OCXO Stability";
                gnss_status.values[i].value = std::to_string(
                    (last_qualityind_.indicators[i] & indicators_value_mask) >> 8);
            } else if ((last_qualityind_.indicators[i] & indicators_type_mask) ==
                       static_cast<uint16_t>(30))
            {
                gnss_status.values[i].key = "Base Station Measurements";
                gnss_status.values[i].value = std::to_string(
                    (last_qualityind_.indicators[i] & indicators_value_mask) >> 8);
            } else
            {
                assert((last_qualityind_.indicators[i] & indicators_type_mask) ==
                       static_cast<uint16_t>(31));
                gnss_status.values[i].key = "RTK Post-Processing";
                gnss_status.values[i].value = std::to_string(
                    (last_qualityind_.indicators[i] & indicators_value_mask) >> 8);
            }
        }
        gnss_status.hardware_id = serialnumber;
        gnss_status.name = "septentrio_driver: Quality indicators";
        gnss_status.message =
            "GNSS quality Indicators (from 0 for low quality to 10 for high quality, 15 if unknown)";
        msg.status.push_back(gnss_status);
        std::string frame_id;
        if (settings_->septentrio_receiver_type == "gnss")
        {
            frame_id = settings_->frame_id;
        }
        if (settings_->septentrio_receiver_type == "ins")
        {
            if (settings_->ins_use_poi)
            {
                frame_id = settings_->poi_frame_id;
            } else
            {
                frame_id = settings_->frame_id;
            }
        }
        assembleHeader(frame_id, telegram, msg);
        publish<DiagnosticArrayMsg>("/diagnostics", msg);
    };

    void MessageHandler::assembleOsnmaDiagnosticArray()
    {
        DiagnosticArrayMsg msg;
        DiagnosticStatusMsg diagOsnma;

        diagOsnma.hardware_id = last_receiversetup_.rx_serial_number;
        diagOsnma.name = "septentrio_driver: OSNMA";
        diagOsnma.message = "Current status of the OSNMA authentication";

        diagOsnma.values.resize(6);
        diagOsnma.values[0].key = "status";
        switch (last_gal_auth_status_.osnma_status & 7)
        {
        case 0:
        {
            diagOsnma.values[0].value = "Disabled";
            break;
        }
        case 1:
        {
            uint16_t percent = (last_gal_auth_status_.osnma_status >> 3) & 127;
            diagOsnma.values[0].value =
                "Initializing " + std::to_string(percent) + " %";
            break;
        }
        case 2:
        {
            diagOsnma.values[0].value = "Waiting on NTP";
            break;
        }
        case 3:
        {
            diagOsnma.values[0].value = "Init failed - inconsistent time";
            break;
        }
        case 4:
        {
            diagOsnma.values[0].value = "Init failed - KROOT signature invalid";
            break;
        }
        case 5:
        {
            diagOsnma.values[0].value = "Init failed - invalid param received";
            break;
        }
        case 6:
        {
            diagOsnma.values[0].value = "Authenticating";
            break;
        }

        default:
            break;
        }

        diagOsnma.values[1].key = "trusted_time_delta";
        if (validValue(last_gal_auth_status_.trusted_time_delta))
            diagOsnma.values[1].value =
                std::to_string(last_gal_auth_status_.trusted_time_delta);
        else
            diagOsnma.values[1].value = "N/A";

        std::bitset<64> gal_active = last_gal_auth_status_.gal_active_mask;
        std::bitset<64> gal_auth = last_gal_auth_status_.gal_authentic_mask;
        uint8_t gal_authentic = (gal_auth & gal_active).count();
        uint8_t gal_spoofed = (~gal_auth & gal_active).count();
        diagOsnma.values[2].key = "Galileo authentic";
        diagOsnma.values[2].value = std::to_string(gal_authentic);
        diagOsnma.values[3].key = "Galileo spoofed";
        diagOsnma.values[3].value = std::to_string(gal_spoofed);

        std::bitset<64> gps_active = last_gal_auth_status_.gps_active_mask;
        std::bitset<64> gps_auth = last_gal_auth_status_.gps_authentic_mask;
        uint8_t gps_authentic = (gps_auth & gps_active).count();
        uint8_t gps_spoofed = (~gps_auth & gps_active).count();
        diagOsnma.values[4].key = "GPS authentic";
        diagOsnma.values[4].value = std::to_string(gps_authentic);
        diagOsnma.values[5].key = "GPS spoofed";
        diagOsnma.values[5].value = std::to_string(gps_spoofed);

        if ((gal_spoofed + gps_spoofed) == 0)
            diagOsnma.level = DiagnosticStatusMsg::OK;
        else if ((gal_authentic + gps_authentic) > 0)
            diagOsnma.level = DiagnosticStatusMsg::WARN;
        else
            diagOsnma.level = DiagnosticStatusMsg::ERROR;

        msg.status.push_back(diagOsnma);
        msg.header = last_gal_auth_status_.header;

        publish<DiagnosticArrayMsg>("/diagnostics", msg);
    }

    void MessageHandler::assembleAimAndDiagnosticArray()
    {
        AimPlusStatusMsg aimMsg;
        DiagnosticArrayMsg msg;
        DiagnosticStatusMsg diagRf;
        diagRf.hardware_id = last_receiversetup_.rx_serial_number;
        diagRf.name = "septentrio_driver: AIM+ status";
        diagRf.message =
            "Current status of the AIM+ interference and spoofing mitigation";

        diagRf.values.resize(2);
        diagRf.values[0].key = "interference";
        bool mitigated = false;
        bool detected = false;
        for (auto rfband : last_rf_status_.rfband)
        {
            std::bitset<8> info = rfband.info;
            if (info.test(1))
                mitigated = true;
            if (info.test(3))
            {
                detected = true;
                break;
            }
        }
        if (detected)
        {
            diagRf.values[0].value = "present";
            aimMsg.interference = AimPlusStatusMsg::INTERFERENCE_PRESENT;
        } else if (mitigated)
        {
            diagRf.values[0].value = "mitigated";
            aimMsg.interference = AimPlusStatusMsg::INTERFERENCE_MITIGATED;
        } else
        {
            diagRf.values[0].value = "spectrum clean";
            aimMsg.interference = AimPlusStatusMsg::SPECTRUM_CLEAN;
        }

        diagRf.values[1].key = "spoofing";
        bool spoofed = false;
        std::bitset<8> flags = last_rf_status_.flags;
        if (flags.test(0) && flags.test(1))
        {
            diagRf.values[1].value = "detected by OSNMA and authenticity test";
            aimMsg.spoofing =
                AimPlusStatusMsg::SPOOFING_DETECTED_BY_OSNMA_AND_AUTHENTCITY_TEST;
            spoofed = true;
        } else if (flags.test(0))
        {
            diagRf.values[1].value = "detected by authenticity test";
            aimMsg.spoofing =
                AimPlusStatusMsg::SPOOFING_DETECTED_BY_AUTHENTCITY_TEST;
            spoofed = true;
        } else if (flags.test(1))
        {
            diagRf.values[1].value = "detected by OSNMA";
            aimMsg.spoofing = AimPlusStatusMsg::SPOOFING_DETECTED_BY_OSNMA;
            spoofed = true;
        } else
        {
            diagRf.values[1].value = "none detected";
            aimMsg.spoofing = AimPlusStatusMsg::NONE_DETECTED;
        }
        if (osnma_info_available_)
        {
            aimMsg.osnma_authenticating =
                ((last_gal_auth_status_.osnma_status & 7) == 6);
            std::bitset<64> gal_active = last_gal_auth_status_.gal_active_mask;
            std::bitset<64> gal_auth = last_gal_auth_status_.gal_authentic_mask;
            aimMsg.galileo_authentic = (gal_auth & gal_active).count();
            aimMsg.galileo_spoofed = (~gal_auth & gal_active).count();
            std::bitset<64> gps_active = last_gal_auth_status_.gps_active_mask;
            std::bitset<64> gps_auth = last_gal_auth_status_.gps_authentic_mask;
            aimMsg.gps_authentic = (gps_auth & gps_active).count();
            aimMsg.gps_spoofed = (~gps_auth & gps_active).count();
        } else
        {
            aimMsg.osnma_authenticating = false;
            aimMsg.galileo_authentic = 0;
            aimMsg.galileo_spoofed = 0;
            aimMsg.gps_authentic = 0;
            aimMsg.gps_spoofed = 0;
        }
        aimMsg.header = last_rf_status_.header;
        aimMsg.tow = last_rf_status_.block_header.tow;
        aimMsg.wnc = last_rf_status_.block_header.wnc;
        publish<AimPlusStatusMsg>("aimplusstatus", aimMsg);

        if (spoofed || detected)
            diagRf.level = DiagnosticStatusMsg::ERROR;
        else if (mitigated)
            diagRf.level = DiagnosticStatusMsg::WARN;
        else
            diagRf.level = DiagnosticStatusMsg::OK;

        msg.status.push_back(diagRf);
        msg.header = last_rf_status_.header;

        publish<DiagnosticArrayMsg>("/diagnostics", msg);
    }

    void MessageHandler::assembleImu()
    {
        ImuMsg msg;

        msg.header = last_extsensmeas_.header;

        msg.linear_acceleration.x = last_extsensmeas_.acceleration_x;
        msg.linear_acceleration.y = last_extsensmeas_.acceleration_y;
        msg.linear_acceleration.z = last_extsensmeas_.acceleration_z;

        msg.angular_velocity.x = deg2rad(last_extsensmeas_.angular_rate_x);
        msg.angular_velocity.y = deg2rad(last_extsensmeas_.angular_rate_y);
        msg.angular_velocity.z = deg2rad(last_extsensmeas_.angular_rate_z);

        bool valid_orientation = false;
        if (settings_->septentrio_receiver_type == "ins")
        {
            if (validValue(last_insnavgeod_.block_header.tow))
            {
                // INS tow and extsens meas tow have the same time scale
                Timestamp tsImu = timestampSBF(last_extsensmeas_.block_header.tow,
                                               last_extsensmeas_.block_header.wnc);
                Timestamp tsIns = timestampSBF(last_insnavgeod_.block_header.tow,
                                               last_insnavgeod_.block_header.wnc);

                static int64_t maxDt = (settings_->polling_period_pvt == 0)
                                           ? 10000000
                                           : settings_->polling_period_pvt * 1000000;
                if ((tsImu - tsIns) > maxDt)
                {
                    valid_orientation = false;
                } else
                {
                    if ((last_insnavgeod_.sb_list & 2) != 0)
                    {
                        // Attitude
                        if (validValue(last_insnavgeod_.heading) &&
                            validValue(last_insnavgeod_.pitch) &&
                            validValue(last_insnavgeod_.roll))
                        {
                            msg.orientation = convertEulerToQuaternionMsg(
                                deg2rad(last_insnavgeod_.roll),
                                deg2rad(last_insnavgeod_.pitch),
                                deg2rad(last_insnavgeod_.heading));
                        } else
                        {
                            valid_orientation = false;
                        }
                    } else
                    {
                        valid_orientation = false;
                    }
                    if ((last_insnavgeod_.sb_list & 4) != 0)
                    {
                        // Attitude autocov
                        if (validValue(last_insnavgeod_.roll_std_dev) &&
                            validValue(last_insnavgeod_.pitch_std_dev) &&
                            validValue(last_insnavgeod_.heading_std_dev))
                        {
                            msg.orientation_covariance[0] =
                                square(deg2rad(last_insnavgeod_.roll_std_dev));
                            msg.orientation_covariance[4] =
                                square(deg2rad(last_insnavgeod_.pitch_std_dev));
                            msg.orientation_covariance[8] =
                                square(deg2rad(last_insnavgeod_.heading_std_dev));
                        } else
                        {
                            valid_orientation = false;
                        }
                    } else
                    {
                        valid_orientation = false;
                    }
                    if ((last_insnavgeod_.sb_list & 64) != 0)
                    {
                        // Attitude cov
                        msg.orientation_covariance[1] =
                            deg2radSq(last_insnavgeod_.pitch_roll_cov);
                        msg.orientation_covariance[2] =
                            deg2radSq(last_insnavgeod_.heading_roll_cov);
                        msg.orientation_covariance[3] =
                            deg2radSq(last_insnavgeod_.pitch_roll_cov);

                        msg.orientation_covariance[5] =
                            deg2radSq(last_insnavgeod_.heading_pitch_cov);
                        msg.orientation_covariance[6] =
                            deg2radSq(last_insnavgeod_.heading_roll_cov);
                        msg.orientation_covariance[7] =
                            deg2radSq(last_insnavgeod_.heading_pitch_cov);
                    }
                }
            } else
            {
                valid_orientation = false;
            }
        }

        if (!valid_orientation)
        {
            msg.orientation.w = std::numeric_limits<double>::quiet_NaN();
            msg.orientation.x = std::numeric_limits<double>::quiet_NaN();
            msg.orientation.y = std::numeric_limits<double>::quiet_NaN();
            msg.orientation.z = std::numeric_limits<double>::quiet_NaN();
            msg.orientation_covariance[0] = -1.0;
            msg.orientation_covariance[4] = -1.0;
            msg.orientation_covariance[8] = -1.0;
        }

        publish<ImuMsg>("imu", msg);
    };

    void MessageHandler::assembleTwist(bool fromIns /* = false*/)
    {
        if (!settings_->publish_twist)
            return;
        TwistWithCovarianceStampedMsg msg;

        if (fromIns)
        {
            msg.header = last_insnavgeod_.header;

            if ((last_insnavgeod_.sb_list & 8) != 0)
            {
                // Linear velocity in navigation frame
                double ve = 0.0;
                if (validValue(last_insnavgeod_.ve))
                    ve = last_insnavgeod_.ve;
                double vn = 0.0;
                if (validValue(last_insnavgeod_.vn))
                    vn = last_insnavgeod_.vn;
                double vu = 0.0;
                if (validValue(last_insnavgeod_.vu))
                    vu = last_insnavgeod_.vu;
                Eigen::Vector3d vel;
                if (settings_->use_ros_axis_orientation)
                {
                    // (ENU)
                    vel << ve, vn, vu;
                } else
                {
                    // (NED)
                    vel << vn, ve, -vu;
                }
                // Linear velocity
                msg.twist.twist.linear.x = vel(0);
                msg.twist.twist.linear.y = vel(1);
                msg.twist.twist.linear.z = vel(2);
            } else
            {
                msg.twist.twist.linear.x = std::numeric_limits<double>::quiet_NaN();
                msg.twist.twist.linear.y = std::numeric_limits<double>::quiet_NaN();
                msg.twist.twist.linear.z = std::numeric_limits<double>::quiet_NaN();
            }

            if (((last_insnavgeod_.sb_list & 16) != 0) &&
                ((last_insnavgeod_.sb_list & 2) != 0) &&
                ((last_insnavgeod_.sb_list & 8) != 0))
            {
                Eigen::Matrix3d covVel_local = Eigen::Matrix3d::Zero();
                if ((last_insnavgeod_.sb_list & 128) != 0)
                {
                    // Linear velocity covariance
                    if (validValue(last_insnavgeod_.ve_std_dev))
                        if (settings_->use_ros_axis_orientation)
                            covVel_local(0, 0) = square(last_insnavgeod_.ve_std_dev);
                        else
                            covVel_local(1, 1) = square(last_insnavgeod_.ve_std_dev);
                    else
                        covVel_local(0, 0) = -1.0;
                    if (validValue(last_insnavgeod_.vn_std_dev))
                        if (settings_->use_ros_axis_orientation)
                            covVel_local(1, 1) = square(last_insnavgeod_.vn_std_dev);
                        else
                            covVel_local(0, 0) = square(last_insnavgeod_.vn_std_dev);
                    else
                        covVel_local(1, 1) = -1.0;
                    if (validValue(last_insnavgeod_.vu_std_dev))
                        covVel_local(2, 2) = square(last_insnavgeod_.vu_std_dev);
                    else
                        covVel_local(2, 2) = -1.0;

                    if (validValue(last_insnavgeod_.ve_vn_cov))
                        covVel_local(0, 1) = covVel_local(1, 0) =
                            last_insnavgeod_.ve_vn_cov;
                    if (settings_->use_ros_axis_orientation)
                    {
                        if (validValue(last_insnavgeod_.ve_vu_cov))
                            covVel_local(0, 2) = covVel_local(2, 0) =
                                last_insnavgeod_.ve_vu_cov;
                        if (validValue(last_insnavgeod_.vn_vu_cov))
                            covVel_local(2, 1) = covVel_local(1, 2) =
                                last_insnavgeod_.vn_vu_cov;
                    } else
                    {
                        if (validValue(last_insnavgeod_.vn_vu_cov))
                            covVel_local(0, 2) = covVel_local(2, 0) =
                                -last_insnavgeod_.vn_vu_cov;
                        if (validValue(last_insnavgeod_.ve_vu_cov))
                            covVel_local(2, 1) = covVel_local(1, 2) =
                                -last_insnavgeod_.ve_vu_cov;
                    }
                } else
                {
                    covVel_local(0, 0) = -1.0;
                    covVel_local(1, 1) = -1.0;
                    covVel_local(2, 2) = -1.0;
                }

                msg.twist.covariance[0] = covVel_local(0, 0);
                msg.twist.covariance[1] = covVel_local(0, 1);
                msg.twist.covariance[2] = covVel_local(0, 2);
                msg.twist.covariance[6] = covVel_local(1, 0);
                msg.twist.covariance[7] = covVel_local(1, 1);
                msg.twist.covariance[8] = covVel_local(1, 2);
                msg.twist.covariance[12] = covVel_local(2, 0);
                msg.twist.covariance[13] = covVel_local(2, 1);
                msg.twist.covariance[14] = covVel_local(2, 2);
            } else
            {
                msg.twist.covariance[0] = -1.0;
                msg.twist.covariance[7] = -1.0;
                msg.twist.covariance[14] = -1.0;
            }
            // Autocovariances of angular velocity
            msg.twist.covariance[21] = -1.0;
            msg.twist.covariance[28] = -1.0;
            msg.twist.covariance[35] = -1.0;

            publish<TwistWithCovarianceStampedMsg>("twist_ins", msg);
        } else
        {
            if ((!validValue(last_pvtgeodetic_.block_header.tow)) ||
                (last_pvtgeodetic_.block_header.tow !=
                 last_velcovgeodetic_.block_header.tow))
                return;
            msg.header = last_pvtgeodetic_.header;

            if (last_pvtgeodetic_.error == 0)
            {
                // Linear velocity in navigation frame
                double ve = 0.0;
                if (validValue(last_pvtgeodetic_.ve))
                    ve = last_pvtgeodetic_.ve;
                double vn = 0.0;
                if (validValue(last_pvtgeodetic_.vn))
                    vn = last_pvtgeodetic_.vn;
                double vu = 0.0;
                if (validValue(last_pvtgeodetic_.vu))
                    vu = last_pvtgeodetic_.vu;
                Eigen::Vector3d vel;
                if (settings_->use_ros_axis_orientation)
                {
                    // (ENU)
                    vel << ve, vn, vu;
                } else
                {
                    // (NED)
                    vel << vn, ve, -vu;
                }
                // Linear velocity
                msg.twist.twist.linear.x = vel(0);
                msg.twist.twist.linear.y = vel(1);
                msg.twist.twist.linear.z = vel(2);
            } else
            {
                msg.twist.twist.linear.x = std::numeric_limits<double>::quiet_NaN();
                msg.twist.twist.linear.y = std::numeric_limits<double>::quiet_NaN();
                msg.twist.twist.linear.z = std::numeric_limits<double>::quiet_NaN();
            }

            if (last_velcovgeodetic_.error == 0)
            {
                Eigen::Matrix3d covVel_local = Eigen::Matrix3d::Zero();
                // Linear velocity covariance in navigation frame
                if (validValue(last_velcovgeodetic_.cov_veve))
                    if (settings_->use_ros_axis_orientation)
                        covVel_local(0, 0) = last_velcovgeodetic_.cov_veve;
                    else
                        covVel_local(1, 1) = last_velcovgeodetic_.cov_veve;
                else
                    covVel_local(0, 0) = -1.0;
                if (validValue(last_velcovgeodetic_.cov_vnvn))
                    if (settings_->use_ros_axis_orientation)
                        covVel_local(1, 1) = last_velcovgeodetic_.cov_vnvn;
                    else
                        covVel_local(0, 0) = last_velcovgeodetic_.cov_vnvn;
                else
                    covVel_local(1, 1) = -1.0;
                if (validValue(last_velcovgeodetic_.cov_vuvu))
                    covVel_local(2, 2) = last_velcovgeodetic_.cov_vuvu;
                else
                    covVel_local(2, 2) = -1.0;

                covVel_local(0, 1) = covVel_local(1, 0) =
                    last_velcovgeodetic_.cov_vnve;
                if (settings_->use_ros_axis_orientation)
                {
                    covVel_local(0, 2) = covVel_local(2, 0) =
                        last_velcovgeodetic_.cov_vevu;
                    covVel_local(2, 1) = covVel_local(1, 2) =
                        last_velcovgeodetic_.cov_vnvu;
                } else
                {
                    covVel_local(0, 2) = covVel_local(2, 0) =
                        -last_velcovgeodetic_.cov_vnvu;
                    covVel_local(2, 1) = covVel_local(1, 2) =
                        -last_velcovgeodetic_.cov_vevu;
                }

                msg.twist.covariance[0] = covVel_local(0, 0);
                msg.twist.covariance[1] = covVel_local(0, 1);
                msg.twist.covariance[2] = covVel_local(0, 2);
                msg.twist.covariance[6] = covVel_local(1, 0);
                msg.twist.covariance[7] = covVel_local(1, 1);
                msg.twist.covariance[8] = covVel_local(1, 2);
                msg.twist.covariance[12] = covVel_local(2, 0);
                msg.twist.covariance[13] = covVel_local(2, 1);
                msg.twist.covariance[14] = covVel_local(2, 2);
            } else
            {
                msg.twist.covariance[0] = -1.0;
                msg.twist.covariance[7] = -1.0;
                msg.twist.covariance[14] = -1.0;
            }
            // Autocovariances of angular velocity
            msg.twist.covariance[21] = -1.0;
            msg.twist.covariance[28] = -1.0;
            msg.twist.covariance[35] = -1.0;

            publish<TwistWithCovarianceStampedMsg>("twist_gnss", msg);
        }
    };

    /**
     * Localization in UTM coordinates. Yaw angle is converted from true north to
     * grid north. Linear velocity of twist in body frame as per msg definition.
     * Angular velocity not available, thus according autocovariances are set to
     * -1.0.
     */
    void MessageHandler::assembleLocalizationUtm()
    {
        if (!settings_->publish_localization && !settings_->publish_tf)
            return;

        LocalizationMsg msg;

        int zone;
        std::string zonestring;
        bool northernHemisphere;
        double easting;
        double northing;
        double meridian_convergence = 0.0;
        if (fixedUtmZone_)
        {
            try
            {
                GeographicLib::UTMUPS::DecodeZone(*fixedUtmZone_, zone,
                                                  northernHemisphere);
                double k;
                GeographicLib::UTMUPS::Forward(
                    rad2deg(last_insnavgeod_.latitude),
                    rad2deg(last_insnavgeod_.longitude), zone, northernHemisphere,
                    easting, northing, meridian_convergence, k, zone);
            } catch (const std::exception& e)
            {
                node_->log(log_level::DEBUG,
                           "UTMUPS conversion exception: " + std::string(e.what()));
                return;
            }
            zonestring = *fixedUtmZone_;
        } else
        {
            try
            {
                double k;
                GeographicLib::UTMUPS::Forward(rad2deg(last_insnavgeod_.latitude),
                                               rad2deg(last_insnavgeod_.longitude),
                                               zone, northernHemisphere, easting,
                                               northing, meridian_convergence, k);
                zonestring =
                    GeographicLib::UTMUPS::EncodeZone(zone, northernHemisphere);
            } catch (const std::exception& e)
            {
                node_->log(log_level::DEBUG,
                           "UTMUPS conversion exception: " + std::string(e.what()));
                return;
            }
        }
        if (settings_->lock_utm_zone && !fixedUtmZone_)
            fixedUtmZone_ = std::make_shared<std::string>(zonestring);

        // UTM position (ENU)
        if (settings_->use_ros_axis_orientation)
        {
            msg.pose.pose.position.x = easting;
            msg.pose.pose.position.y = northing;
            msg.pose.pose.position.z = last_insnavgeod_.height;
        } else // (NED)
        {
            msg.pose.pose.position.x = northing;
            msg.pose.pose.position.y = easting;
            msg.pose.pose.position.z = -last_insnavgeod_.height;
        }

        msg.header.frame_id = "utm_" + zonestring;
        msg.header.stamp = last_insnavgeod_.header.stamp;
        if (settings_->ins_use_poi)
            msg.child_frame_id = settings_->poi_frame_id;
        else
            msg.child_frame_id = settings_->frame_id;

        Eigen::Matrix3d P_pos = -Eigen::Matrix3d::Identity();
        if ((last_insnavgeod_.sb_list & 1) != 0)
        {
            // Position autocovariance
            P_pos(0, 0) = square(last_insnavgeod_.longitude_std_dev);
            P_pos(1, 1) = square(last_insnavgeod_.latitude_std_dev);
            P_pos(2, 2) = square(last_insnavgeod_.height_std_dev);
        }

        // Euler angles
        double roll = 0.0;
        if (validValue(last_insnavgeod_.roll))
            roll = deg2rad(last_insnavgeod_.roll);
        double pitch = 0.0;
        if (validValue(last_insnavgeod_.pitch))
            pitch = deg2rad(last_insnavgeod_.pitch);
        double yaw = 0.0;
        if (validValue(last_insnavgeod_.heading))
            yaw = deg2rad(last_insnavgeod_.heading);
        // meridian_convergence for conversion from true north to grid north
        if (settings_->use_ros_axis_orientation)
            yaw += deg2rad(meridian_convergence);
        else
            yaw -= deg2rad(meridian_convergence);

        if ((last_insnavgeod_.sb_list & 2) != 0)
        {
            // Attitude
            msg.pose.pose.orientation =
                convertEulerToQuaternionMsg(roll, pitch, yaw);
        } else
        {
            msg.pose.pose.orientation.w = std::numeric_limits<double>::quiet_NaN();
            msg.pose.pose.orientation.x = std::numeric_limits<double>::quiet_NaN();
            msg.pose.pose.orientation.y = std::numeric_limits<double>::quiet_NaN();
            msg.pose.pose.orientation.z = std::numeric_limits<double>::quiet_NaN();
        }
        if ((last_insnavgeod_.sb_list & 4) != 0)
        {
            // Attitude autocovariance
            if (validValue(last_insnavgeod_.roll_std_dev))
                msg.pose.covariance[21] =
                    square(deg2rad(last_insnavgeod_.roll_std_dev));
            else
                msg.pose.covariance[21] = -1.0;
            if (validValue(last_insnavgeod_.pitch_std_dev))
                msg.pose.covariance[28] =
                    square(deg2rad(last_insnavgeod_.pitch_std_dev));
            else
                msg.pose.covariance[28] = -1.0;
            if (validValue(last_insnavgeod_.heading_std_dev))
                msg.pose.covariance[35] =
                    square(deg2rad(last_insnavgeod_.heading_std_dev));
            else
                msg.pose.covariance[35] = -1.0;
        } else
        {
            msg.pose.covariance[21] = -1.0;
            msg.pose.covariance[28] = -1.0;
            msg.pose.covariance[35] = -1.0;
        }

        if ((last_insnavgeod_.sb_list & 32) != 0)
        {
            // Position covariance
            P_pos(0, 1) = last_insnavgeod_.latitude_longitude_cov;
            P_pos(1, 0) = last_insnavgeod_.latitude_longitude_cov;

            if (settings_->use_ros_axis_orientation)
            {
                // (ENU)
                P_pos(0, 2) = last_insnavgeod_.longitude_height_cov;
                P_pos(1, 2) = last_insnavgeod_.latitude_height_cov;
                P_pos(2, 0) = last_insnavgeod_.longitude_height_cov;
                P_pos(2, 1) = last_insnavgeod_.latitude_height_cov;
            } else
            {
                // (NED): down = -height
                P_pos(0, 2) = -last_insnavgeod_.latitude_height_cov;
                P_pos(1, 2) = -last_insnavgeod_.longitude_height_cov;
                P_pos(2, 0) = -last_insnavgeod_.latitude_height_cov;
                P_pos(2, 1) = -last_insnavgeod_.longitude_height_cov;
            }
        }

        if ((meridian_convergence != 0.0) && (last_insnavgeod_.sb_list & 1))
        {
            double cg = std::cos(meridian_convergence);
            double sg = std::sin(meridian_convergence);
            Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
            R(0, 0) = cg;
            R(0, 1) = -sg;
            R(1, 0) = sg;
            R(1, 1) = cg;
            P_pos = (R * P_pos * R.transpose()).eval();
        }

        msg.pose.covariance[0] = P_pos(0, 0);
        msg.pose.covariance[1] = P_pos(0, 1);
        msg.pose.covariance[2] = P_pos(0, 2);
        msg.pose.covariance[6] = P_pos(1, 0);
        msg.pose.covariance[7] = P_pos(1, 1);
        msg.pose.covariance[8] = P_pos(1, 2);
        msg.pose.covariance[12] = P_pos(2, 0);
        msg.pose.covariance[13] = P_pos(2, 1);
        msg.pose.covariance[14] = P_pos(2, 2);

        if ((last_insnavgeod_.sb_list & 64) != 0)
        {
            // Attitude covariancae
            msg.pose.covariance[22] = deg2radSq(last_insnavgeod_.pitch_roll_cov);
            msg.pose.covariance[23] = deg2radSq(last_insnavgeod_.heading_roll_cov);
            msg.pose.covariance[27] = deg2radSq(last_insnavgeod_.pitch_roll_cov);

            msg.pose.covariance[29] = deg2radSq(last_insnavgeod_.heading_pitch_cov);
            msg.pose.covariance[33] = deg2radSq(last_insnavgeod_.heading_roll_cov);
            msg.pose.covariance[34] = deg2radSq(last_insnavgeod_.heading_pitch_cov);
        }

        assembleLocalizationMsgTwist(roll, pitch, yaw, msg);

        if (settings_->publish_localization)
            publish<LocalizationMsg>("localization", msg);
        if (settings_->publish_tf)
            publishTf(msg);
    };

    /**
     * Localization in ECEF coordinates. Attitude is converted from local to ECEF
     * Linear velocity of twist in body frame as per msg definition. Angular
     * velocity not available, thus according autocovariances are set to -1.0.
     */
    void MessageHandler::assembleLocalizationEcef()
    {
        if (!settings_->publish_localization_ecef && !settings_->publish_tf_ecef)
            return;

        if ((!validValue(last_insnavcart_.block_header.tow)) ||
            (last_insnavcart_.block_header.tow != last_insnavgeod_.block_header.tow))
            return;

        LocalizationMsg msg;

        msg.header.frame_id = "ecef";
        msg.header.stamp = last_insnavcart_.header.stamp;
        if (settings_->ins_use_poi)
            msg.child_frame_id = settings_->poi_frame_id;
        else
            msg.child_frame_id = settings_->frame_id;

        // ECEF position
        msg.pose.pose.position.x = last_insnavcart_.x;
        msg.pose.pose.position.y = last_insnavcart_.y;
        msg.pose.pose.position.z = last_insnavcart_.z;

        if ((last_insnavgeod_.sb_list & 1) != 0)
        {
            // Position autocovariance
            msg.pose.covariance[0] = square(last_insnavcart_.x_std_dev);
            msg.pose.covariance[7] = square(last_insnavcart_.y_std_dev);
            msg.pose.covariance[14] = square(last_insnavcart_.z_std_dev);
        } else
        {
            msg.pose.covariance[0] = -1.0;
            msg.pose.covariance[7] = -1.0;
            msg.pose.covariance[14] = -1.0;
        }
        if ((last_insnavcart_.sb_list & 32) != 0)
        {
            // Position covariance
            msg.pose.covariance[1] = last_insnavcart_.xy_cov;
            msg.pose.covariance[6] = last_insnavcart_.xy_cov;
            msg.pose.covariance[2] = last_insnavcart_.xz_cov;
            msg.pose.covariance[8] = last_insnavcart_.yz_cov;
            msg.pose.covariance[12] = last_insnavcart_.xz_cov;
            msg.pose.covariance[13] = last_insnavcart_.yz_cov;
        }

        // Euler angles
        double roll = 0.0;
        if (validValue(last_insnavcart_.roll))
            roll = deg2rad(last_insnavcart_.roll);
        double pitch = 0.0;
        if (validValue(last_insnavcart_.pitch))
            pitch = deg2rad(last_insnavcart_.pitch);
        double yaw = 0.0;
        if (validValue(last_insnavcart_.heading))
            yaw = deg2rad(last_insnavcart_.heading);

        if ((last_insnavcart_.sb_list & 2) != 0)
        {
            // Attitude
            Eigen::Quaterniond q_local_ecef;
            if (settings_->use_ros_axis_orientation)
                q_local_ecef = parsing_utilities::q_enu_ecef(
                    last_insnavgeod_.latitude, last_insnavgeod_.longitude);
            else
                q_local_ecef = parsing_utilities::q_ned_ecef(
                    last_insnavgeod_.latitude, last_insnavgeod_.longitude);
            Eigen::Quaterniond q_b_local =
                parsing_utilities::convertEulerToQuaternion(roll, pitch, yaw);

            Eigen::Quaterniond q_b_ecef = q_local_ecef * q_b_local;

            msg.pose.pose.orientation =
                parsing_utilities::quaternionToQuaternionMsg(q_b_ecef);
        } else
        {
            msg.pose.pose.orientation.w = std::numeric_limits<double>::quiet_NaN();
            msg.pose.pose.orientation.x = std::numeric_limits<double>::quiet_NaN();
            msg.pose.pose.orientation.y = std::numeric_limits<double>::quiet_NaN();
            msg.pose.pose.orientation.z = std::numeric_limits<double>::quiet_NaN();
        }
        Eigen::Matrix3d covAtt_local = Eigen::Matrix3d::Zero();
        bool covAttValid = true;
        if ((last_insnavgeod_.sb_list & 4) != 0)
        {
            // Attitude autocovariance
            if (validValue(last_insnavgeod_.roll_std_dev))
                covAtt_local(0, 0) = square(deg2rad(last_insnavgeod_.roll_std_dev));
            else
            {
                covAtt_local(0, 0) = -1.0;
                covAttValid = false;
            }
            if (validValue(last_insnavgeod_.pitch_std_dev))
                covAtt_local(1, 1) = square(deg2rad(last_insnavgeod_.pitch_std_dev));
            else
            {
                covAtt_local(1, 1) = -1.0;
                covAttValid = false;
            }
            if (validValue(last_insnavgeod_.heading_std_dev))
                covAtt_local(2, 2) =
                    square(deg2rad(last_insnavgeod_.heading_std_dev));
            else
            {
                covAtt_local(2, 2) = -1.0;
                covAttValid = false;
            }
        } else
        {
            covAtt_local(0, 0) = -1.0;
            covAtt_local(1, 1) = -1.0;
            covAtt_local(2, 2) = -1.0;
            covAttValid = false;
        }

        if (covAttValid)
        {
            if ((last_insnavcart_.sb_list & 64) != 0)
            {
                // Attitude covariancae
                covAtt_local(0, 1) = deg2radSq(last_insnavcart_.pitch_roll_cov);
                covAtt_local(0, 2) = deg2radSq(last_insnavcart_.heading_roll_cov);
                covAtt_local(1, 0) = deg2radSq(last_insnavcart_.pitch_roll_cov);
                covAtt_local(2, 1) = deg2radSq(last_insnavcart_.heading_pitch_cov);
                covAtt_local(2, 0) = deg2radSq(last_insnavcart_.heading_roll_cov);
                covAtt_local(1, 2) = deg2radSq(last_insnavcart_.heading_pitch_cov);
            }

            Eigen::Matrix3d R_local_ecef;
            if (settings_->use_ros_axis_orientation)
                R_local_ecef = parsing_utilities::R_enu_ecef(
                    last_insnavgeod_.latitude, last_insnavgeod_.longitude);
            else
                R_local_ecef = parsing_utilities::R_ned_ecef(
                    last_insnavgeod_.latitude, last_insnavgeod_.longitude);
            // Rotate attitude covariance matrix to ecef coordinates
            Eigen::Matrix3d covAtt_ecef =
                R_local_ecef * covAtt_local * R_local_ecef.transpose();

            msg.pose.covariance[21] = covAtt_ecef(0, 0);
            msg.pose.covariance[22] = covAtt_ecef(0, 1);
            msg.pose.covariance[23] = covAtt_ecef(0, 2);
            msg.pose.covariance[27] = covAtt_ecef(1, 0);
            msg.pose.covariance[28] = covAtt_ecef(1, 1);
            msg.pose.covariance[29] = covAtt_ecef(1, 2);
            msg.pose.covariance[33] = covAtt_ecef(2, 0);
            msg.pose.covariance[34] = covAtt_ecef(2, 1);
            msg.pose.covariance[35] = covAtt_ecef(2, 2);
        } else
        {
            msg.pose.covariance[21] = -1.0;
            msg.pose.covariance[28] = -1.0;
            msg.pose.covariance[35] = -1.0;
        }

        assembleLocalizationMsgTwist(roll, pitch, yaw, msg);

        if (settings_->publish_localization_ecef)
            publish<LocalizationMsg>("localization_ecef", msg);
        if (settings_->publish_tf_ecef)
            publishTf(msg);
    };

    void MessageHandler::assembleLocalizationMsgTwist(double roll, double pitch,
                                                      double yaw,
                                                      LocalizationMsg& msg) const
    {
        Eigen::Matrix3d R_local_body =
            parsing_utilities::rpyToRot(roll, pitch, yaw).inverse();
        if ((last_insnavgeod_.sb_list & 8) != 0)
        {
            // Linear velocity (ENU)
            double ve = 0.0;
            if (validValue(last_insnavgeod_.ve))
                ve = last_insnavgeod_.ve;
            double vn = 0.0;
            if (validValue(last_insnavgeod_.vn))
                vn = last_insnavgeod_.vn;
            double vu = 0.0;
            if (validValue(last_insnavgeod_.vu))
                vu = last_insnavgeod_.vu;
            Eigen::Vector3d vel_local;
            if (settings_->use_ros_axis_orientation)
            {
                // (ENU)
                vel_local << ve, vn, vu;
            } else
            {
                // (NED)
                vel_local << vn, ve, -vu;
            }
            // Linear velocity, rotate to body coordinates
            Eigen::Vector3d vel_body = R_local_body * vel_local;
            msg.twist.twist.linear.x = vel_body(0);
            msg.twist.twist.linear.y = vel_body(1);
            msg.twist.twist.linear.z = vel_body(2);
        } else
        {
            msg.twist.twist.linear.x = std::numeric_limits<double>::quiet_NaN();
            msg.twist.twist.linear.y = std::numeric_limits<double>::quiet_NaN();
            msg.twist.twist.linear.z = std::numeric_limits<double>::quiet_NaN();
        }
        Eigen::Matrix3d covVel_local = Eigen::Matrix3d::Zero();
        if ((last_insnavgeod_.sb_list & 16) != 0)
        {
            // Linear velocity autocovariance
            if (validValue(last_insnavgeod_.ve_std_dev))
                if (settings_->use_ros_axis_orientation)
                    covVel_local(0, 0) = square(last_insnavgeod_.ve_std_dev);
                else
                    covVel_local(0, 0) = square(last_insnavgeod_.vn_std_dev);
            else
                covVel_local(0, 0) = -1.0;
            if (validValue(last_insnavgeod_.vn_std_dev))
                if (settings_->use_ros_axis_orientation)
                    covVel_local(1, 1) = square(last_insnavgeod_.vn_std_dev);
                else
                    covVel_local(1, 1) = square(last_insnavgeod_.ve_std_dev);
            else
                covVel_local(1, 1) = -1.0;
            if (validValue(last_insnavgeod_.vu_std_dev))
                covVel_local(2, 2) = square(last_insnavgeod_.vu_std_dev);
            else
                covVel_local(2, 2) = -1.0;
        } else
        {
            covVel_local(0, 0) = -1.0;
            covVel_local(1, 1) = -1.0;
            covVel_local(2, 2) = -1.0;
        }

        if ((last_insnavgeod_.sb_list & 128) != 0)
        {
            covVel_local(0, 1) = covVel_local(1, 0) = last_insnavgeod_.ve_vn_cov;
            if (settings_->use_ros_axis_orientation)
            {
                covVel_local(0, 2) = covVel_local(2, 0) = last_insnavgeod_.ve_vu_cov;
                covVel_local(2, 1) = covVel_local(1, 2) = last_insnavgeod_.vn_vu_cov;
            } else
            {
                covVel_local(0, 2) = covVel_local(2, 0) =
                    -last_insnavgeod_.vn_vu_cov;
                covVel_local(2, 1) = covVel_local(1, 2) =
                    -last_insnavgeod_.ve_vu_cov;
            }
        }

        if (((last_insnavgeod_.sb_list & 16) != 0) &&
            ((last_insnavgeod_.sb_list & 2) != 0) &&
            ((last_insnavgeod_.sb_list & 8) != 0) &&
            validValue(last_insnavgeod_.ve_std_dev) &&
            validValue(last_insnavgeod_.vn_std_dev) &&
            validValue(last_insnavgeod_.vu_std_dev))
        {
            // Rotate velocity covariance matrix to body coordinates
            Eigen::Matrix3d covVel_body =
                R_local_body * covVel_local * R_local_body.transpose();

            msg.twist.covariance[0] = covVel_body(0, 0);
            msg.twist.covariance[1] = covVel_body(0, 1);
            msg.twist.covariance[2] = covVel_body(0, 2);
            msg.twist.covariance[6] = covVel_body(1, 0);
            msg.twist.covariance[7] = covVel_body(1, 1);
            msg.twist.covariance[8] = covVel_body(1, 2);
            msg.twist.covariance[12] = covVel_body(2, 0);
            msg.twist.covariance[13] = covVel_body(2, 1);
            msg.twist.covariance[14] = covVel_body(2, 2);
        } else
        {
            msg.twist.covariance[0] = -1.0;
            msg.twist.covariance[7] = -1.0;
            msg.twist.covariance[14] = -1.0;
        }
        // Autocovariances of angular velocity
        msg.twist.covariance[21] = -1.0;
        msg.twist.covariance[28] = -1.0;
        msg.twist.covariance[35] = -1.0;
    }

    /**
     * The position_covariance array is populated in row-major order, where the basis
     * of the corresponding matrix is ENU (so Cov_lonlon is in location 11 of the
     * matrix). The B2b signal type of BeiDou is not checked for usage, since the
     * SignalInfo field of the PVTGeodetic block does not disclose it. For that, one
     * would need to go to the ObsInfo field of the MeasEpochChannelType1 sub-block.
     */
    void MessageHandler::assembleNavSatFix()
    {
        if (!settings_->publish_navsatfix)
            return;

        static auto last_ins_tow = last_insnavgeod_.block_header.tow;

        NavSatFixMsg msg;
        uint16_t mask = 15; // We extract the first four bits using this mask.
        if (settings_->septentrio_receiver_type == "gnss")
        {
            if ((!validValue(last_pvtgeodetic_.block_header.tow)) ||
                (last_pvtgeodetic_.block_header.tow !=
                 last_poscovgeodetic_.block_header.tow))
                return;

            msg.header = last_pvtgeodetic_.header;

            uint16_t type_of_pvt = ((uint16_t)(last_pvtgeodetic_.mode)) & mask;
            switch (type_of_pvt)
            {
            case evNoPVT:
            {
                msg.status.status = NavSatStatusMsg::STATUS_NO_FIX;
                break;
            }
            case evStandAlone:
            case evFixed:
            {
                msg.status.status = NavSatStatusMsg::STATUS_FIX;
                break;
            }
            case evDGPS:
            case evRTKFixed:
            case evRTKFloat:
            case evMovingBaseRTKFixed:
            case evMovingBaseRTKFloat:
            case evPPP:
            {
                msg.status.status = NavSatStatusMsg::STATUS_GBAS_FIX;
                break;
            }
            case evSBAS:
            {
                msg.status.status = NavSatStatusMsg::STATUS_SBAS_FIX;
                break;
            }
            default:
            {
                node_->log(
                    log_level::DEBUG,
                    "PVTGeodetic's Mode field contains an invalid type of PVT solution.");
                break;
            }
            }
            bool gps_in_pvt = false;
            bool glo_in_pvt = false;
            bool com_in_pvt = false;
            bool gal_in_pvt = false;
            uint32_t mask_2 = 1;
            for (int bit = 0; bit != 31; ++bit)
            {
                bool in_use = last_pvtgeodetic_.signal_info & mask_2;
                if (bit <= 5 && in_use)
                {
                    gps_in_pvt = true;
                }
                if (8 <= bit && bit <= 12 && in_use)
                    glo_in_pvt = true;
                if (((13 <= bit && bit <= 14) || (28 <= bit && bit <= 30)) && in_use)
                    com_in_pvt = true;
                if ((bit == 17 || (19 <= bit && bit <= 22)) && in_use)
                    gal_in_pvt = true;
                mask_2 *= 2;
            }
            // Below, booleans will be promoted to integers automatically.
            uint16_t service =
                gps_in_pvt * 1 + glo_in_pvt * 2 + com_in_pvt * 4 + gal_in_pvt * 8;
            msg.status.service = service;
            msg.latitude = rad2deg(last_pvtgeodetic_.latitude);
            msg.longitude = rad2deg(last_pvtgeodetic_.longitude);
            msg.altitude = last_pvtgeodetic_.height;
            msg.position_covariance[0] = last_poscovgeodetic_.cov_lonlon;
            msg.position_covariance[1] = last_poscovgeodetic_.cov_latlon;
            msg.position_covariance[2] = last_poscovgeodetic_.cov_lonhgt;
            msg.position_covariance[3] = last_poscovgeodetic_.cov_latlon;
            msg.position_covariance[4] = last_poscovgeodetic_.cov_latlat;
            msg.position_covariance[5] = last_poscovgeodetic_.cov_lathgt;
            msg.position_covariance[6] = last_poscovgeodetic_.cov_lonhgt;
            msg.position_covariance[7] = last_poscovgeodetic_.cov_lathgt;
            msg.position_covariance[8] = last_poscovgeodetic_.cov_hgthgt;
            msg.position_covariance_type = NavSatFixMsg::COVARIANCE_TYPE_KNOWN;
        } else if (settings_->septentrio_receiver_type == "ins")
        {
            if ((!validValue(last_insnavgeod_.block_header.tow)) ||
                (last_insnavgeod_.block_header.tow == last_ins_tow))
            {
                return;
            }
            last_ins_tow = last_insnavgeod_.block_header.tow;

            msg.header = last_insnavgeod_.header;

            uint16_t type_of_pvt = ((uint16_t)(last_insnavgeod_.gnss_mode)) & mask;
            switch (type_of_pvt)
            {
            case evNoPVT:
            {
                msg.status.status = NavSatStatusMsg::STATUS_NO_FIX;
                break;
            }
            case evStandAlone:
            case evFixed:
            {
                msg.status.status = NavSatStatusMsg::STATUS_FIX;
                break;
            }
            case evDGPS:
            case evRTKFixed:
            case evRTKFloat:
            case evMovingBaseRTKFixed:
            case evMovingBaseRTKFloat:
            case evPPP:
            {
                msg.status.status = NavSatStatusMsg::STATUS_GBAS_FIX;
                break;
            }
            case evSBAS:
            {
                msg.status.status = NavSatStatusMsg::STATUS_SBAS_FIX;
                break;
            }
            default:
            {
                node_->log(
                    log_level::DEBUG,
                    "INSNavGeod's Mode field contains an invalid type of PVT solution.");
                break;
            }
            }
            bool gps_in_pvt = false;
            bool glo_in_pvt = false;
            bool com_in_pvt = false;
            bool gal_in_pvt = false;
            uint32_t mask_2 = 1;
            for (int bit = 0; bit != 31; ++bit)
            {
                bool in_use = last_pvtgeodetic_.signal_info & mask_2;
                if (bit <= 5 && in_use)
                {
                    gps_in_pvt = true;
                }
                if (8 <= bit && bit <= 12 && in_use)
                    glo_in_pvt = true;
                if (((13 <= bit && bit <= 14) || (28 <= bit && bit <= 30)) && in_use)
                    com_in_pvt = true;
                if ((bit == 17 || (19 <= bit && bit <= 22)) && in_use)
                    gal_in_pvt = true;
                mask_2 *= 2;
            }
            // Below, booleans will be promoted to integers automatically.
            uint16_t service =
                gps_in_pvt * 1 + glo_in_pvt * 2 + com_in_pvt * 4 + gal_in_pvt * 8;
            msg.status.service = service;
            msg.latitude = rad2deg(last_insnavgeod_.latitude);
            msg.longitude = rad2deg(last_insnavgeod_.longitude);
            msg.altitude = last_insnavgeod_.height;

            if ((last_insnavgeod_.sb_list & 1) != 0)
            {
                msg.position_covariance[0] =
                    square(last_insnavgeod_.longitude_std_dev);
                msg.position_covariance[4] =
                    square(last_insnavgeod_.latitude_std_dev);
                msg.position_covariance[8] = square(last_insnavgeod_.height_std_dev);
            }
            if ((last_insnavgeod_.sb_list & 32) != 0)
            {
                msg.position_covariance[1] = last_insnavgeod_.latitude_longitude_cov;
                msg.position_covariance[2] = last_insnavgeod_.longitude_height_cov;
                msg.position_covariance[3] = last_insnavgeod_.latitude_longitude_cov;
                msg.position_covariance[5] = last_insnavgeod_.latitude_height_cov;
                msg.position_covariance[6] = last_insnavgeod_.longitude_height_cov;
                msg.position_covariance[7] = last_insnavgeod_.latitude_height_cov;
            }
            msg.position_covariance_type =
                NavSatFixMsg::COVARIANCE_TYPE_DIAGONAL_KNOWN;
        }
        publish<NavSatFixMsg>("navsatfix", msg);
    };

    /**
     * Note that the field "dip" denotes the local magnetic inclination in degrees
     * (positive when the magnetic field points downwards (into the Earth)).
     * This quantity cannot be calculated by most Septentrio
     * receivers. We assume that for the ROS field "err_time", we are requested to
     * provide the 2 sigma uncertainty on the clock bias estimate in square meters,
     * not the clock drift estimate (latter would be
     * "2*std::sqrt(last_velcovgeodetic_.Cov_DtDt)").
     * The "err_track" entry is calculated via the Gaussian error propagation formula
     * from the eastward and the northward velocities. For the formula's usage we
     * have to assume that the eastward and the northward velocities are independent
     * variables. Note that elevations and azimuths of visible satellites are taken
     * from the ChannelStatus block, which provides 1 degree precision, while the
     * SatVisibility block could provide hundredths of degrees precision. Change if
     * imperative for your application... Definition of "visible satellite" adopted
     * here: We define a visible satellite as being !up to! "in sync" mode with the
     * receiver, which corresponds to last_measepoch_.N (signal-to-noise ratios are
     * thereby available for these), though not last_channelstatus_.N, which also
     * includes those "in search". In case certain values appear unphysical, please
     * consult the firmware, since those most likely refer to Do-Not-Use values.
     */
    void MessageHandler::assembleGpsFix()
    {
        if (!settings_->publish_gpsfix)
            return;

        if (settings_->septentrio_receiver_type == "gnss")
        {
            if (!validValue(last_measepoch_.block_header.tow) ||
                !validValue(last_channelstatus_.block_header.tow) ||
                (last_measepoch_.block_header.tow !=
                 last_pvtgeodetic_.block_header.tow) ||
                (last_measepoch_.block_header.tow !=
                 last_poscovgeodetic_.block_header.tow) ||
                (last_measepoch_.block_header.tow !=
                 last_atteuler_.block_header.tow) ||
                (last_measepoch_.block_header.tow !=
                 last_attcoveuler_.block_header.tow))
                return;
        } else if (settings_->septentrio_receiver_type == "ins")
        {
            if (!validValue(last_measepoch_.block_header.tow) ||
                !validValue(last_channelstatus_.block_header.tow) ||
                (last_measepoch_.block_header.tow !=
                 last_insnavgeod_.block_header.tow))
                return;
        }

        GpsFixMsg msg;
        msg.status.satellites_used = static_cast<uint16_t>(last_pvtgeodetic_.nr_sv);

        // MeasEpoch Processing
        std::vector<int32_t> cno_tracked;
        std::vector<int32_t> svid_in_sync;
        {
            cno_tracked.reserve(last_measepoch_.type1.size());
            svid_in_sync.reserve(last_measepoch_.type1.size());
            for (const auto& measepoch_channel_type1 : last_measepoch_.type1)
            {
                // Define MeasEpochChannelType1 struct for the corresponding
                // sub-block
                svid_in_sync.push_back(
                    static_cast<int32_t>(measepoch_channel_type1.sv_id));
                uint8_t type_mask =
                    15; // We extract the first four bits using this mask.
                if (((measepoch_channel_type1.type & type_mask) ==
                     static_cast<uint8_t>(1)) ||
                    ((measepoch_channel_type1.type & type_mask) ==
                     static_cast<uint8_t>(2)))
                {
                    cno_tracked.push_back(
                        static_cast<int32_t>(measepoch_channel_type1.cn0) / 4);
                } else
                {
                    cno_tracked.push_back(
                        static_cast<int32_t>(measepoch_channel_type1.cn0) / 4 +
                        static_cast<int32_t>(10));
                }
            }
        }

        // ChannelStatus Processing
        std::vector<int32_t> svid_in_sync_2;
        std::vector<int32_t> elevation_tracked;
        std::vector<int32_t> azimuth_tracked;
        std::vector<int32_t> svid_pvt;
        svid_pvt.clear();
        std::vector<int32_t> ordering;
        {
            svid_in_sync_2.reserve(last_channelstatus_.satInfo.size());
            elevation_tracked.reserve(last_channelstatus_.satInfo.size());
            azimuth_tracked.reserve(last_channelstatus_.satInfo.size());
            for (const auto& channel_sat_info : last_channelstatus_.satInfo)
            {
                bool to_be_added = false;
                for (int32_t j = 0; j < static_cast<int32_t>(svid_in_sync.size());
                     ++j)
                {
                    if (svid_in_sync[j] ==
                        static_cast<int32_t>(channel_sat_info.sv_id))
                    {
                        ordering.push_back(j);
                        to_be_added = true;
                        break;
                    }
                }
                if (to_be_added)
                {
                    svid_in_sync_2.push_back(
                        static_cast<int32_t>(channel_sat_info.sv_id));
                    elevation_tracked.push_back(
                        static_cast<int32_t>(channel_sat_info.elev));
                    static uint16_t azimuth_mask = 511;
                    azimuth_tracked.push_back(static_cast<int32_t>(
                        (channel_sat_info.az_rise_set & azimuth_mask)));
                }
                svid_pvt.reserve(channel_sat_info.stateInfo.size());
                for (const auto& channel_state_info : channel_sat_info.stateInfo)
                {
                    // Define ChannelStateInfo struct for the corresponding sub-block
                    bool pvt_status = false;
                    uint16_t pvt_status_mask = std::pow(2, 15) + std::pow(2, 14);
                    for (int k = 15; k != -1; k -= 2)
                    {
                        uint16_t pvt_status_value =
                            (channel_state_info.pvt_status & pvt_status_mask) >>
                            k - 1;
                        if (pvt_status_value == 2)
                        {
                            pvt_status = true;
                        }
                        if (k > 1)
                        {
                            pvt_status_mask = pvt_status_mask - std::pow(2, k) -
                                              std::pow(2, k - 1) +
                                              std::pow(2, k - 2) +
                                              std::pow(2, k - 3);
                        }
                    }
                    if (pvt_status)
                    {
                        svid_pvt.push_back(
                            static_cast<int32_t>(channel_sat_info.sv_id));
                    }
                }
            }
        }
        msg.status.satellite_used_prn =
            svid_pvt; // Entries such as int32[] in ROS messages are to be treated as
                      // std::vectors.
        msg.status.satellites_visible = static_cast<uint16_t>(svid_in_sync.size());
        msg.status.satellite_visible_prn = svid_in_sync_2;
        msg.status.satellite_visible_z = elevation_tracked;
        msg.status.satellite_visible_azimuth = azimuth_tracked;

        // Reordering CNO vector to that of all previous arrays
        std::vector<int32_t> cno_tracked_reordered;
        if (static_cast<int32_t>(last_channelstatus_.n) != 0)
        {
            for (int32_t k = 0; k < static_cast<int32_t>(ordering.size()); ++k)
            {
                cno_tracked_reordered.push_back(cno_tracked[ordering[k]]);
            }
        }
        msg.status.satellite_visible_snr = cno_tracked_reordered;
        msg.err_time = 2 * std::sqrt(last_poscovgeodetic_.cov_bb);

        if (settings_->septentrio_receiver_type == "gnss")
        {
            msg.header = last_pvtgeodetic_.header;

            // PVT Status Analysis
            uint16_t status_mask =
                15; // We extract the first four bits using this mask.
            uint16_t type_of_pvt =
                ((uint16_t)(last_pvtgeodetic_.mode)) & status_mask;
            switch (type_of_pvt)
            {
            case evNoPVT:
            {
                msg.status.status = GpsStatusMsg::STATUS_NO_FIX;
                break;
            }
            case evStandAlone:
            case evFixed:
            {
                msg.status.status = GpsStatusMsg::STATUS_FIX;
                break;
            }
            case evDGPS:
            case evRTKFixed:
            case evRTKFloat:
            case evMovingBaseRTKFixed:
            case evMovingBaseRTKFloat:
            case evPPP:
            {
                msg.status.status = GpsStatusMsg::STATUS_GBAS_FIX;
                break;
            }
            case evSBAS:
            {
                uint16_t reference_id = last_pvtgeodetic_.reference_id;
                // Here come the PRNs of the 4 WAAS satellites..
                if (reference_id == 131 || reference_id == 133 ||
                    reference_id == 135 || reference_id == 135)
                {
                    msg.status.status = GpsStatusMsg::STATUS_WAAS_FIX;
                } else
                {
                    msg.status.status = GpsStatusMsg::STATUS_SBAS_FIX;
                }
                break;
            }
            default:
            {
                node_->log(
                    log_level::DEBUG,
                    "PVTGeodetic's Mode field contains an invalid type of PVT solution.");
                break;
            }
            }
            // Doppler is not used when calculating the velocities of, say,
            // mosaic-x5, hence:
            msg.status.motion_source = GpsStatusMsg::SOURCE_POINTS;
            // Doppler is not used when calculating the orientation of, say,
            // mosaic-x5, hence:
            msg.status.orientation_source = GpsStatusMsg::SOURCE_POINTS;
            msg.status.position_source = GpsStatusMsg::SOURCE_GPS;
            msg.latitude = rad2deg(last_pvtgeodetic_.latitude);
            msg.longitude = rad2deg(last_pvtgeodetic_.longitude);
            msg.altitude = last_pvtgeodetic_.height;
            // Note that cog is of type float32 while track is of type float64.
            msg.track = last_pvtgeodetic_.cog;
            msg.speed = std::sqrt(square(last_pvtgeodetic_.vn) +
                                  square(last_pvtgeodetic_.ve));
            msg.climb = last_pvtgeodetic_.vu;

            msg.roll = last_atteuler_.roll;
            msg.pitch = last_atteuler_.pitch;
            msg.dip = last_atteuler_.heading;

            if (last_dop_.pdop == 0.0 || last_dop_.tdop == 0.0)
            {
                msg.gdop = -1.0;
            } else
            {
                msg.gdop =
                    std::sqrt(square(last_dop_.pdop) + square(last_dop_.tdop));
            }
            if (last_dop_.pdop == 0.0)
            {
                msg.pdop = -1.0;
            } else
            {
                msg.pdop = last_dop_.pdop;
            }
            if (last_dop_.hdop == 0.0)
            {
                msg.hdop = -1.0;
            } else
            {
                msg.hdop = last_dop_.hdop;
            }
            if (last_dop_.vdop == 0.0)
            {
                msg.vdop = -1.0;
            } else
            {
                msg.vdop = last_dop_.vdop;
            }
            if (last_dop_.tdop == 0.0)
            {
                msg.tdop = -1.0;
            } else
            {
                msg.tdop = last_dop_.tdop;
            }
            msg.time =
                static_cast<double>(last_pvtgeodetic_.block_header.tow) / 1000 +
                static_cast<double>(last_pvtgeodetic_.block_header.wnc * 604800);
            // position
            msg.err =
                2 *
                (std::sqrt(static_cast<double>(last_poscovgeodetic_.cov_latlat) +
                           static_cast<double>(last_poscovgeodetic_.cov_lonlon) +
                           static_cast<double>(last_poscovgeodetic_.cov_hgthgt)));
            msg.err_horz =
                2 *
                (std::sqrt(static_cast<double>(last_poscovgeodetic_.cov_latlat) +
                           static_cast<double>(last_poscovgeodetic_.cov_lonlon)));
            msg.err_vert =
                2 * std::sqrt(static_cast<double>(last_poscovgeodetic_.cov_hgthgt));
            // motion
            msg.err_track =
                2 * (std::sqrt(square(1.0 / (last_pvtgeodetic_.vn +
                                             square(last_pvtgeodetic_.ve) /
                                                 last_pvtgeodetic_.vn)) *
                                   last_poscovgeodetic_.cov_lonlon +
                               square((last_pvtgeodetic_.ve) /
                                      (square(last_pvtgeodetic_.vn) +
                                       square(last_pvtgeodetic_.ve))) *
                                   last_poscovgeodetic_.cov_latlat));
            msg.err_speed =
                2 * (std::sqrt(static_cast<double>(last_velcovgeodetic_.cov_vnvn) +
                               static_cast<double>(last_velcovgeodetic_.cov_veve)));
            msg.err_climb =
                2 * std::sqrt(static_cast<double>(last_velcovgeodetic_.cov_vuvu));
            // attitude
            msg.err_roll =
                2 * std::sqrt(static_cast<double>(last_attcoveuler_.cov_rollroll));
            msg.err_pitch =
                2 * std::sqrt(static_cast<double>(last_attcoveuler_.cov_pitchpitch));
            msg.err_dip =
                2 * std::sqrt(static_cast<double>(last_attcoveuler_.cov_headhead));

            msg.position_covariance[0] = last_poscovgeodetic_.cov_lonlon;
            msg.position_covariance[1] = last_poscovgeodetic_.cov_latlon;
            msg.position_covariance[2] = last_poscovgeodetic_.cov_lonhgt;
            msg.position_covariance[3] = last_poscovgeodetic_.cov_latlon;
            msg.position_covariance[4] = last_poscovgeodetic_.cov_latlat;
            msg.position_covariance[5] = last_poscovgeodetic_.cov_lathgt;
            msg.position_covariance[6] = last_poscovgeodetic_.cov_lonhgt;
            msg.position_covariance[7] = last_poscovgeodetic_.cov_lathgt;
            msg.position_covariance[8] = last_poscovgeodetic_.cov_hgthgt;
            msg.position_covariance_type = NavSatFixMsg::COVARIANCE_TYPE_KNOWN;
        } else if (settings_->septentrio_receiver_type == "ins")
        {
            msg.header = last_insnavgeod_.header;

            // PVT Status Analysis
            uint16_t status_mask =
                15; // We extract the first four bits using this mask.
            uint16_t type_of_pvt =
                ((uint16_t)(last_insnavgeod_.gnss_mode)) & status_mask;
            switch (type_of_pvt)
            {
            case evNoPVT:
            {
                msg.status.status = GpsStatusMsg::STATUS_NO_FIX;
                break;
            }
            case evStandAlone:
            case evFixed:
            {
                msg.status.status = GpsStatusMsg::STATUS_FIX;
                break;
            }
            case evDGPS:
            case evRTKFixed:
            case evRTKFloat:
            case evMovingBaseRTKFixed:
            case evMovingBaseRTKFloat:
            case evPPP:
            {
                msg.status.status = GpsStatusMsg::STATUS_GBAS_FIX;
                break;
            }
            case evSBAS:
            default:
            {
                node_->log(
                    log_level::DEBUG,
                    "INSNavGeod's Mode field contains an invalid type of PVT solution.");
                break;
            }
            }
            // Doppler is not used when calculating the velocities of, say,
            // mosaic-x5, hence:
            msg.status.motion_source = GpsStatusMsg::SOURCE_POINTS;
            // Doppler is not used when calculating the orientation of, say,
            // mosaic-x5, hence:
            msg.status.orientation_source = GpsStatusMsg::SOURCE_POINTS;
            msg.status.position_source = GpsStatusMsg::SOURCE_GPS;
            msg.latitude = rad2deg(last_insnavgeod_.latitude);
            msg.longitude = rad2deg(last_insnavgeod_.longitude);
            msg.altitude = last_insnavgeod_.height;
            // Note that cog is of type float32 while track is of type float64.
            if ((last_insnavgeod_.sb_list & 2) != 0)
            {
                msg.pitch = last_insnavgeod_.pitch;
                msg.roll = last_insnavgeod_.roll;
                msg.dip = last_insnavgeod_.heading;
            }
            if ((last_insnavgeod_.sb_list & 8) != 0)
            {
                msg.speed = std::sqrt(square(last_insnavgeod_.vn) +
                                      square(last_insnavgeod_.ve));

                msg.climb = last_insnavgeod_.vu;
                msg.track = std::atan2(last_insnavgeod_.vn, last_insnavgeod_.ve);
            }
            if (last_dop_.pdop == 0.0 || last_dop_.tdop == 0.0)
            {
                msg.gdop = -1.0;
            } else
            {
                msg.gdop =
                    std::sqrt(square(last_dop_.pdop) + square(last_dop_.tdop));
            }
            if (last_dop_.pdop == 0.0)
            {
                msg.pdop = -1.0;
            } else
            {
                msg.pdop = last_dop_.pdop;
            }
            if (last_dop_.hdop == 0.0)
            {
                msg.hdop = -1.0;
            } else
            {
                msg.hdop = last_dop_.hdop;
            }
            if (last_dop_.vdop == 0.0)
            {
                msg.vdop = -1.0;
            } else
            {
                msg.vdop = last_dop_.vdop;
            }
            if (last_dop_.tdop == 0.0)
            {
                msg.tdop = -1.0;
            } else
            {
                msg.tdop = last_dop_.tdop;
            }
            msg.time =
                static_cast<double>(last_insnavgeod_.block_header.tow) / 1000 +
                static_cast<double>(last_insnavgeod_.block_header.wnc * 604800);
            if ((last_insnavgeod_.sb_list & 1) != 0)
            {
                msg.err = 2 * (std::sqrt(square(last_insnavgeod_.latitude_std_dev) +
                                         square(last_insnavgeod_.longitude_std_dev) +
                                         square(last_insnavgeod_.height_std_dev)));
                msg.err_horz =
                    2 * (std::sqrt(square(last_insnavgeod_.latitude_std_dev) +
                                   square(last_insnavgeod_.longitude_std_dev)));
                msg.err_vert = 2 * last_insnavgeod_.height_std_dev;
            }
            if (((last_insnavgeod_.sb_list & 8) != 0) ||
                ((last_insnavgeod_.sb_list & 1) != 0))
            {
                msg.err_track =
                    2 * (std::sqrt(square(1.0 / (last_insnavgeod_.vn +
                                                 square(last_insnavgeod_.ve) /
                                                     last_insnavgeod_.vn)) *
                                       square(last_insnavgeod_.longitude_std_dev) +
                                   square((last_insnavgeod_.ve) /
                                          (square(last_insnavgeod_.vn) +
                                           square(last_insnavgeod_.ve))) *
                                       square(last_insnavgeod_.latitude_std_dev)));
            }
            if ((last_insnavgeod_.sb_list & 8) != 0)
            {
                msg.err_speed = 2 * (std::sqrt(square(last_insnavgeod_.ve_std_dev) +
                                               square(last_insnavgeod_.vn_std_dev)));
                msg.err_climb = 2 * last_insnavgeod_.vu_std_dev;
            }
            if ((last_insnavgeod_.sb_list & 2) != 0)
            {
                msg.err_pitch = 2 * last_insnavgeod_.pitch_std_dev;
                msg.err_roll = 2 * last_insnavgeod_.roll_std_dev;
                msg.err_dip = 2 * last_insnavgeod_.heading_std_dev;
            }
            if ((last_insnavgeod_.sb_list & 1) != 0)
            {
                msg.position_covariance[0] =
                    square(last_insnavgeod_.longitude_std_dev);
                msg.position_covariance[4] =
                    square(last_insnavgeod_.latitude_std_dev);
                msg.position_covariance[8] = square(last_insnavgeod_.height_std_dev);
            }
            if ((last_insnavgeod_.sb_list & 32) != 0)
            {
                msg.position_covariance[1] = last_insnavgeod_.latitude_longitude_cov;
                msg.position_covariance[2] = last_insnavgeod_.longitude_height_cov;
                msg.position_covariance[3] = last_insnavgeod_.latitude_longitude_cov;
                msg.position_covariance[5] = last_insnavgeod_.latitude_height_cov;
                msg.position_covariance[6] = last_insnavgeod_.longitude_height_cov;
                msg.position_covariance[7] = last_insnavgeod_.latitude_height_cov;
            }
            msg.position_covariance_type =
                NavSatFixMsg::COVARIANCE_TYPE_DIAGONAL_KNOWN;
        }
        publish<GpsFixMsg>("gpsfix", msg);
    }

    void
    MessageHandler::assembleTimeReference(const std::shared_ptr<Telegram>& telegram)
    {
        TimeReferenceMsg msg;
        Timestamp time_obj = timestampSBF(telegram->message);
        msg.time_ref = timestampToRos(time_obj);
        msg.source = "GPST";
        assembleHeader(settings_->frame_id, telegram, msg);
        publish<TimeReferenceMsg>("gpst", msg);
    }

    template <typename T>
    void MessageHandler::assembleHeader(const std::string& frameId,
                                        const std::shared_ptr<Telegram>& telegram,
                                        T& msg) const
    {
        Timestamp time_obj = settings_->use_gnss_time
                                 ? timestampSBF(telegram->message)
                                 : telegram->stamp;
        msg.header.frame_id = frameId;

        if (!settings_->use_gnss_time && settings_->latency_compensation)
        {
            if constexpr (std::is_same<INSNavCartMsg, T>::value ||
                          std::is_same<INSNavGeodMsg, T>::value)
            {
                time_obj -= msg.latency * 100000ul; // from 0.0001 s to ns
            } else if constexpr (std::is_same<PVTCartesianMsg, T>::value ||
                                 std::is_same<PVTGeodeticMsg, T>::value)
            {
                last_pvt_latency_ = msg.latency * 100000ul; // from 0.0001 s to ns
                time_obj -= last_pvt_latency_;
            } else if constexpr (std::is_same<PosCovCartesianMsg, T>::value ||
                                 std::is_same<PosCovGeodeticMsg, T>::value ||
                                 std::is_same<VelCovCartesianMsg, T>::value ||
                                 std::is_same<VelCovGeodeticMsg, T>::value ||
                                 std::is_same<AttEulerMsg, T>::value ||
                                 std::is_same<AttCovEulerMsg, T>::value ||
                                 std::is_same<BaseVectorCartMsg, T>::value ||
                                 std::is_same<BaseVectorGeodMsg, T>::value)
            {
                time_obj -= last_pvt_latency_;
            }
        }

        msg.header.stamp = timestampToRos(time_obj);
    }

    Timestamp MessageHandler::timestampSBF(const std::vector<uint8_t>& message) const
    {
        uint32_t tow = parsing_utilities::getTow(message);
        uint16_t wnc = parsing_utilities::getWnc(message);

        if (!validValue(tow) || !validValue(wnc))
            return 0;

        return timestampSBF(tow, wnc);
    }

    /// If the current time shall be employed, it is calculated via the time(NULL)
    /// function found in the \<ctime\> library At the time of writing the code
    /// (2020), the GPS time was ahead of UTC time by 18 (leap) seconds. Adapt the
    /// settings_->leap_seconds ROSaic parameter accordingly as soon as the
    /// next leap second is inserted into the UTC time.
    Timestamp MessageHandler::timestampSBF(uint32_t tow, uint16_t wnc) const
    {
        Timestamp time_obj;

        // conversion from GPS time of week and week number to UTC taking leap
        // seconds into account
        static uint64_t secToNSec = 1000000000;
        static uint64_t mSec2NSec = 1000000;
        static uint64_t nsOfGpsStart =
            315964800 *
            secToNSec; // GPS week counter starts at 1980-01-06 which is
                       // 315964800 seconds since Unix epoch (1970-01-01 UTC)
        static uint64_t nsecPerWeek = 7 * 24 * 60 * 60 * secToNSec;

        time_obj = nsOfGpsStart + tow * mSec2NSec + wnc * nsecPerWeek;

        if (current_leap_seconds_ != -128)
            time_obj -= current_leap_seconds_ * secToNSec;
        // else: warn?

        return time_obj;
    }

    /**
     * If GNSS time is used, Publishing is only done with valid leap seconds
     */
    template <typename M>
    void MessageHandler::publish(const std::string& topic, const M& msg)
    {
        // TODO: maybe publish only if wnc and tow is valid?
        if (!settings_->use_gnss_time ||
            (settings_->use_gnss_time && (current_leap_seconds_ != -128)))
        {
            if (settings_->read_from_sbf_log || settings_->read_from_pcap)
            {
                wait(timestampFromRos(msg.header.stamp));
            }
            node_->publishMessage<M>(topic, msg);
        } else
        {
            node_->log(
                log_level::DEBUG,
                "Not publishing message with GNSS time because no leap seconds are available yet.");
            if (settings_->read_from_sbf_log || settings_->read_from_pcap)
            {
                node_->log(
                    log_level::WARN,
                    "No leap seconds were set and none were received from log yet.");
                setLeapSeconds();
            }
        }
    }

    /**
     * If GNSS time is used, Publishing is only done with valid leap seconds
     */
    void MessageHandler::publishTf(const LocalizationMsg& msg)
    {
        // TODO: maybe publish only if wnc and tow is valid?
        if (!settings_->use_gnss_time ||
            (settings_->use_gnss_time && (current_leap_seconds_ != -128)))
        {
            if (settings_->read_from_sbf_log || settings_->read_from_pcap)
            {
                wait(timestampFromRos(msg.header.stamp));
            }
            node_->publishTf(msg);
        } else
        {
            node_->log(
                log_level::DEBUG,
                "Not publishing tf with GNSS time because no leap seconds are available yet.");
            if (settings_->read_from_sbf_log || settings_->read_from_pcap)
            {
                node_->log(
                    log_level::WARN,
                    "No leap seconds were set and none were received from log yet. ");
                setLeapSeconds();
            };
        }
    }

    void MessageHandler::parseSbf(const std::shared_ptr<Telegram>& telegram)
    {

        uint16_t sbfId = parsing_utilities::getId(telegram->message);

        /*node_->log(log_level::DEBUG, "ROSaic reading SBF block " +
                                        std::to_string(sbfId) + " made up of " +
                                        std::to_string(telegram->message.size()) +
                                        " bytes...");*/

        switch (sbfId)
        {
        case PVT_CARTESIAN: // Position and velocity in XYZ
        {
            if (settings_->publish_pvtcartesian)
            {
                PVTCartesianMsg msg;

                if (!PVTCartesianParser(node_, telegram->message.begin(),
                                        telegram->message.end(), msg))
                {
                    node_->log(log_level::ERROR, "parse error in PVTCartesian");
                    break;
                }
                assembleHeader(settings_->frame_id, telegram, msg);
                publish<PVTCartesianMsg>("pvtcartesian", msg);
            }
            break;
        }
        case PVT_GEODETIC: // Position and velocity in geodetic coordinate frame
                           // (ENU frame)
        {
            if (!PVTGeodeticParser(node_, telegram->message.begin(),
                                   telegram->message.end(), last_pvtgeodetic_))
            {
                node_->log(log_level::ERROR, "parse error in PVTGeodetic");
                break;
            }
            assembleHeader(settings_->frame_id, telegram, last_pvtgeodetic_);
            if (settings_->publish_pvtgeodetic)
                publish<PVTGeodeticMsg>("pvtgeodetic", last_pvtgeodetic_);
            assembleTwist();
            assembleNavSatFix();
            assemblePoseWithCovarianceStamped();
            assembleGpsFix();
            if (settings_->publish_gpst &&
                (settings_->septentrio_receiver_type == "gnss"))
                assembleTimeReference(telegram);
            break;
        }
        case BASE_VECTOR_CART:
        {
            if (settings_->publish_basevectorcart)
            {
                BaseVectorCartMsg msg;

                if (!BaseVectorCartParser(node_, telegram->message.begin(),
                                          telegram->message.end(), msg))
                {
                    node_->log(log_level::ERROR, "parse error in BaseVectorCart");
                    break;
                }
                assembleHeader(settings_->frame_id, telegram, msg);
                publish<BaseVectorCartMsg>("basevectorcart", msg);
            }
            break;
        }
        case BASE_VECTOR_GEOD:
        {
            if (settings_->publish_basevectorgeod)
            {
                BaseVectorGeodMsg msg;

                if (!BaseVectorGeodParser(node_, telegram->message.begin(),
                                          telegram->message.end(), msg))
                {
                    node_->log(log_level::ERROR, "parse error in BaseVectorGeod");
                    break;
                }
                assembleHeader(settings_->frame_id, telegram, msg);
                publish<BaseVectorGeodMsg>("basevectorgeod", msg);
            }
            break;
        }
        case POS_COV_CARTESIAN:
        {
            if (settings_->publish_poscovcartesian)
            {
                PosCovCartesianMsg msg;

                if (!PosCovCartesianParser(node_, telegram->message.begin(),
                                           telegram->message.end(), msg))
                {
                    node_->log(log_level::ERROR, "parse error in PosCovCartesian");
                    break;
                }
                assembleHeader(settings_->frame_id, telegram, msg);
                publish<PosCovCartesianMsg>("poscovcartesian", msg);
            }
            break;
        }
        case POS_COV_GEODETIC:
        {
            if (!PosCovGeodeticParser(node_, telegram->message.begin(),
                                      telegram->message.end(), last_poscovgeodetic_))
            {
                node_->log(log_level::ERROR, "parse error in PosCovGeodetic");
                break;
            }
            assembleHeader(settings_->frame_id, telegram, last_poscovgeodetic_);
            if (settings_->publish_poscovgeodetic)
                publish<PosCovGeodeticMsg>("poscovgeodetic", last_poscovgeodetic_);
            assembleNavSatFix();
            assemblePoseWithCovarianceStamped();
            assembleGpsFix();
            break;
        }
        case ATT_EULER:
        {
            if (!AttEulerParser(node_, telegram->message.begin(),
                                telegram->message.end(), last_atteuler_,
                                settings_->use_ros_axis_orientation))
            {
                node_->log(log_level::ERROR, "parse error in AttEuler");
                break;
            }
            assembleHeader(settings_->frame_id, telegram, last_atteuler_);
            if (settings_->publish_atteuler)
                publish<AttEulerMsg>("atteuler", last_atteuler_);
            assemblePoseWithCovarianceStamped();
            assembleGpsFix();
            break;
        }
        case ATT_COV_EULER:
        {
            if (!AttCovEulerParser(node_, telegram->message.begin(),
                                   telegram->message.end(), last_attcoveuler_,
                                   settings_->use_ros_axis_orientation))
            {
                node_->log(log_level::ERROR, "parse error in AttCovEuler");
                break;
            }
            assembleHeader(settings_->frame_id, telegram, last_attcoveuler_);
            if (settings_->publish_attcoveuler)
                publish<AttCovEulerMsg>("attcoveuler", last_attcoveuler_);
            assemblePoseWithCovarianceStamped();
            assembleGpsFix();
            break;
        }
        case GAL_AUTH_STATUS:
        {
            if (!GalAuthStatusParser(node_, telegram->message.begin(),
                                     telegram->message.end(), last_gal_auth_status_))
            {
                node_->log(log_level::ERROR, "parse error in GalAuthStatus");
                break;
            }
            osnma_info_available_ = true;
            assembleHeader(settings_->frame_id, telegram, last_gal_auth_status_);
            if (settings_->publish_galauthstatus)
            {
                publish<GalAuthStatusMsg>("galauthstatus", last_gal_auth_status_);
                assembleOsnmaDiagnosticArray();
            }
            break;
        }
        case RF_STATUS:
        {
            if (!RfStatusParser(node_, telegram->message.begin(),
                                telegram->message.end(), last_rf_status_))
            {
                node_->log(log_level::ERROR, "parse error inRfStatus");
                break;
            }
            assembleHeader(settings_->frame_id, telegram, last_rf_status_);
            if (settings_->publish_aimplusstatus)
            {
                publish<RfStatusMsg>("rfstatus", last_rf_status_);
                assembleAimAndDiagnosticArray();
            }
            break;
        }
        case INS_NAV_CART: // Position, velocity and orientation in cartesian
                           // coordinate frame (ENU frame)
        {
            if (!INSNavCartParser(node_, telegram->message.begin(),
                                  telegram->message.end(), last_insnavcart_,
                                  settings_->use_ros_axis_orientation))
            {
                node_->log(log_level::ERROR, "parse error in INSNavCart");
                break;
            }
            std::string frame_id;
            if (settings_->ins_use_poi)
            {
                frame_id = settings_->poi_frame_id;
            } else
            {
                frame_id = settings_->frame_id;
            }
            assembleHeader(frame_id, telegram, last_insnavcart_);
            if (settings_->publish_insnavcart)
                publish<INSNavCartMsg>("insnavcart", last_insnavcart_);
            assembleLocalizationEcef();
            break;
        }
        case INS_NAV_GEOD: // Position, velocity and orientation in geodetic
                           // coordinate frame (ENU frame)
        {
            if (!INSNavGeodParser(node_, telegram->message.begin(),
                                  telegram->message.end(), last_insnavgeod_,
                                  settings_->use_ros_axis_orientation))
            {
                node_->log(log_level::ERROR, "parse error in INSNavGeod");
                break;
            }
            std::string frame_id;
            if (settings_->ins_use_poi)
            {
                frame_id = settings_->poi_frame_id;
            } else
            {
                frame_id = settings_->frame_id;
            }
            assembleHeader(frame_id, telegram, last_insnavgeod_);
            if (settings_->publish_insnavgeod)
                publish<INSNavGeodMsg>("insnavgeod", last_insnavgeod_);
            assembleLocalizationUtm();
            assembleLocalizationEcef();
            assembleTwist(true);
            assemblePoseWithCovarianceStamped();
            assembleNavSatFix();
            assembleGpsFix();
            if (settings_->publish_gpst)
                assembleTimeReference(telegram);
            break;
        }
        case IMU_SETUP: // IMU orientation and lever arm
        {
            if (settings_->publish_imusetup)
            {
                IMUSetupMsg msg;

                if (!IMUSetupParser(node_, telegram->message.begin(),
                                    telegram->message.end(), msg,
                                    settings_->use_ros_axis_orientation))
                {
                    node_->log(log_level::ERROR, "parse error in IMUSetup");
                    break;
                }
                assembleHeader(settings_->vehicle_frame_id, telegram, msg);
                publish<IMUSetupMsg>("imusetup", msg);
            }
            break;
        }
        case VEL_SENSOR_SETUP: // Velocity sensor lever arm
        {
            if (settings_->publish_velcovgeodetic)
            {
                VelSensorSetupMsg msg;

                if (!VelSensorSetupParser(node_, telegram->message.begin(),
                                          telegram->message.end(), msg,
                                          settings_->use_ros_axis_orientation))
                {
                    node_->log(log_level::ERROR, "parse error in VelSensorSetup");
                    break;
                }
                assembleHeader(settings_->vehicle_frame_id, telegram, msg);
                publish<VelSensorSetupMsg>("velsensorsetup", msg);
            }
            break;
        }
        case EXT_EVENT_INS_NAV_CART: // Position, velocity and orientation in
                                     // cartesian coordinate frame (ENU frame)
        {
            if (settings_->publish_exteventinsnavcart)
            {
                INSNavCartMsg msg;

                if (!INSNavCartParser(node_, telegram->message.begin(),
                                      telegram->message.end(), msg,
                                      settings_->use_ros_axis_orientation))
                {
                    node_->log(log_level::ERROR,
                               "parse error in ExtEventINSNavCart");
                    break;
                }
                std::string frame_id;
                if (settings_->ins_use_poi)
                {
                    frame_id = settings_->poi_frame_id;
                } else
                {
                    frame_id = settings_->frame_id;
                }
                assembleHeader(frame_id, telegram, msg);
                publish<INSNavCartMsg>("exteventinsnavcart", msg);
            }
            break;
        }
        case EXT_EVENT_INS_NAV_GEOD:
        {
            if (settings_->publish_exteventinsnavgeod)
            {
                INSNavGeodMsg msg;

                if (!INSNavGeodParser(node_, telegram->message.begin(),
                                      telegram->message.end(), msg,
                                      settings_->use_ros_axis_orientation))
                {
                    node_->log(log_level::ERROR,
                               "parse error in ExtEventINSNavGeod");
                    break;
                }
                std::string frame_id;
                if (settings_->ins_use_poi)
                {
                    frame_id = settings_->poi_frame_id;
                } else
                {
                    frame_id = settings_->frame_id;
                }
                assembleHeader(frame_id, telegram, msg);
                publish<INSNavGeodMsg>("exteventinsnavgeod", msg);
            }
            break;
        }
        case EXT_SENSOR_MEAS:
        {
            bool hasImuMeas = false;
            if (!ExtSensorMeasParser(node_, telegram->message.begin(),
                                     telegram->message.end(), last_extsensmeas_,
                                     settings_->use_ros_axis_orientation,
                                     hasImuMeas))
            {
                node_->log(log_level::ERROR, "parse error in ExtSensorMeas");
                break;
            }
            assembleHeader(settings_->imu_frame_id, telegram, last_extsensmeas_);
            if (settings_->publish_extsensormeas)
                publish<ExtSensorMeasMsg>("extsensormeas", last_extsensmeas_);
            if (settings_->publish_imu && hasImuMeas)
            {
                assembleImu();
            }
            break;
        }
        case CHANNEL_STATUS:
        {
            if (!ChannelStatusParser(node_, telegram->message.begin(),
                                     telegram->message.end(), last_channelstatus_))
            {
                node_->log(log_level::ERROR, "parse error in ChannelStatus");
                break;
            }
            assembleGpsFix();
            break;
        }
        case MEAS_EPOCH:
        {
            if (!MeasEpochParser(node_, telegram->message.begin(),
                                 telegram->message.end(), last_measepoch_))
            {
                node_->log(log_level::ERROR, "parse error in MeasEpoch");
                break;
            }
            assembleHeader(settings_->frame_id, telegram, last_measepoch_);
            if (settings_->publish_measepoch)
                publish<MeasEpochMsg>("measepoch", last_measepoch_);
            assembleGpsFix();
            break;
        }
        case DOP:
        {
            if (!DOPParser(node_, telegram->message.begin(), telegram->message.end(),
                           last_dop_))
            {
                node_->log(log_level::ERROR, "parse error in DOP");
                break;
            }
            assembleGpsFix();
            break;
        }
        case VEL_COV_CARTESIAN:
        {
            if (settings_->publish_velcovcartesian)
            {
                VelCovCartesianMsg msg;
                if (!VelCovCartesianParser(node_, telegram->message.begin(),
                                           telegram->message.end(), msg))
                {
                    node_->log(log_level::ERROR, "parse error in VelCovCartesian");
                    break;
                }
                assembleHeader(settings_->frame_id, telegram, msg);
                publish<VelCovCartesianMsg>("velcovcartesian", msg);
            }
            break;
        }
        case VEL_COV_GEODETIC:
        {

            if (!VelCovGeodeticParser(node_, telegram->message.begin(),
                                      telegram->message.end(), last_velcovgeodetic_))
            {
                node_->log(log_level::ERROR, "parse error in VelCovGeodetic");
                break;
            }
            assembleHeader(settings_->frame_id, telegram, last_velcovgeodetic_);
            if (settings_->publish_velcovgeodetic)
                publish<VelCovGeodeticMsg>("velcovgeodetic", last_velcovgeodetic_);
            assembleTwist();
            assembleGpsFix();
            break;
        }
        case RECEIVER_STATUS:
        {
            if (!ReceiverStatusParser(node_, telegram->message.begin(),
                                      telegram->message.end(), last_receiverstatus_))
            {
                node_->log(log_level::ERROR, "parse error in ReceiverStatus");
                break;
            }
            assembleDiagnosticArray(telegram);
            break;
        }
        case QUALITY_IND:
        {
            if (!QualityIndParser(node_, telegram->message.begin(),
                                  telegram->message.end(), last_qualityind_))
            {
                node_->log(log_level::ERROR, "parse error in QualityInd");
                break;
            }
            assembleDiagnosticArray(telegram);
            break;
        }
        case RECEIVER_SETUP:
        {
            if (!ReceiverSetupParser(node_, telegram->message.begin(),
                                     telegram->message.end(), last_receiversetup_))
            {
                node_->log(log_level::ERROR, "parse error in ReceiverSetup");
                break;
            }
            node_->log(log_level::DEBUG,
                       "receiver setup firmware: " + last_receiversetup_.rx_version);

            static const int32_t ins_major = 1;
            static const int32_t ins_minor = 4;
            static const int32_t ins_patch = 0;
            static const int32_t gnss_major = 4;
            static const int32_t gnss_minor = 12;
            static const int32_t gnss_patch = 1;
            boost::tokenizer<> tok(last_receiversetup_.rx_version);
            boost::tokenizer<>::iterator it = tok.begin();
            std::vector<int32_t> major_minor_patch;
            major_minor_patch.reserve(3);
            for (boost::tokenizer<>::iterator it = tok.begin(); it != tok.end();
                 ++it)
            {
                int32_t v = std::atoi(it->c_str());
                major_minor_patch.push_back(v);
            }
            if (major_minor_patch.size() < 3)
            {
                node_->log(log_level::ERROR, "parse error of firmware version.");
            } else
            {
                if ((settings_->septentrio_receiver_type == "ins") || node_->isIns())
                {
                    if ((major_minor_patch[0] < ins_major) ||
                        ((major_minor_patch[0] == ins_major) &&
                         (major_minor_patch[1] < ins_minor)) ||
                        ((major_minor_patch[0] == ins_major) &&
                         (major_minor_patch[1] == ins_minor) &&
                         (major_minor_patch[2] < ins_patch)))
                    {
                        node_->log(
                            log_level::INFO,
                            "INS receiver has firmware version: " +
                                last_receiversetup_.rx_version +
                                ", which does not support all features. Please update to at least " +
                                std::to_string(ins_major) + "." +
                                std::to_string(ins_minor) + "." +
                                std::to_string(ins_patch) + " or consult README.");
                    } else
                        node_->setImprovedVsmHandling();
                } else if (settings_->septentrio_receiver_type == "gnss")
                {
                    if ((major_minor_patch[0] < gnss_major) ||
                        ((major_minor_patch[0] == gnss_major) &&
                         (major_minor_patch[1] < gnss_minor)) ||
                        ((major_minor_patch[0] == gnss_major) &&
                         (major_minor_patch[1] == gnss_minor) &&
                         (major_minor_patch[2] < gnss_patch)))
                    {
                        node_->log(
                            log_level::INFO,
                            "GNSS receiver has firmware version: " +
                                last_receiversetup_.rx_version +
                                ", which may not support all features. Please update to at least " +
                                std::to_string(gnss_major) + "." +
                                std::to_string(gnss_minor) + "." +
                                std::to_string(gnss_patch) + " or consult README.");
                    }
                }
            }

            break;
        }
        case RECEIVER_TIME:
        {
            ReceiverTimeMsg msg;

            if (!ReceiverTimeParser(node_, telegram->message.begin(),
                                    telegram->message.end(), msg))
            {
                node_->log(log_level::ERROR, "parse error in ReceiverTime");
                break;
            }
            current_leap_seconds_ = msg.delta_ls;
            break;
        }
        default:
        {
            node_->log(log_level::DEBUG, "unhandled SBF block " +
                                             std::to_string(sbfId) + " received.");
            break;
        }
            // Many more to be implemented...
        }
    }

    void MessageHandler::wait(Timestamp time_obj)
    {
        Timestamp unix_old = unix_time_;
        unix_time_ = time_obj;
        if ((unix_old != 0) && (unix_time_ > unix_old))
        {
            auto sleep_nsec = unix_time_ - unix_old;

            std::stringstream ss;
            ss << "Waiting for " << sleep_nsec / 1000000 << " milliseconds...";
            node_->log(log_level::DEBUG, ss.str());

            std::this_thread::sleep_for(std::chrono::nanoseconds(sleep_nsec));
        }
    }

    void MessageHandler::parseNmea(const std::shared_ptr<Telegram>& telegram)
    {
        std::string message(telegram->message.begin(), telegram->message.end());
        /*node_->log(
          LogLevel::DEBUG,
          "The NMEA message contains " + std::to_string(message.size()) +
              " bytes and is ready to be parsed. It reads: " + message);*/
        boost::char_separator<char> sep_2(",*", "", boost::keep_empty_tokens);
        boost::tokenizer<boost::char_separator<char>> tokens(message, sep_2);
        std::vector<std::string> body;
        body.reserve(20);
        for (boost::tokenizer<boost::char_separator<char>>::iterator tok_iter =
                 tokens.begin();
             tok_iter != tokens.end(); ++tok_iter)
        {
            body.push_back(*tok_iter);
        }

        std::string id(body[0].begin() + 1, body[0].end());

        auto it = nmeaMap_.find(body[0]);
        if (it != nmeaMap_.end())
        {
            switch (it->second)
            {
            case 0:
            {
                // Create NmeaSentence struct to pass to GpggaParser::parseASCII
                NMEASentence gga_message(id, body);
                GpggaMsg msg;
                GpggaParser parser_obj;
                try
                {
                    msg = parser_obj.parseASCII(gga_message, settings_->frame_id,
                                                settings_->use_gnss_time,
                                                telegram->stamp);
                } catch (ParseException& e)
                {
                    node_->log(log_level::DEBUG,
                               "GpggaMsg: " + std::string(e.what()));
                    break;
                }
                publish<GpggaMsg>("gpgga", msg);
                break;
            }
            case 1:
            {
                // Create NmeaSentence struct to pass to GprmcParser::parseASCII
                NMEASentence rmc_message(id, body);
                GprmcMsg msg;
                GprmcParser parser_obj;
                try
                {
                    msg = parser_obj.parseASCII(rmc_message, settings_->frame_id,
                                                settings_->use_gnss_time,
                                                telegram->stamp);
                } catch (ParseException& e)
                {
                    node_->log(log_level::DEBUG,
                               "GprmcMsg: " + std::string(e.what()));
                    break;
                }
                publish<GprmcMsg>("gprmc", msg);
                break;
            }
            case 2:
            {
                // Create NmeaSentence struct to pass to GpgsaParser::parseASCII
                NMEASentence gsa_message(id, body);
                GpgsaMsg msg;
                GpgsaParser parser_obj;
                try
                {
                    msg = parser_obj.parseASCII(gsa_message, settings_->frame_id,
                                                settings_->use_gnss_time,
                                                node_->getTime());
                } catch (ParseException& e)
                {
                    node_->log(log_level::DEBUG,
                               "GpgsaMsg: " + std::string(e.what()));
                    break;
                }
                if (settings_->use_gnss_time)
                {
                    if (settings_->septentrio_receiver_type == "gnss")
                    {
                        Timestamp time_obj =
                            timestampSBF(last_pvtgeodetic_.block_header.tow,
                                         last_pvtgeodetic_.block_header.wnc);
                        msg.header.stamp = timestampToRos(time_obj);
                    }
                    if (settings_->septentrio_receiver_type == "ins")
                    {
                        Timestamp time_obj =
                            timestampSBF(last_insnavgeod_.block_header.tow,
                                         last_insnavgeod_.block_header.wnc);
                        msg.header.stamp = timestampToRos(time_obj);
                    }
                } else
                    msg.header.stamp = timestampToRos(telegram->stamp);
                publish<GpgsaMsg>("gpgsa", msg);
                break;
            }
            case 4:
            {
                // Create NmeaSentence struct to pass to GpgsvParser::parseASCII
                NMEASentence gsv_message(id, body);
                GpgsvMsg msg;
                GpgsvParser parser_obj;
                try
                {
                    msg = parser_obj.parseASCII(gsv_message, settings_->frame_id,
                                                settings_->use_gnss_time,
                                                node_->getTime());
                } catch (ParseException& e)
                {
                    node_->log(log_level::DEBUG,
                               "GpgsvMsg: " + std::string(e.what()));
                    break;
                }
                if (settings_->use_gnss_time)
                {

                    if (settings_->septentrio_receiver_type == "gnss")
                    {
                        Timestamp time_obj =
                            timestampSBF(last_pvtgeodetic_.block_header.tow,
                                         last_pvtgeodetic_.block_header.wnc);
                        msg.header.stamp = timestampToRos(time_obj);
                    }
                    if (settings_->septentrio_receiver_type == "ins")
                    {
                        Timestamp time_obj =
                            timestampSBF(last_insnavgeod_.block_header.tow,
                                         last_insnavgeod_.block_header.wnc);
                        msg.header.stamp = timestampToRos(time_obj);
                    }
                } else
                    msg.header.stamp = timestampToRos(telegram->stamp);
                publish<GpgsvMsg>("gpgsv", msg);
                break;
            }
            }
        } else
        {
            node_->log(log_level::DEBUG, "Unknown NMEA message: " + body[0]);
        }
    }

} // namespace io
