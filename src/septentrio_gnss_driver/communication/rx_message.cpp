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
#include <septentrio_gnss_driver/communication/rx_message.hpp>
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

using parsing_utilities::deg2rad;
using parsing_utilities::deg2radSq;
using parsing_utilities::rad2deg;

PoseWithCovarianceStampedMsg
io_comm_rx::RxMessage::PoseWithCovarianceStampedCallback()
{
    PoseWithCovarianceStampedMsg msg;
    if (settings_->septentrio_receiver_type == "gnss")
    {
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
        msg.pose.pose.orientation = parsing_utilities::convertEulerToQuaternion(
            deg2rad(yaw), deg2rad(pitch), deg2rad(roll));
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
    if (settings_->septentrio_receiver_type == "ins")
    {
        msg.pose.pose.position.x = rad2deg(last_insnavgeod_.longitude);
        msg.pose.pose.position.y = rad2deg(last_insnavgeod_.latitude);
        msg.pose.pose.position.z = last_insnavgeod_.height;

        // Filling in the pose data
        if ((last_insnavgeod_.sb_list & 1) != 0)
        {
            // Pos autocov
            msg.pose.covariance[0] =
                parsing_utilities::square(last_insnavgeod_.longitude_std_dev);
            msg.pose.covariance[7] =
                parsing_utilities::square(last_insnavgeod_.latitude_std_dev);
            msg.pose.covariance[14] =
                parsing_utilities::square(last_insnavgeod_.height_std_dev);
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
            msg.pose.pose.orientation = parsing_utilities::convertEulerToQuaternion(
                deg2rad(yaw), deg2rad(pitch), deg2rad(roll));
        } else
        {
            msg.pose.pose.orientation.w = std::numeric_limits<double>::quiet_NaN();
            msg.pose.pose.orientation.x = std::numeric_limits<double>::quiet_NaN();
            msg.pose.pose.orientation.y = std::numeric_limits<double>::quiet_NaN();
            msg.pose.pose.orientation.z = std::numeric_limits<double>::quiet_NaN();
        }
        if ((last_insnavgeod_.sb_list & 4) != 0)
        {
            // Attitude autocov
            if (validValue(last_insnavgeod_.roll_std_dev))
                msg.pose.covariance[21] = parsing_utilities::square(
                    deg2rad(last_insnavgeod_.roll_std_dev));
            else
                msg.pose.covariance[21] = -1.0;
            if (validValue(last_insnavgeod_.pitch_std_dev))
                msg.pose.covariance[28] = parsing_utilities::square(
                    deg2rad(last_insnavgeod_.pitch_std_dev));
            else
                msg.pose.covariance[28] = -1.0;
            if (validValue(last_insnavgeod_.heading_std_dev))
                msg.pose.covariance[35] = parsing_utilities::square(
                    deg2rad(last_insnavgeod_.heading_std_dev));
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
            msg.pose.covariance[23] = deg2radSq(last_insnavgeod_.heading_roll_cov);
            msg.pose.covariance[27] = deg2radSq(last_insnavgeod_.pitch_roll_cov);

            msg.pose.covariance[29] = deg2radSq(last_insnavgeod_.heading_pitch_cov);
            msg.pose.covariance[33] = deg2radSq(last_insnavgeod_.heading_roll_cov);
            msg.pose.covariance[34] = deg2radSq(last_insnavgeod_.heading_pitch_cov);
        }
    }
    return msg;
};

DiagnosticArrayMsg io_comm_rx::RxMessage::DiagnosticArrayCallback()
{
    DiagnosticArrayMsg msg;
    std::string serialnumber(last_receiversetup_.rx_serial_number);
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
            if (((last_qualityind_.indicators[i] & indicators_value_mask) >> 8) ==
                static_cast<uint16_t>(0))
            {
                gnss_status.level = DiagnosticStatusMsg::STALE;
            } else if (((last_qualityind_.indicators[i] & indicators_value_mask) >>
                        8) == static_cast<uint16_t>(1) ||
                       ((last_qualityind_.indicators[i] & indicators_value_mask) >>
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
    // If the ReceiverStatus's RxError field is not 0, then at least one error has
    // been detected.
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
    gnss_status.name = "gnss";
    gnss_status.message =
        "Quality Indicators (from 0 for low quality to 10 for high quality, 15 if unknown)";
    msg.status.push_back(gnss_status);
    return msg;
};

ImuMsg io_comm_rx::RxMessage::ImuCallback()
{
    ImuMsg msg;

    msg.linear_acceleration.x = last_extsensmeas_.acceleration_x;
    msg.linear_acceleration.y = last_extsensmeas_.acceleration_y;
    msg.linear_acceleration.z = last_extsensmeas_.acceleration_z;

    msg.angular_velocity.x = last_extsensmeas_.angular_rate_x;
    msg.angular_velocity.y = last_extsensmeas_.angular_rate_y;
    msg.angular_velocity.z = last_extsensmeas_.angular_rate_z;

    bool valid_orientation = true;
    if (settings_->septentrio_receiver_type == "ins")
    {
        if (validValue(last_insnavgeod_.block_header.tow))
        {
            Timestamp tsImu = timestampSBF(last_extsensmeas_.block_header.tow,
                                           last_extsensmeas_.block_header.wnc, true);
            Timestamp tsIns = timestampSBF(last_insnavgeod_.block_header.tow,
                                           last_insnavgeod_.block_header.wnc,
                                           true); // Filling in the oreintation data

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
                        msg.orientation =
                            parsing_utilities::convertEulerToQuaternion(
                                deg2rad(last_insnavgeod_.heading),
                                deg2rad(last_insnavgeod_.pitch),
                                deg2rad(last_insnavgeod_.roll));
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
                        msg.orientation_covariance[0] = parsing_utilities::square(
                            deg2rad(last_insnavgeod_.roll_std_dev));
                        msg.orientation_covariance[4] = parsing_utilities::square(
                            deg2rad(last_insnavgeod_.pitch_std_dev));
                        msg.orientation_covariance[8] = parsing_utilities::square(
                            deg2rad(last_insnavgeod_.heading_std_dev));
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
    } else
    {
        valid_orientation = false;
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

    return msg;
};

TwistWithCovarianceStampedMsg
io_comm_rx::RxMessage::TwistCallback(bool fromIns /* = false*/)
{
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
            Eigen::Matrix3d Cov_vel_n = Eigen::Matrix3d::Zero();
            if ((last_insnavgeod_.sb_list & 128) != 0)
            {
                // Linear velocity covariance
                if (validValue(last_insnavgeod_.ve_std_dev))
                    if (settings_->use_ros_axis_orientation)
                        Cov_vel_n(0, 0) =
                            parsing_utilities::square(last_insnavgeod_.ve_std_dev);
                    else
                        Cov_vel_n(1, 1) =
                            parsing_utilities::square(last_insnavgeod_.ve_std_dev);
                else
                    Cov_vel_n(0, 0) = -1.0;
                if (validValue(last_insnavgeod_.vn_std_dev))
                    if (settings_->use_ros_axis_orientation)
                        Cov_vel_n(1, 1) =
                            parsing_utilities::square(last_insnavgeod_.vn_std_dev);
                    else
                        Cov_vel_n(0, 0) =
                            parsing_utilities::square(last_insnavgeod_.vn_std_dev);
                else
                    Cov_vel_n(1, 1) = -1.0;
                if (validValue(last_insnavgeod_.vu_std_dev))
                    Cov_vel_n(2, 2) =
                        parsing_utilities::square(last_insnavgeod_.vu_std_dev);
                else
                    Cov_vel_n(2, 2) = -1.0;

                if (validValue(last_insnavgeod_.ve_vn_cov))
                    Cov_vel_n(0, 1) = Cov_vel_n(1, 0) = last_insnavgeod_.ve_vn_cov;
                if (settings_->use_ros_axis_orientation)
                {
                    if (validValue(last_insnavgeod_.ve_vu_cov))
                        Cov_vel_n(0, 2) = Cov_vel_n(2, 0) =
                            last_insnavgeod_.ve_vu_cov;
                    if (validValue(last_insnavgeod_.vn_vu_cov))
                        Cov_vel_n(2, 1) = Cov_vel_n(1, 2) =
                            last_insnavgeod_.vn_vu_cov;
                } else
                {
                    if (validValue(last_insnavgeod_.vn_vu_cov))
                        Cov_vel_n(0, 2) = Cov_vel_n(2, 0) =
                            -last_insnavgeod_.vn_vu_cov;
                    if (validValue(last_insnavgeod_.ve_vu_cov))
                        Cov_vel_n(2, 1) = Cov_vel_n(1, 2) =
                            -last_insnavgeod_.ve_vu_cov;
                }
            } else
            {
                Cov_vel_n(0, 0) = -1.0;
                Cov_vel_n(1, 1) = -1.0;
                Cov_vel_n(2, 2) = -1.0;
            }

            msg.twist.covariance[0] = Cov_vel_n(0, 0);
            msg.twist.covariance[1] = Cov_vel_n(0, 1);
            msg.twist.covariance[2] = Cov_vel_n(0, 2);
            msg.twist.covariance[6] = Cov_vel_n(1, 0);
            msg.twist.covariance[7] = Cov_vel_n(1, 1);
            msg.twist.covariance[8] = Cov_vel_n(1, 2);
            msg.twist.covariance[12] = Cov_vel_n(2, 0);
            msg.twist.covariance[13] = Cov_vel_n(2, 1);
            msg.twist.covariance[14] = Cov_vel_n(2, 2);
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

    } else
    {
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
            Eigen::Matrix3d Cov_vel_n = Eigen::Matrix3d::Zero();
            // Linear velocity covariance in navigation frame
            if (validValue(last_velcovgeodetic_.cov_veve))
                if (settings_->use_ros_axis_orientation)
                    Cov_vel_n(0, 0) = last_velcovgeodetic_.cov_veve;
                else
                    Cov_vel_n(1, 1) = last_velcovgeodetic_.cov_veve;
            else
                Cov_vel_n(0, 0) = -1.0;
            if (validValue(last_velcovgeodetic_.cov_vnvn))
                if (settings_->use_ros_axis_orientation)
                    Cov_vel_n(1, 1) = last_velcovgeodetic_.cov_vnvn;
                else
                    Cov_vel_n(0, 0) = last_velcovgeodetic_.cov_vnvn;
            else
                Cov_vel_n(1, 1) = -1.0;
            if (validValue(last_velcovgeodetic_.cov_vuvu))
                Cov_vel_n(2, 2) = last_velcovgeodetic_.cov_vuvu;
            else
                Cov_vel_n(2, 2) = -1.0;

            Cov_vel_n(0, 1) = Cov_vel_n(1, 0) = last_velcovgeodetic_.cov_vnve;
            if (settings_->use_ros_axis_orientation)
            {
                Cov_vel_n(0, 2) = Cov_vel_n(2, 0) = last_velcovgeodetic_.cov_vevu;
                Cov_vel_n(2, 1) = Cov_vel_n(1, 2) = last_velcovgeodetic_.cov_vnvu;
            } else
            {
                Cov_vel_n(0, 2) = Cov_vel_n(2, 0) = -last_velcovgeodetic_.cov_vnvu;
                Cov_vel_n(2, 1) = Cov_vel_n(1, 2) = -last_velcovgeodetic_.cov_vevu;
            }

            msg.twist.covariance[0] = Cov_vel_n(0, 0);
            msg.twist.covariance[1] = Cov_vel_n(0, 1);
            msg.twist.covariance[2] = Cov_vel_n(0, 2);
            msg.twist.covariance[6] = Cov_vel_n(1, 0);
            msg.twist.covariance[7] = Cov_vel_n(1, 1);
            msg.twist.covariance[8] = Cov_vel_n(1, 2);
            msg.twist.covariance[12] = Cov_vel_n(2, 0);
            msg.twist.covariance[13] = Cov_vel_n(2, 1);
            msg.twist.covariance[14] = Cov_vel_n(2, 2);
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

    msg.header.frame_id = "navigation";

    return msg;
};

/**
 * Localization in UTM coordinates. Yaw angle is converted from true north to grid
 * north. Linear velocity of twist in body frame as per msg definition. Angular
 * velocity not available, thus according autocovariances are set to -1.0.
 */
LocalizationUtmMsg io_comm_rx::RxMessage::LocalizationUtmCallback()
{
    LocalizationUtmMsg msg;

    int zone;
    std::string zonestring;
    bool northernHemisphere;
    double easting;
    double northing;
    double gamma = 0.0;
    if (fixedUtmZone_)
    {
        double k;
        GeographicLib::UTMUPS::DecodeZone(*fixedUtmZone_, zone, northernHemisphere);
        GeographicLib::UTMUPS::Forward(
            rad2deg(last_insnavgeod_.latitude), rad2deg(last_insnavgeod_.longitude),
            zone, northernHemisphere, easting, northing, gamma, k, zone);
        zonestring = *fixedUtmZone_;
    } else
    {
        double k;
        GeographicLib::UTMUPS::Forward(
            rad2deg(last_insnavgeod_.latitude), rad2deg(last_insnavgeod_.longitude),
            zone, northernHemisphere, easting, northing, gamma, k);
        zonestring = GeographicLib::UTMUPS::EncodeZone(zone, northernHemisphere);
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
    if (settings_->ins_use_poi)
        msg.child_frame_id = settings_->poi_frame_id; // TODO param
    else
        msg.child_frame_id = settings_->frame_id;

    if ((last_insnavgeod_.sb_list & 1) != 0)
    {
        // Position autocovariance
        msg.pose.covariance[0] =
            parsing_utilities::square(last_insnavgeod_.longitude_std_dev);
        msg.pose.covariance[7] =
            parsing_utilities::square(last_insnavgeod_.latitude_std_dev);
        msg.pose.covariance[14] =
            parsing_utilities::square(last_insnavgeod_.height_std_dev);
    } else
    {
        msg.pose.covariance[0] = -1.0;
        msg.pose.covariance[7] = -1.0;
        msg.pose.covariance[14] = -1.0;
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
    // gamma for conversion from true north to grid north
    if (settings_->use_ros_axis_orientation)
        yaw -= deg2rad(gamma);
    else
        yaw += deg2rad(gamma);

    Eigen::Matrix3d R_n_b = parsing_utilities::rpyToRot(roll, pitch, yaw).inverse();
    if ((last_insnavgeod_.sb_list & 2) != 0)
    {
        // Attitude
        msg.pose.pose.orientation =
            parsing_utilities::convertEulerToQuaternion(yaw, pitch, roll);
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
                parsing_utilities::square(deg2rad(last_insnavgeod_.roll_std_dev));
        else
            msg.pose.covariance[21] = -1.0;
        if (validValue(last_insnavgeod_.pitch_std_dev))
            msg.pose.covariance[28] =
                parsing_utilities::square(deg2rad(last_insnavgeod_.pitch_std_dev));
        else
            msg.pose.covariance[28] = -1.0;
        if (validValue(last_insnavgeod_.heading_std_dev))
            msg.pose.covariance[35] =
                parsing_utilities::square(deg2rad(last_insnavgeod_.heading_std_dev));
        else
            msg.pose.covariance[35] = -1.0;
    } else
    {
        msg.pose.covariance[21] = -1.0;
        msg.pose.covariance[28] = -1.0;
        msg.pose.covariance[35] = -1.0;
    }
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
        Eigen::Vector3d vel_enu;
        if (settings_->use_ros_axis_orientation)
        {
            // (ENU)
            vel_enu << ve, vn, vu;
        } else
        {
            // (NED)
            vel_enu << vn, ve, -vu;
        }
        // Linear velocity, rotate to body coordinates
        Eigen::Vector3d vel_body = R_n_b * vel_enu;
        msg.twist.twist.linear.x = vel_body(0);
        msg.twist.twist.linear.y = vel_body(1);
        msg.twist.twist.linear.z = vel_body(2);
    } else
    {
        msg.twist.twist.linear.x = std::numeric_limits<double>::quiet_NaN();
        msg.twist.twist.linear.y = std::numeric_limits<double>::quiet_NaN();
        msg.twist.twist.linear.z = std::numeric_limits<double>::quiet_NaN();
    }
    Eigen::Matrix3d Cov_vel_n = Eigen::Matrix3d::Zero();
    if ((last_insnavgeod_.sb_list & 16) != 0)
    {
        // Linear velocity autocovariance
        if (validValue(last_insnavgeod_.ve_std_dev))
            if (settings_->use_ros_axis_orientation)
                Cov_vel_n(0, 0) =
                    parsing_utilities::square(last_insnavgeod_.ve_std_dev);
            else
                Cov_vel_n(0, 0) =
                    parsing_utilities::square(last_insnavgeod_.vn_std_dev);
        else
            Cov_vel_n(0, 0) = -1.0;
        if (validValue(last_insnavgeod_.vn_std_dev))
            if (settings_->use_ros_axis_orientation)
                Cov_vel_n(1, 1) =
                    parsing_utilities::square(last_insnavgeod_.vn_std_dev);
            else
                Cov_vel_n(1, 1) =
                    parsing_utilities::square(last_insnavgeod_.ve_std_dev);
        else
            Cov_vel_n(1, 1) = -1.0;
        if (validValue(last_insnavgeod_.vu_std_dev))
            Cov_vel_n(2, 2) = parsing_utilities::square(last_insnavgeod_.vu_std_dev);
        else
            Cov_vel_n(2, 2) = -1.0;
    } else
    {
        Cov_vel_n(0, 0) = -1.0;
        Cov_vel_n(1, 1) = -1.0;
        Cov_vel_n(2, 2) = -1.0;
    }
    if ((last_insnavgeod_.sb_list & 32) != 0)
    {
        // Position covariance
        msg.pose.covariance[1] = last_insnavgeod_.latitude_longitude_cov;
        msg.pose.covariance[6] = last_insnavgeod_.latitude_longitude_cov;

        if (settings_->use_ros_axis_orientation)
        {
            // (ENU)
            msg.pose.covariance[2] = last_insnavgeod_.longitude_height_cov;
            msg.pose.covariance[8] = last_insnavgeod_.latitude_height_cov;
            msg.pose.covariance[12] = last_insnavgeod_.longitude_height_cov;
            msg.pose.covariance[13] = last_insnavgeod_.latitude_height_cov;
        } else
        {
            // (NED)
            msg.pose.covariance[2] = -last_insnavgeod_.latitude_height_cov;
            msg.pose.covariance[8] = -last_insnavgeod_.longitude_height_cov;
            msg.pose.covariance[12] = -last_insnavgeod_.latitude_height_cov;
            msg.pose.covariance[13] = -last_insnavgeod_.longitude_height_cov;
        }
    }
    if ((last_insnavgeod_.sb_list & 64) != 0)
    {
        // Attitude covariancae
        msg.pose.covariance[22] = deg2radSq(last_insnavgeod_.pitch_roll_cov);
        msg.pose.covariance[23] = deg2radSq(last_insnavgeod_.heading_roll_cov);
        msg.pose.covariance[27] = deg2radSq(last_insnavgeod_.pitch_roll_cov);

        msg.pose.covariance[29] = deg2radSq(last_insnavgeod_.heading_pitch_cov);
        msg.pose.covariance[33] = deg2radSq(last_insnavgeod_.heading_roll_cov);
        msg.pose.covariance[34] = deg2radSq(last_insnavgeod_.heading_pitch_cov);

        if (!settings_->use_ros_axis_orientation)
        {
            // (NED)
            msg.pose.covariance[33] *= -1.0;
            msg.pose.covariance[23] *= -1.0;
            msg.pose.covariance[22] *= -1.0;
            msg.pose.covariance[27] *= -1.0;
        }
    }
    if ((last_insnavgeod_.sb_list & 128) != 0)
    {
        Cov_vel_n(0, 1) = Cov_vel_n(1, 0) = last_insnavgeod_.ve_vn_cov;
        if (settings_->use_ros_axis_orientation)
        {
            Cov_vel_n(0, 2) = Cov_vel_n(2, 0) = last_insnavgeod_.ve_vu_cov;
            Cov_vel_n(2, 1) = Cov_vel_n(1, 2) = last_insnavgeod_.vn_vu_cov;
        } else
        {
            Cov_vel_n(0, 2) = Cov_vel_n(2, 0) = -last_insnavgeod_.vn_vu_cov;
            Cov_vel_n(2, 1) = Cov_vel_n(1, 2) = -last_insnavgeod_.ve_vu_cov;
        }
    }

    if (((last_insnavgeod_.sb_list & 16) != 0) &&
        ((last_insnavgeod_.sb_list & 2) != 0) &&
        ((last_insnavgeod_.sb_list & 8) != 0) &&
        validValue(last_insnavgeod_.ve_std_dev) &&
        validValue(last_insnavgeod_.vn_std_dev) &&
        validValue(last_insnavgeod_.vu_std_dev))
    {
        // Rotate covariance matrix to body coordinates
        Eigen::Matrix3d Cov_vel_body = R_n_b * Cov_vel_n * R_n_b.transpose();

        msg.twist.covariance[0] = Cov_vel_body(0, 0);
        msg.twist.covariance[1] = Cov_vel_body(0, 1);
        msg.twist.covariance[2] = Cov_vel_body(0, 2);
        msg.twist.covariance[6] = Cov_vel_body(1, 0);
        msg.twist.covariance[7] = Cov_vel_body(1, 1);
        msg.twist.covariance[8] = Cov_vel_body(1, 2);
        msg.twist.covariance[12] = Cov_vel_body(2, 0);
        msg.twist.covariance[13] = Cov_vel_body(2, 1);
        msg.twist.covariance[14] = Cov_vel_body(2, 2);
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
    return msg;
};

/**
 * The position_covariance array is populated in row-major order, where the basis of
 * the corresponding matrix is ENU (so Cov_lonlon is in location 11 of the matrix).
 * The B2b signal type of BeiDou is not checked for usage, since the SignalInfo field
 * of the PVTGeodetic block does not disclose it. For that, one would need to go to
 * the ObsInfo field of the MeasEpochChannelType1 sub-block.
 */
NavSatFixMsg io_comm_rx::RxMessage::NavSatFixCallback()
{
    NavSatFixMsg msg;
    uint16_t mask = 15; // We extract the first four bits using this mask.
    if (settings_->septentrio_receiver_type == "gnss")
    {
        uint16_t type_of_pvt = ((uint16_t)(last_pvtgeodetic_.mode)) & mask;
        switch (type_of_pvt_map[type_of_pvt])
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
                LogLevel::DEBUG,
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
        return msg;
    }

    if (settings_->septentrio_receiver_type == "ins")
    {
        NavSatFixMsg msg;
        uint16_t type_of_pvt = ((uint16_t)(last_insnavgeod_.gnss_mode)) & mask;
        switch (type_of_pvt_map[type_of_pvt])
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
                LogLevel::DEBUG,
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
                parsing_utilities::square(last_insnavgeod_.longitude_std_dev);
            msg.position_covariance[4] =
                parsing_utilities::square(last_insnavgeod_.latitude_std_dev);
            msg.position_covariance[8] =
                parsing_utilities::square(last_insnavgeod_.height_std_dev);
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
        msg.position_covariance_type = NavSatFixMsg::COVARIANCE_TYPE_DIAGONAL_KNOWN;
    }
    return msg;
};

/**
 * Note that the field "dip" denotes the local magnetic inclination in degrees
 * (positive when the magnetic field points downwards (into the Earth)).
 * This quantity cannot be calculated by most Septentrio
 * receivers. We assume that for the ROS field "err_time", we are requested to
 * provide the 2 sigma uncertainty on the clock bias estimate in square meters, not
 * the clock drift estimate (latter would be
 * "2*std::sqrt(last_velcovgeodetic_.Cov_DtDt)").
 * The "err_track" entry is calculated via the Gaussian error propagation formula
 * from the eastward and the northward velocities. For the formula's usage we have to
 * assume that the eastward and the northward velocities are independent variables.
 * Note that elevations and azimuths of visible satellites are taken from the
 * ChannelStatus block, which provides 1 degree precision, while the SatVisibility
 * block could provide hundredths of degrees precision. Change if imperative for your
 * application... Definition of "visible satellite" adopted here: We define a visible
 * satellite as being !up to! "in sync" mode with the receiver, which corresponds to
 * last_measepoch_.N (signal-to-noise ratios are thereby available for these), though
 * not last_channelstatus_.N, which also includes those "in search". In case certain
 * values appear unphysical, please consult the firmware, since those most likely
 * refer to Do-Not-Use values.
 */
GPSFixMsg io_comm_rx::RxMessage::GPSFixCallback()
{
    GPSFixMsg msg;
    msg.status.satellites_used = static_cast<uint16_t>(last_pvtgeodetic_.nr_sv);

    // MeasEpoch Processing
    std::vector<int32_t> cno_tracked;
    std::vector<int32_t> svid_in_sync;
    {
        cno_tracked.reserve(last_measepoch_.type1.size());
        svid_in_sync.reserve(last_measepoch_.type1.size());
        for (const auto& measepoch_channel_type1 : last_measepoch_.type1)
        {
            // Define MeasEpochChannelType1 struct for the corresponding sub-block
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
            for (int32_t j = 0; j < static_cast<int32_t>(svid_in_sync.size()); ++j)
            {
                if (svid_in_sync[j] == static_cast<int32_t>(channel_sat_info.sv_id))
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
                        (channel_state_info.pvt_status & pvt_status_mask) >> k - 1;
                    if (pvt_status_value == 2)
                    {
                        pvt_status = true;
                    }
                    if (k > 1)
                    {
                        pvt_status_mask = pvt_status_mask - std::pow(2, k) -
                                          std::pow(2, k - 1) + std::pow(2, k - 2) +
                                          std::pow(2, k - 3);
                    }
                }
                if (pvt_status)
                {
                    svid_pvt.push_back(static_cast<int32_t>(channel_sat_info.sv_id));
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

        // PVT Status Analysis
        uint16_t status_mask = 15; // We extract the first four bits using this mask.
        uint16_t type_of_pvt = ((uint16_t)(last_pvtgeodetic_.mode)) & status_mask;
        switch (type_of_pvt_map[type_of_pvt])
        {
        case evNoPVT:
        {
            msg.status.status = GPSStatusMsg::STATUS_NO_FIX;
            break;
        }
        case evStandAlone:
        case evFixed:
        {
            msg.status.status = GPSStatusMsg::STATUS_FIX;
            break;
        }
        case evDGPS:
        case evRTKFixed:
        case evRTKFloat:
        case evMovingBaseRTKFixed:
        case evMovingBaseRTKFloat:
        case evPPP:
        {
            msg.status.status = GPSStatusMsg::STATUS_GBAS_FIX;
            break;
        }
        case evSBAS:
        {
            uint16_t reference_id = last_pvtgeodetic_.reference_id;
            // Here come the PRNs of the 4 WAAS satellites..
            if (reference_id == 131 || reference_id == 133 || reference_id == 135 ||
                reference_id == 135)
            {
                msg.status.status = GPSStatusMsg::STATUS_WAAS_FIX;
            } else
            {
                msg.status.status = GPSStatusMsg::STATUS_SBAS_FIX;
            }
            break;
        }
        default:
        {
            node_->log(
                LogLevel::DEBUG,
                "PVTGeodetic's Mode field contains an invalid type of PVT solution.");
            break;
        }
        }
        // Doppler is not used when calculating the velocities of, say, mosaic-x5,
        // hence:
        msg.status.motion_source = GPSStatusMsg::SOURCE_POINTS;
        // Doppler is not used when calculating the orientation of, say, mosaic-x5,
        // hence:
        msg.status.orientation_source = GPSStatusMsg::SOURCE_POINTS;
        msg.status.position_source = GPSStatusMsg::SOURCE_GPS;
        msg.latitude = rad2deg(last_pvtgeodetic_.latitude);
        msg.longitude = rad2deg(last_pvtgeodetic_.longitude);
        msg.altitude = last_pvtgeodetic_.height;
        // Note that cog is of type float32 while track is of type float64.
        msg.track = last_pvtgeodetic_.cog;
        msg.speed = std::sqrt(parsing_utilities::square(last_pvtgeodetic_.vn) +
                              parsing_utilities::square(last_pvtgeodetic_.ve));
        msg.climb = last_pvtgeodetic_.vu;
        msg.pitch = last_atteuler_.pitch;
        msg.roll = last_atteuler_.roll;
        if (last_dop_.pdop == 0.0 || last_dop_.tdop == 0.0)
        {
            msg.gdop = -1.0;
        } else
        {
            msg.gdop = std::sqrt(parsing_utilities::square(last_dop_.pdop) +
                                 parsing_utilities::square(last_dop_.tdop));
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
        msg.time = static_cast<double>(last_pvtgeodetic_.block_header.tow) / 1000 +
                   static_cast<double>(last_pvtgeodetic_.block_header.wnc * 7 * 24 *
                                       60 * 60);
        msg.err =
            2 * (std::sqrt(static_cast<double>(last_poscovgeodetic_.cov_latlat) +
                           static_cast<double>(last_poscovgeodetic_.cov_lonlon) +
                           static_cast<double>(last_poscovgeodetic_.cov_hgthgt)));
        msg.err_horz =
            2 * (std::sqrt(static_cast<double>(last_poscovgeodetic_.cov_latlat) +
                           static_cast<double>(last_poscovgeodetic_.cov_lonlon)));
        msg.err_vert =
            2 * std::sqrt(static_cast<double>(last_poscovgeodetic_.cov_hgthgt));
        msg.err_track =
            2 *
            (std::sqrt(parsing_utilities::square(
                           1.0 / (last_pvtgeodetic_.vn +
                                  parsing_utilities::square(last_pvtgeodetic_.ve) /
                                      last_pvtgeodetic_.vn)) *
                           last_poscovgeodetic_.cov_lonlon +
                       parsing_utilities::square(
                           (last_pvtgeodetic_.ve) /
                           (parsing_utilities::square(last_pvtgeodetic_.vn) +
                            parsing_utilities::square(last_pvtgeodetic_.ve))) *
                           last_poscovgeodetic_.cov_latlat));
        msg.err_speed =
            2 * (std::sqrt(static_cast<double>(last_velcovgeodetic_.cov_vnvn) +
                           static_cast<double>(last_velcovgeodetic_.cov_veve)));
        msg.err_climb =
            2 * std::sqrt(static_cast<double>(last_velcovgeodetic_.cov_vuvu));
        msg.err_pitch =
            2 * std::sqrt(static_cast<double>(last_attcoveuler_.cov_pitchpitch));
        msg.err_roll =
            2 * std::sqrt(static_cast<double>(last_attcoveuler_.cov_rollroll));
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
    }

    if (settings_->septentrio_receiver_type == "ins")
    {
        // PVT Status Analysis
        uint16_t status_mask = 15; // We extract the first four bits using this mask.
        uint16_t type_of_pvt =
            ((uint16_t)(last_insnavgeod_.gnss_mode)) & status_mask;
        switch (type_of_pvt_map[type_of_pvt])
        {
        case evNoPVT:
        {
            msg.status.status = GPSStatusMsg::STATUS_NO_FIX;
            break;
        }
        case evStandAlone:
        case evFixed:
        {
            msg.status.status = GPSStatusMsg::STATUS_FIX;
            break;
        }
        case evDGPS:
        case evRTKFixed:
        case evRTKFloat:
        case evMovingBaseRTKFixed:
        case evMovingBaseRTKFloat:
        case evPPP:
        {
            msg.status.status = GPSStatusMsg::STATUS_GBAS_FIX;
            break;
        }
        case evSBAS:
        default:
        {
            node_->log(
                LogLevel::DEBUG,
                "INSNavGeod's Mode field contains an invalid type of PVT solution.");
            break;
        }
        }
        // Doppler is not used when calculating the velocities of, say, mosaic-x5,
        // hence:
        msg.status.motion_source = GPSStatusMsg::SOURCE_POINTS;
        // Doppler is not used when calculating the orientation of, say, mosaic-x5,
        // hence:
        msg.status.orientation_source = GPSStatusMsg::SOURCE_POINTS;
        msg.status.position_source = GPSStatusMsg::SOURCE_GPS;
        msg.latitude = rad2deg(last_insnavgeod_.latitude);
        msg.longitude = rad2deg(last_insnavgeod_.longitude);
        msg.altitude = last_insnavgeod_.height;
        // Note that cog is of type float32 while track is of type float64.
        if ((last_insnavgeod_.sb_list & 2) != 0)
        {
            msg.track = last_insnavgeod_.heading;
            msg.pitch = last_insnavgeod_.pitch;
            msg.roll = last_insnavgeod_.roll;
        }
        if ((last_insnavgeod_.sb_list & 8) != 0)
        {
            msg.speed = std::sqrt(parsing_utilities::square(last_insnavgeod_.vn) +
                                  parsing_utilities::square(last_insnavgeod_.ve));

            msg.climb = last_insnavgeod_.vu;
        }
        if (last_dop_.pdop == 0.0 || last_dop_.tdop == 0.0)
        {
            msg.gdop = -1.0;
        } else
        {
            msg.gdop = std::sqrt(parsing_utilities::square(last_dop_.pdop) +
                                 parsing_utilities::square(last_dop_.tdop));
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
        msg.time = static_cast<double>(last_insnavgeod_.block_header.tow) / 1000 +
                   static_cast<double>(last_insnavgeod_.block_header.wnc * 7 * 24 *
                                       60 * 60);
        if ((last_insnavgeod_.sb_list & 1) != 0)
        {
            msg.err =
                2 *
                (std::sqrt(
                    parsing_utilities::square(last_insnavgeod_.latitude_std_dev) +
                    parsing_utilities::square(last_insnavgeod_.longitude_std_dev) +
                    parsing_utilities::square(last_insnavgeod_.height_std_dev)));
            msg.err_horz =
                2 *
                (std::sqrt(
                    parsing_utilities::square(last_insnavgeod_.latitude_std_dev) +
                    parsing_utilities::square(last_insnavgeod_.longitude_std_dev)));
            msg.err_vert = 2 * (std::sqrt(parsing_utilities::square(
                                   last_insnavgeod_.height_std_dev)));
        }
        if (((last_insnavgeod_.sb_list & 8) != 0) ||
            ((last_insnavgeod_.sb_list & 1) != 0))
        {
            msg.err_track =
                2 * (std::sqrt(
                        parsing_utilities::square(
                            1.0 / (last_insnavgeod_.vn +
                                   parsing_utilities::square(last_insnavgeod_.ve) /
                                       last_insnavgeod_.vn)) *
                            parsing_utilities::square(
                                last_insnavgeod_.longitude_std_dev) +
                        parsing_utilities::square(
                            (last_insnavgeod_.ve) /
                            (parsing_utilities::square(last_insnavgeod_.vn) +
                             parsing_utilities::square(last_insnavgeod_.ve))) *
                            parsing_utilities::square(
                                last_insnavgeod_.latitude_std_dev)));
        }
        if ((last_insnavgeod_.sb_list & 8) != 0)
        {
            msg.err_speed =
                2 * (std::sqrt(parsing_utilities::square(last_insnavgeod_.vn) +
                               parsing_utilities::square(last_insnavgeod_.ve)));
            msg.err_climb =
                2 * std::sqrt(parsing_utilities::square(last_insnavgeod_.vn));
        }
        if ((last_insnavgeod_.sb_list & 2) != 0)
        {
            msg.err_pitch =
                2 * std::sqrt(parsing_utilities::square(last_insnavgeod_.pitch));
        }
        if ((last_insnavgeod_.sb_list & 2) != 0)
        {
            msg.err_pitch =
                2 * std::sqrt(parsing_utilities::square(last_insnavgeod_.roll));
        }
        if ((last_insnavgeod_.sb_list & 1) != 0)
        {
            msg.position_covariance[0] =
                parsing_utilities::square(last_insnavgeod_.longitude_std_dev);
            msg.position_covariance[4] =
                parsing_utilities::square(last_insnavgeod_.latitude_std_dev);
            msg.position_covariance[8] =
                parsing_utilities::square(last_insnavgeod_.height_std_dev);
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
        msg.position_covariance_type = NavSatFixMsg::COVARIANCE_TYPE_DIAGONAL_KNOWN;
    }
    return msg;
};

Timestamp io_comm_rx::RxMessage::timestampSBF(const uint8_t* data,
                                              bool use_gnss_time)
{
    uint32_t tow = parsing_utilities::getTow(data);
    uint16_t wnc = parsing_utilities::getWnc(data);

    return timestampSBF(tow, wnc, use_gnss_time);
}

/// If the current time shall be employed, it is calculated via the time(NULL)
/// function found in the \<ctime\> library At the time of writing the code (2020),
/// the GPS time was ahead of UTC time by 18 (leap) seconds. Adapt the
/// settings_->leap_seconds ROSaic parameter accordingly as soon as the next leap
/// second is inserted into the UTC time.
Timestamp io_comm_rx::RxMessage::timestampSBF(uint32_t tow, uint16_t wnc,
                                              bool use_gnss_time)
{
    Timestamp time_obj;
    if (use_gnss_time)
    {
        // conversion from GPS time of week and week number to UTC taking leap
        // seconds into account
        static uint64_t secToNSec = 1000000000;
        static uint64_t mSec2NSec = 1000000;
        static uint64_t nsOfGpsStart =
            315964800 *
            secToNSec; // GPS week counter starts at 1980-01-06 which is 315964800
                       // seconds since Unix epoch (1970-01-01 UTC)
        static uint64_t nsecPerWeek = 7 * 24 * 60 * 60 * secToNSec;

        time_obj = nsOfGpsStart + tow * mSec2NSec + wnc * nsecPerWeek;

        if (current_leap_seconds_ != -128)
            time_obj -= current_leap_seconds_ * secToNSec;
    } else
    {
        time_obj = recvTimestamp_;
    }
    return time_obj;
}

bool io_comm_rx::RxMessage::found()
{
    if (found_)
        return true;

    // Verify header bytes
    if (!this->isSBF() && !this->isNMEA() && !this->isResponse() &&
        !(g_read_cd && this->isConnectionDescriptor()))
    {
        return false;
    }

    found_ = true;
    return true;
}

const uint8_t* io_comm_rx::RxMessage::search()
{
    if (found_)
    {
        next();
    }
    // Search for message or a response header
    for (; count_ > 0; --count_, ++data_)
    {
        if (this->isSBF() || this->isNMEA() || this->isResponse() ||
            (g_read_cd && this->isConnectionDescriptor()))
        {
            break;
        }
    }
    found_ = true;
    return data_;
}

std::size_t io_comm_rx::RxMessage::messageSize()
{
    uint16_t pos = 0;
    message_size_ = 0;
    std::size_t count_copy = count_;
    if (this->isResponse())
    {
        do
        {
            ++message_size_;
            ++pos;
            --count_copy;
            if (count_copy == 0)
                break;
        } while (!((data_[pos] == CARRIAGE_RETURN && data_[pos + 1] == LINE_FEED)) ||
                 (data_[pos] == CARRIAGE_RETURN && data_[pos + 1] == LINE_FEED &&
                  data_[pos + 2] == 0x20 && data_[pos + 3] == 0x20 &&
                  data_[pos + 4] == 0x4E) ||
                 (data_[pos] == CARRIAGE_RETURN && data_[pos + 1] == LINE_FEED &&
                  data_[pos + 2] == 0x20 && data_[pos + 3] == 0x20 &&
                  data_[pos + 4] == 0x53) ||
                 (data_[pos] == CARRIAGE_RETURN && data_[pos + 1] == LINE_FEED &&
                  data_[pos + 2] == 0x20 && data_[pos + 3] == 0x20 &&
                  data_[pos + 4] == 0x52));
    } else
    {
        do
        {
            ++message_size_;
            ++pos;
            --count_copy;
            if (count_copy == 0)
                break;
        } while (!((data_[pos] == CARRIAGE_RETURN && data_[pos + 1] == LINE_FEED) ||
                   data_[pos] == CARRIAGE_RETURN || data_[pos] == LINE_FEED));
    }
    return message_size_;
}

bool io_comm_rx::RxMessage::isMessage(const uint16_t id)
{
    if (this->isSBF())
    {
        return (parsing_utilities::getId(data_) == static_cast<const uint16_t>(id));
    } else
    {
        return false;
    }
}

bool io_comm_rx::RxMessage::isMessage(std::string id)
{
    if (this->isNMEA())
    {
        boost::char_separator<char> sep(",");
        typedef boost::tokenizer<boost::char_separator<char>> tokenizer;
        std::size_t nmea_size = this->messageSize();
        std::string block_in_string(reinterpret_cast<const char*>(data_), nmea_size);
        tokenizer tokens(block_in_string, sep);
        if (*tokens.begin() == id)
        {
            return true;
        } else
        {
            return false;
        }
    } else
    {
        return false;
    }
}

bool io_comm_rx::RxMessage::isSBF()
{
    if (count_ >= 2)
    {
        if (data_[0] == SBF_SYNC_BYTE_1 && data_[1] == SBF_SYNC_BYTE_2)
        {
            return true;
        } else
        {
            return false;
        }
    } else
    {
        return false;
    }
}

bool io_comm_rx::RxMessage::isNMEA()
{
    if (count_ >= 2)
    {
        if ((data_[0] == NMEA_SYNC_BYTE_1 && data_[1] == NMEA_SYNC_BYTE_2_1) ||
            (data_[0] == NMEA_SYNC_BYTE_1 && data_[1] == NMEA_SYNC_BYTE_2_2))
        {
            return true;
        } else
        {
            return false;
        }
    } else
    {
        return false;
    }
}

bool io_comm_rx::RxMessage::isResponse()
{
    if (count_ >= 2)
    {
        if (data_[0] == RESPONSE_SYNC_BYTE_1 && data_[1] == RESPONSE_SYNC_BYTE_2)
        {
            return true;
        } else
        {
            return false;
        }
    } else
    {
        return false;
    }
}

bool io_comm_rx::RxMessage::isConnectionDescriptor()
{
    if (count_ >= 2)
    {
        if (data_[0] == CONNECTION_DESCRIPTOR_BYTE_1 &&
            data_[1] == CONNECTION_DESCRIPTOR_BYTE_2)
        {
            return true;
        } else
        {
            return false;
        }
    } else
    {
        return false;
    }
}

bool io_comm_rx::RxMessage::isErrorMessage()
{
    if (count_ >= 3)
    {
        if (data_[0] == RESPONSE_SYNC_BYTE_1 && data_[1] == RESPONSE_SYNC_BYTE_2 &&
            data_[2] == RESPONSE_SYNC_BYTE_3)
        {
            return true;
        } else
        {
            return false;
        }
    } else
    {
        return false;
    }
}

std::string io_comm_rx::RxMessage::messageID()
{
    if (this->isSBF())
    {
        std::stringstream ss;
        ss << parsing_utilities::getId(data_);
        return ss.str();
    }
    if (this->isNMEA())
    {
        boost::char_separator<char> sep(",");
        typedef boost::tokenizer<boost::char_separator<char>> tokenizer;
        std::size_t nmea_size = this->messageSize();
        std::string block_in_string(reinterpret_cast<const char*>(data_), nmea_size);
        tokenizer tokens(block_in_string, sep);
        return *tokens.begin();
    }
    return std::string(); // less CPU work than return "";
}

const uint8_t* io_comm_rx::RxMessage::getPosBuffer() { return data_; }

const uint8_t* io_comm_rx::RxMessage::getEndBuffer() { return data_ + count_; }

uint16_t io_comm_rx::RxMessage::getBlockLength()
{
    if (this->isSBF())
    {
        uint16_t block_length;
        // Note that static_cast<uint16_t>(data_[6]) would just take the one byte
        // "data_[6]" and cast it as requested, !neglecting! the byte "data_[7]".
        block_length = parsing_utilities::parseUInt16(data_ + 6);
        return block_length;
    } else
    {
        return 0;
    }
}

/**
 * This method won't make data_ jump to the next message if the current one is an
 * NMEA message or a command reply. In that case, search() will check the bytes one
 * by one for the new message's sync bytes ($P, $G or $R).
 */
void io_comm_rx::RxMessage::next()
{
    std::size_t jump_size;
    if (found())
    {
        if (this->isNMEA() || this->isResponse() ||
            (g_read_cd && this->isConnectionDescriptor()))
        {
            if (g_read_cd && this->isConnectionDescriptor() && g_cd_count == 2)
            {
                g_read_cd = false;
            }
            jump_size = static_cast<uint32_t>(1);
        }
        if (this->isSBF())
        {
            if (crc_check_)
            {
                jump_size = static_cast<std::size_t>(this->getBlockLength());
                // Some corrupted messages that survive the CRC check (this happened)
                // could tell ROSaic their size is 0, which would lead to an endless
                // while loop in the ReadCallback() method of the CallbackHandlers
                // class.
                if (jump_size == 0)
                    jump_size = static_cast<std::size_t>(1);
            } else
            {
                jump_size = static_cast<std::size_t>(1);
            }
        }
    }
    found_ = false;
    data_ += jump_size;
    count_ -= jump_size;
    // node_->log(LogLevel::DEBUG, "Jump about to happen with jump size %li and count
    // after jump being %li.", jump_size, count_);
    return; // For readability
}

/**
 * If GNSS time is used, Publishing is only done with valid leap seconds
 */
template <typename M>
void io_comm_rx::RxMessage::publish(const std::string& topic, const M& msg)
{
    // TODO: maybe publish only if wnc and tow is valid?
    if (!settings_->use_gnss_time ||
        (settings_->use_gnss_time && (current_leap_seconds_ != -128)))
    {
        node_->publishMessage<M>(topic, msg);
    } else
    {
        node_->log(
            LogLevel::DEBUG,
            "Not publishing message with GNSS time because no leap seconds are available yet.");
        if (settings_->read_from_sbf_log || settings_->read_from_pcap)
            node_->log(
                LogLevel::WARN,
                "No leap seconds were set and none were received from log yet.");
    }
}

/**
 * If GNSS time is used, Publishing is only done with valid leap seconds
 */
void io_comm_rx::RxMessage::publishTf(const LocalizationUtmMsg& msg)
{
    // TODO: maybe publish only if wnc and tow is valid?
    if (!settings_->use_gnss_time ||
        (settings_->use_gnss_time && (current_leap_seconds_ != -128) &&
         (current_leap_seconds_ != 0)))
    {
        node_->publishTf(msg);
    } else
    {
        node_->log(
            LogLevel::DEBUG,
            "Not publishing tf with GNSS time because no leap seconds are available yet.");
        if (settings_->read_from_sbf_log || settings_->read_from_pcap)
            node_->log(
                LogLevel::WARN,
                "No leap seconds were set and none were received from log yet.");
    }
}

/**
 * Note that putting the default in the definition's argument list instead of the
 * declaration's is an added extra that is not available for function templates,
 * hence no search = false here. Also note that the SBF block header part of the
 * SBF-echoing ROS messages have ID fields that only show the block number as found
 * in the firmware (e.g. 4007 for PVTGeodetic), without the revision number. NMEA
 * 0183 messages are at most 82 characters long in principle, but most Septentrio Rxs
 * by default increase precision on lat/lon s.t. the maximum allowed e.g. for GGA
 * seems to be 89 on a mosaic-x5. Luckily, when parsing we do not care since we just
 * search for \<LF\>\<CR\>.
 */
bool io_comm_rx::RxMessage::read(std::string message_key, bool search)
{
    if (search)
        this->search();
    if (!found())
        return false;
    if (this->isSBF())
    {
        // If the CRC check is unsuccessful, return false
        crc_check_ = isValid(data_);
        if (!crc_check_)
        {
            node_->log(
                LogLevel::DEBUG,
                "CRC Check returned False. Not a valid data block. Retrieving full SBF block.");
            return false;
        }
    }
    switch (rx_id_map[message_key])
    {
    case evPVTCartesian: // Position and velocity in XYZ
    { // The curly bracket here is crucial: Declarations inside a block remain
        // inside, and will die at
        // the end of the block. Otherwise variable overloading etc.
        PVTCartesianMsg msg;
        std::vector<uint8_t> dvec(data_,
                                  data_ + parsing_utilities::getLength(data_));
        if (!PVTCartesianParser(node_, dvec.begin(), dvec.end(), msg))
        {
            node_->log(LogLevel::ERROR,
                       "septentrio_gnss_driver: parse error in PVTCartesian");
            break;
        }
        msg.header.frame_id = settings_->frame_id;
        Timestamp time_obj = timestampSBF(data_, settings_->use_gnss_time);
        msg.header.stamp = timestampToRos(time_obj);
        // Wait as long as necessary (only when reading from SBF/PCAP file)
        if (settings_->read_from_sbf_log || settings_->read_from_pcap)
        {
            wait(time_obj);
        }
        publish<PVTCartesianMsg>("/pvtcartesian", msg);
        break;
    }
    case evPVTGeodetic: // Position and velocity in geodetic coordinate frame (ENU
                        // frame)
    {
        std::vector<uint8_t> dvec(data_,
                                  data_ + parsing_utilities::getLength(data_));
        if (!PVTGeodeticParser(node_, dvec.begin(), dvec.end(), last_pvtgeodetic_))
        {
            node_->log(LogLevel::ERROR,
                       "septentrio_gnss_driver: parse error in PVTGeodetic");
            break;
        }
        last_pvtgeodetic_.header.frame_id = settings_->frame_id;
        Timestamp time_obj = timestampSBF(data_, settings_->use_gnss_time);
        last_pvtgeodetic_.header.stamp = timestampToRos(time_obj);
        pvtgeodetic_has_arrived_gpsfix_ = true;
        pvtgeodetic_has_arrived_navsatfix_ = true;
        pvtgeodetic_has_arrived_pose_ = true;
        // Wait as long as necessary (only when reading from SBF/PCAP file)
        if (settings_->read_from_sbf_log || settings_->read_from_pcap)
        {
            wait(time_obj);
        }
        if (settings_->publish_pvtgeodetic)
            publish<PVTGeodeticMsg>("/pvtgeodetic", last_pvtgeodetic_);
        break;
    }
    case evBaseVectorCart:
    {
        BaseVectorCartMsg msg;
        std::vector<uint8_t> dvec(data_,
                                  data_ + parsing_utilities::getLength(data_));
        if (!BaseVectorCartParser(node_, dvec.begin(), dvec.end(), msg))
        {
            node_->log(LogLevel::ERROR,
                       "septentrio_gnss_driver: parse error in BaseVectorCart");
            break;
        }
        msg.header.frame_id = settings_->frame_id;
        Timestamp time_obj = timestampSBF(data_, settings_->use_gnss_time);
        msg.header.stamp = timestampToRos(time_obj);
        // Wait as long as necessary (only when reading from SBF/PCAP file)
        if (settings_->read_from_sbf_log || settings_->read_from_pcap)
        {
            wait(time_obj);
        }
        publish<BaseVectorCartMsg>("/basevectorcart", msg);
        break;
    }
    case evBaseVectorGeod:
    {
        BaseVectorGeodMsg msg;
        std::vector<uint8_t> dvec(data_,
                                  data_ + parsing_utilities::getLength(data_));
        if (!BaseVectorGeodParser(node_, dvec.begin(), dvec.end(), msg))
        {
            node_->log(LogLevel::ERROR,
                       "septentrio_gnss_driver: parse error in BaseVectorGeod");
            break;
        }
        msg.header.frame_id = settings_->frame_id;
        Timestamp time_obj = timestampSBF(data_, settings_->use_gnss_time);
        msg.header.stamp = timestampToRos(time_obj);
        // Wait as long as necessary (only when reading from SBF/PCAP file)
        if (settings_->read_from_sbf_log || settings_->read_from_pcap)
        {
            wait(time_obj);
        }
        publish<BaseVectorGeodMsg>("/basevectorgeod", msg);
        break;
    }
    case evPosCovCartesian:
    {
        PosCovCartesianMsg msg;
        std::vector<uint8_t> dvec(data_,
                                  data_ + parsing_utilities::getLength(data_));
        if (!PosCovCartesianParser(node_, dvec.begin(), dvec.end(), msg))
        {
            node_->log(LogLevel::ERROR,
                       "septentrio_gnss_driver: parse error in PosCovCartesian");
            break;
        }
        msg.header.frame_id = settings_->frame_id;
        Timestamp time_obj = timestampSBF(data_, settings_->use_gnss_time);
        msg.header.stamp = timestampToRos(time_obj);
        // Wait as long as necessary (only when reading from SBF/PCAP file)
        if (settings_->read_from_sbf_log || settings_->read_from_pcap)
        {
            wait(time_obj);
        }
        publish<PosCovCartesianMsg>("/poscovcartesian", msg);
        break;
    }
    case evPosCovGeodetic:
    {
        std::vector<uint8_t> dvec(data_,
                                  data_ + parsing_utilities::getLength(data_));
        if (!PosCovGeodeticParser(node_, dvec.begin(), dvec.end(),
                                  last_poscovgeodetic_))
        {
            poscovgeodetic_has_arrived_gpsfix_ = false;
            poscovgeodetic_has_arrived_navsatfix_ = false;
            poscovgeodetic_has_arrived_pose_ = false;
            node_->log(LogLevel::ERROR,
                       "septentrio_gnss_driver: parse error in PosCovGeodetic");
            break;
        }
        last_poscovgeodetic_.header.frame_id = settings_->frame_id;
        Timestamp time_obj = timestampSBF(data_, settings_->use_gnss_time);
        last_poscovgeodetic_.header.stamp = timestampToRos(time_obj);
        poscovgeodetic_has_arrived_gpsfix_ = true;
        poscovgeodetic_has_arrived_navsatfix_ = true;
        poscovgeodetic_has_arrived_pose_ = true;
        // Wait as long as necessary (only when reading from SBF/PCAP file)
        if (settings_->read_from_sbf_log || settings_->read_from_pcap)
        {
            wait(time_obj);
        }
        if (settings_->publish_poscovgeodetic)
            publish<PosCovGeodeticMsg>("/poscovgeodetic", last_poscovgeodetic_);
        break;
    }
    case evAttEuler:
    {
        std::vector<uint8_t> dvec(data_,
                                  data_ + parsing_utilities::getLength(data_));
        if (!AttEulerParser(node_, dvec.begin(), dvec.end(), last_atteuler_,
                            settings_->use_ros_axis_orientation))
        {
            atteuler_has_arrived_gpsfix_ = false;
            atteuler_has_arrived_pose_ = false;
            node_->log(LogLevel::ERROR,
                       "septentrio_gnss_driver: parse error in AttEuler");
            break;
        }
        last_atteuler_.header.frame_id = settings_->frame_id;
        Timestamp time_obj = timestampSBF(data_, settings_->use_gnss_time);
        last_atteuler_.header.stamp = timestampToRos(time_obj);
        atteuler_has_arrived_gpsfix_ = true;
        atteuler_has_arrived_pose_ = true;
        // Wait as long as necessary (only when reading from SBF/PCAP file)
        if (settings_->read_from_sbf_log || settings_->read_from_pcap)
        {
            wait(time_obj);
        }
        if (settings_->publish_atteuler)
            publish<AttEulerMsg>("/atteuler", last_atteuler_);
        break;
    }
    case evAttCovEuler:
    {
        std::vector<uint8_t> dvec(data_,
                                  data_ + parsing_utilities::getLength(data_));
        if (!AttCovEulerParser(node_, dvec.begin(), dvec.end(), last_attcoveuler_,
                               settings_->use_ros_axis_orientation))
        {
            attcoveuler_has_arrived_gpsfix_ = false;
            attcoveuler_has_arrived_pose_ = false;
            node_->log(LogLevel::ERROR,
                       "septentrio_gnss_driver: parse error in AttCovEuler");
            break;
        }
        last_attcoveuler_.header.frame_id = settings_->frame_id;
        Timestamp time_obj = timestampSBF(data_, settings_->use_gnss_time);
        last_attcoveuler_.header.stamp = timestampToRos(time_obj);
        attcoveuler_has_arrived_gpsfix_ = true;
        attcoveuler_has_arrived_pose_ = true;
        // Wait as long as necessary (only when reading from SBF/PCAP file)
        if (settings_->read_from_sbf_log || settings_->read_from_pcap)
        {
            wait(time_obj);
        }
        if (settings_->publish_attcoveuler)
            publish<AttCovEulerMsg>("/attcoveuler", last_attcoveuler_);
        break;
    }
    case evINSNavCart: // Position, velocity and orientation in cartesian coordinate
                       // frame (ENU frame)
    {
        INSNavCartMsg msg;
        std::vector<uint8_t> dvec(data_,
                                  data_ + parsing_utilities::getLength(data_));
        if (!INSNavCartParser(node_, dvec.begin(), dvec.end(), msg,
                              settings_->use_ros_axis_orientation))
        {
            node_->log(LogLevel::ERROR,
                       "septentrio_gnss_driver: parse error in INSNavCart");
            break;
        }
        if (settings_->ins_use_poi)
        {
            msg.header.frame_id = settings_->poi_frame_id;
        } else
        {
            msg.header.frame_id = settings_->frame_id;
        }
        Timestamp time_obj = timestampSBF(data_, settings_->use_gnss_time);
        msg.header.stamp = timestampToRos(time_obj);
        // Wait as long as necessary (only when reading from SBF/PCAP file)
        if (settings_->read_from_sbf_log || settings_->read_from_pcap)
        {
            wait(time_obj);
        }
        publish<INSNavCartMsg>("/insnavcart", msg);
        break;
    }
    case evINSNavGeod: // Position, velocity and orientation in geodetic coordinate
                       // frame (ENU frame)
    {
        std::vector<uint8_t> dvec(data_,
                                  data_ + parsing_utilities::getLength(data_));
        if (!INSNavGeodParser(node_, dvec.begin(), dvec.end(), last_insnavgeod_,
                              settings_->use_ros_axis_orientation))
        {
            insnavgeod_has_arrived_gpsfix_ = false;
            insnavgeod_has_arrived_navsatfix_ = false;
            insnavgeod_has_arrived_pose_ = false;
            insnavgeod_has_arrived_localization_ = false;
            node_->log(LogLevel::ERROR,
                       "septentrio_gnss_driver: parse error in INSNavGeod");
            break;
        }
        if (settings_->ins_use_poi)
        {
            last_insnavgeod_.header.frame_id = settings_->poi_frame_id;
        } else
        {
            last_insnavgeod_.header.frame_id = settings_->frame_id;
        }
        Timestamp time_obj = timestampSBF(data_, settings_->use_gnss_time);
        last_insnavgeod_.header.stamp = timestampToRos(time_obj);
        insnavgeod_has_arrived_gpsfix_ = true;
        insnavgeod_has_arrived_navsatfix_ = true;
        insnavgeod_has_arrived_pose_ = true;
        insnavgeod_has_arrived_localization_ = true;
        // Wait as long as necessary (only when reading from SBF/PCAP file)
        if (settings_->read_from_sbf_log || settings_->read_from_pcap)
        {
            wait(time_obj);
        }
        if (settings_->publish_insnavgeod)
            publish<INSNavGeodMsg>("/insnavgeod", last_insnavgeod_);
        if (settings_->publish_twist)
        {
            TwistWithCovarianceStampedMsg twist = TwistCallback(true);
            publish<TwistWithCovarianceStampedMsg>("/twist_ins", twist);
        }
        break;
    }

    case evIMUSetup: // IMU orientation and lever arm
    {
        IMUSetupMsg msg;
        std::vector<uint8_t> dvec(data_,
                                  data_ + parsing_utilities::getLength(data_));
        if (!IMUSetupParser(node_, dvec.begin(), dvec.end(), msg,
                            settings_->use_ros_axis_orientation))
        {
            node_->log(LogLevel::ERROR,
                       "septentrio_gnss_driver: parse error in IMUSetup");
            break;
        }
        msg.header.frame_id = settings_->vehicle_frame_id;
        Timestamp time_obj = timestampSBF(data_, settings_->use_gnss_time);
        msg.header.stamp = timestampToRos(time_obj);
        // Wait as long as necessary (only when reading from SBF/PCAP file)
        if (settings_->read_from_sbf_log || settings_->read_from_pcap)
        {
            wait(time_obj);
        }
        publish<IMUSetupMsg>("/imusetup", msg);
        break;
    }

    case evVelSensorSetup: // Velocity sensor lever arm
    {
        VelSensorSetupMsg msg;
        std::vector<uint8_t> dvec(data_,
                                  data_ + parsing_utilities::getLength(data_));
        if (!VelSensorSetupParser(node_, dvec.begin(), dvec.end(), msg,
                                  settings_->use_ros_axis_orientation))
        {
            node_->log(LogLevel::ERROR,
                       "septentrio_gnss_driver: parse error in VelSensorSetup");
            break;
        }
        msg.header.frame_id = settings_->vehicle_frame_id;
        Timestamp time_obj = timestampSBF(data_, settings_->use_gnss_time);
        msg.header.stamp = timestampToRos(time_obj);
        // Wait as long as necessary (only when reading from SBF/PCAP file)
        if (settings_->read_from_sbf_log || settings_->read_from_pcap)
        {
            wait(time_obj);
        }
        publish<VelSensorSetupMsg>("/velsensorsetup", msg);
        break;
    }

    case evExtEventINSNavCart: // Position, velocity and orientation in cartesian
                               // coordinate frame (ENU frame)
    {
        INSNavCartMsg msg;
        std::vector<uint8_t> dvec(data_,
                                  data_ + parsing_utilities::getLength(data_));
        if (!INSNavCartParser(node_, dvec.begin(), dvec.end(), msg,
                              settings_->use_ros_axis_orientation))
        {
            node_->log(LogLevel::ERROR,
                       "septentrio_gnss_driver: parse error in ExtEventINSNavCart");
            break;
        }
        if (settings_->ins_use_poi)
        {
            msg.header.frame_id = settings_->poi_frame_id;
        } else
        {
            msg.header.frame_id = settings_->frame_id;
        }
        Timestamp time_obj = timestampSBF(data_, settings_->use_gnss_time);
        msg.header.stamp = timestampToRos(time_obj);
        // Wait as long as necessary (only when reading from SBF/PCAP file)
        if (settings_->read_from_sbf_log || settings_->read_from_pcap)
        {
            wait(time_obj);
        }
        publish<INSNavCartMsg>("/exteventinsnavcart", msg);
        break;
    }

    case evExtEventINSNavGeod:
    {
        INSNavGeodMsg msg;
        std::vector<uint8_t> dvec(data_,
                                  data_ + parsing_utilities::getLength(data_));
        if (!INSNavGeodParser(node_, dvec.begin(), dvec.end(), msg,
                              settings_->use_ros_axis_orientation))
        {
            node_->log(LogLevel::ERROR,
                       "septentrio_gnss_driver: parse error in ExtEventINSNavGeod");
            break;
        }
        if (settings_->ins_use_poi)
        {
            msg.header.frame_id = settings_->poi_frame_id;
        } else
        {
            msg.header.frame_id = settings_->frame_id;
        }
        Timestamp time_obj = timestampSBF(data_, settings_->use_gnss_time);
        msg.header.stamp = timestampToRos(time_obj);
        // Wait as long as necessary (only when reading from SBF/PCAP file)
        if (settings_->read_from_sbf_log || settings_->read_from_pcap)
        {
            wait(time_obj);
        }
        publish<INSNavGeodMsg>("/exteventinsnavgeod", msg);
        break;
    }

    case evExtSensorMeas:
    {
        std::vector<uint8_t> dvec(data_,
                                  data_ + parsing_utilities::getLength(data_));
        bool hasImuMeas = false;
        if (!ExtSensorMeasParser(node_, dvec.begin(), dvec.end(), last_extsensmeas_,
                                 settings_->use_ros_axis_orientation, hasImuMeas))
        {
            node_->log(LogLevel::ERROR,
                       "septentrio_gnss_driver: parse error in ExtSensorMeas");
            break;
        }
        last_extsensmeas_.header.frame_id = settings_->imu_frame_id;
        Timestamp time_obj = timestampSBF(data_, settings_->use_gnss_time);
        last_extsensmeas_.header.stamp = timestampToRos(time_obj);
        // Wait as long as necessary (only when reading from SBF/PCAP file)
        if (settings_->read_from_sbf_log || settings_->read_from_pcap)
        {
            wait(time_obj);
        }
        if (settings_->publish_extsensormeas)
            publish<ExtSensorMeasMsg>("/extsensormeas", last_extsensmeas_);
        if (settings_->publish_imu && hasImuMeas)
        {
            ImuMsg msg;
            try
            {
                msg = ImuCallback();
            } catch (std::runtime_error& e)
            {
                node_->log(LogLevel::DEBUG, "ImuMsg: " + std::string(e.what()));
                break;
            }
            msg.header.frame_id = settings_->imu_frame_id;
            msg.header.stamp = last_extsensmeas_.header.stamp;
            publish<ImuMsg>("/imu", msg);
        }
        break;
    }

    case evGPST:
    {
        TimeReferenceMsg msg;
        Timestamp time_obj =
            timestampSBF(data_, true); // We need the GPS time, hence true
        msg.time_ref = timestampToRos(time_obj);
        msg.source = "GPST";
        // Wait as long as necessary (only when reading from SBF/PCAP file)
        if (settings_->read_from_sbf_log || settings_->read_from_pcap)
        {
            wait(time_obj);
        }
        publish<TimeReferenceMsg>("/gpst", msg);
        break;
    }
    case evGPGGA:
    {
        boost::char_separator<char> sep("\r");
        typedef boost::tokenizer<boost::char_separator<char>> tokenizer;
        std::size_t nmea_size = this->messageSize();
        std::string block_in_string(reinterpret_cast<const char*>(data_), nmea_size);
        tokenizer tokens(block_in_string, sep);

        std::string id = this->messageID();
        std::string one_message = *tokens.begin();
        // No kept delimiters, hence "". Also, we specify that empty tokens should
        // show up in the output when two delimiters are next to each other. Hence we
        // also append the checksum part of the GGA message to "body" below, though
        // it is not parsed.
        boost::char_separator<char> sep_2(",*", "", boost::keep_empty_tokens);
        tokenizer tokens_2(one_message, sep_2);
        std::vector<std::string> body;
        for (tokenizer::iterator tok_iter = tokens_2.begin();
             tok_iter != tokens_2.end(); ++tok_iter)
        {
            body.push_back(*tok_iter);
        }
        // Create NmeaSentence struct to pass to GpggaParser::parseASCII
        NMEASentence gga_message(id, body);
        GpggaMsg msg;
        Timestamp time_obj = node_->getTime();
        GpggaParser parser_obj;
        try
        {
            msg = parser_obj.parseASCII(gga_message, settings_->frame_id,
                                        settings_->use_gnss_time, time_obj);
        } catch (ParseException& e)
        {
            node_->log(LogLevel::DEBUG, "GpggaMsg: " + std::string(e.what()));
            break;
        }
        // Wait as long as necessary (only when reading from SBF/PCAP file)
        if (settings_->read_from_sbf_log || settings_->read_from_pcap)
        {
            Timestamp time_obj = timestampFromRos(msg.header.stamp);
            wait(time_obj);
        }
        publish<GpggaMsg>("/gpgga", msg);
        break;
    }
    case evGPRMC:
    {
        Timestamp time_obj = node_->getTime();

        boost::char_separator<char> sep("\r");
        typedef boost::tokenizer<boost::char_separator<char>> tokenizer;
        std::size_t nmea_size = this->messageSize();
        std::string block_in_string(reinterpret_cast<const char*>(data_), nmea_size);
        tokenizer tokens(block_in_string, sep);

        std::string id = this->messageID();
        std::string one_message = *tokens.begin();
        boost::char_separator<char> sep_2(",*", "", boost::keep_empty_tokens);
        tokenizer tokens_2(one_message, sep_2);
        std::vector<std::string> body;
        for (tokenizer::iterator tok_iter = tokens_2.begin();
             tok_iter != tokens_2.end(); ++tok_iter)
        {
            body.push_back(*tok_iter);
        }
        // Create NmeaSentence struct to pass to GprmcParser::parseASCII
        NMEASentence rmc_message(id, body);
        GprmcMsg msg;
        GprmcParser parser_obj;
        try
        {
            msg = parser_obj.parseASCII(rmc_message, settings_->frame_id,
                                        settings_->use_gnss_time, time_obj);
        } catch (ParseException& e)
        {
            node_->log(LogLevel::DEBUG, "GprmcMsg: " + std::string(e.what()));
            break;
        }
        // Wait as long as necessary (only when reading from SBF/PCAP file)
        if (settings_->read_from_sbf_log || settings_->read_from_pcap)
        {
            Timestamp time_obj = timestampFromRos(msg.header.stamp);
            wait(time_obj);
        }
        publish<GprmcMsg>("/gprmc", msg);
        break;
    }
    case evGPGSA:
    {
        boost::char_separator<char> sep("\r");
        typedef boost::tokenizer<boost::char_separator<char>> tokenizer;
        std::size_t nmea_size = this->messageSize();
        std::string block_in_string(reinterpret_cast<const char*>(data_), nmea_size);
        tokenizer tokens(block_in_string, sep);

        std::string id = this->messageID();
        std::string one_message = *tokens.begin();
        boost::char_separator<char> sep_2(",*", "", boost::keep_empty_tokens);
        tokenizer tokens_2(one_message, sep_2);
        std::vector<std::string> body;
        for (tokenizer::iterator tok_iter = tokens_2.begin();
             tok_iter != tokens_2.end(); ++tok_iter)
        {
            body.push_back(*tok_iter);
        }
        // Create NmeaSentence struct to pass to GpgsaParser::parseASCII
        NMEASentence gsa_message(id, body);
        GpgsaMsg msg;
        GpgsaParser parser_obj;
        try
        {
            msg = parser_obj.parseASCII(gsa_message, settings_->frame_id,
                                        settings_->use_gnss_time, node_->getTime());
        } catch (ParseException& e)
        {
            node_->log(LogLevel::DEBUG, "GpgsaMsg: " + std::string(e.what()));
            break;
        }
        if (settings_->septentrio_receiver_type == "gnss")
        {
            Timestamp time_obj;
            time_obj = timestampSBF(last_pvtgeodetic_.block_header.tow,
                                    last_pvtgeodetic_.block_header.wnc,
                                    settings_->use_gnss_time);
            msg.header.stamp = timestampToRos(time_obj);
        }
        if (settings_->septentrio_receiver_type == "ins")
        {
            Timestamp time_obj;
            time_obj = timestampSBF(last_insnavgeod_.block_header.tow,
                                    last_insnavgeod_.block_header.wnc,
                                    settings_->use_gnss_time);
            msg.header.stamp = timestampToRos(time_obj);
        }
        // Wait as long as necessary (only when reading from SBF/PCAP file)
        if (settings_->read_from_sbf_log || settings_->read_from_pcap)
        {
            Timestamp time_obj = timestampFromRos(msg.header.stamp);
            wait(time_obj);
        }
        publish<GpgsaMsg>("/gpgsa", msg);
        break;
    }
    case evGPGSV:
    case evGLGSV:
    case evGAGSV:
    {
        boost::char_separator<char> sep("\r");
        typedef boost::tokenizer<boost::char_separator<char>> tokenizer;
        std::size_t nmea_size = this->messageSize();
        std::string block_in_string(reinterpret_cast<const char*>(data_), nmea_size);
        tokenizer tokens(block_in_string, sep);

        std::string id = this->messageID();
        std::string one_message = *tokens.begin();
        boost::char_separator<char> sep_2(",*", "", boost::keep_empty_tokens);
        tokenizer tokens_2(one_message, sep_2);
        std::vector<std::string> body;
        for (tokenizer::iterator tok_iter = tokens_2.begin();
             tok_iter != tokens_2.end(); ++tok_iter)
        {
            body.push_back(*tok_iter);
        }
        // Create NmeaSentence struct to pass to GpgsvParser::parseASCII
        NMEASentence gsv_message(id, body);
        GpgsvMsg msg;
        GpgsvParser parser_obj;
        try
        {
            msg = parser_obj.parseASCII(gsv_message, settings_->frame_id,
                                        settings_->use_gnss_time, node_->getTime());
        } catch (ParseException& e)
        {
            node_->log(LogLevel::DEBUG, "GpgsvMsg: " + std::string(e.what()));
            break;
        }
        if (settings_->septentrio_receiver_type == "gnss")
        {
            Timestamp time_obj;
            time_obj = timestampSBF(last_pvtgeodetic_.block_header.tow,
                                    last_pvtgeodetic_.block_header.wnc,
                                    settings_->use_gnss_time);
            msg.header.stamp = timestampToRos(time_obj);
        }
        if (settings_->septentrio_receiver_type == "ins")
        {
            Timestamp time_obj;
            time_obj = timestampSBF(last_insnavgeod_.block_header.tow,
                                    last_insnavgeod_.block_header.wnc,
                                    settings_->use_gnss_time);
            msg.header.stamp = timestampToRos(time_obj);
        }
        // Wait as long as necessary (only when reading from SBF/PCAP file)
        if (settings_->read_from_sbf_log || settings_->read_from_pcap)
        {
            Timestamp time_obj = timestampFromRos(msg.header.stamp);
            wait(time_obj);
        }
        publish<GpgsvMsg>("/gpgsv", msg);
        break;
    }

        if (settings_->septentrio_receiver_type == "gnss")
        {
        case evNavSatFix:
        {
            NavSatFixMsg msg;
            try
            {
                msg = NavSatFixCallback();
            } catch (std::runtime_error& e)
            {
                node_->log(LogLevel::DEBUG,
                           "NavSatFixMsg: " + std::string(e.what()));
                break;
            }
            msg.header.frame_id = settings_->frame_id;
            Timestamp time_obj = timestampSBF(data_, settings_->use_gnss_time);
            msg.header.stamp = timestampToRos(time_obj);
            pvtgeodetic_has_arrived_navsatfix_ = false;
            poscovgeodetic_has_arrived_navsatfix_ = false;
            // Wait as long as necessary (only when reading from SBF/PCAP file)
            if (settings_->read_from_sbf_log || settings_->read_from_pcap)
            {
                wait(time_obj);
            }
            publish<NavSatFixMsg>("/navsatfix", msg);
            break;
        }
        }
        if (settings_->septentrio_receiver_type == "ins")
        {
        case evINSNavSatFix:
        {
            NavSatFixMsg msg;
            try
            {
                msg = NavSatFixCallback();
            } catch (std::runtime_error& e)
            {
                node_->log(LogLevel::DEBUG,
                           "NavSatFixMsg: " + std::string(e.what()));
                break;
            }
            if (settings_->ins_use_poi)
            {
                msg.header.frame_id = settings_->poi_frame_id;
            } else
            {
                msg.header.frame_id = settings_->frame_id;
            }
            Timestamp time_obj = timestampSBF(data_, settings_->use_gnss_time);
            msg.header.stamp = timestampToRos(time_obj);
            insnavgeod_has_arrived_navsatfix_ = false;
            // Wait as long as necessary (only when reading from SBF/PCAP file)
            if (settings_->read_from_sbf_log || settings_->read_from_pcap)
            {
                wait(time_obj);
            }
            publish<NavSatFixMsg>("/navsatfix", msg);
            break;
        }
        }

        if (settings_->septentrio_receiver_type == "gnss")
        {
        case evGPSFix:
        {
            GPSFixMsg msg;
            try
            {
                msg = GPSFixCallback();
            } catch (std::runtime_error& e)
            {
                node_->log(LogLevel::DEBUG, "GPSFixMsg: " + std::string(e.what()));
                break;
            }
            msg.header.frame_id = settings_->frame_id;
            msg.status.header.frame_id = settings_->frame_id;
            Timestamp time_obj = timestampSBF(data_, settings_->use_gnss_time);
            msg.header.stamp = timestampToRos(time_obj);
            msg.status.header.stamp = timestampToRos(time_obj);
            ++count_gpsfix_;
            channelstatus_has_arrived_gpsfix_ = false;
            measepoch_has_arrived_gpsfix_ = false;
            dop_has_arrived_gpsfix_ = false;
            pvtgeodetic_has_arrived_gpsfix_ = false;
            poscovgeodetic_has_arrived_gpsfix_ = false;
            velcovgeodetic_has_arrived_gpsfix_ = false;
            atteuler_has_arrived_gpsfix_ = false;
            attcoveuler_has_arrived_gpsfix_ = false;
            // Wait as long as necessary (only when reading from SBF/PCAP file)
            if (settings_->read_from_sbf_log || settings_->read_from_pcap)
            {
                wait(time_obj);
            }
            publish<GPSFixMsg>("/gpsfix", msg);
            break;
        }
        }
        if (settings_->septentrio_receiver_type == "ins")
        {
        case evINSGPSFix:
        {
            GPSFixMsg msg;
            try
            {
                msg = GPSFixCallback();
            } catch (std::runtime_error& e)
            {
                node_->log(LogLevel::DEBUG, "GPSFixMsg: " + std::string(e.what()));
                break;
            }
            if (settings_->ins_use_poi)
            {
                msg.header.frame_id = settings_->poi_frame_id;
            } else
            {
                msg.header.frame_id = settings_->frame_id;
            }
            msg.status.header.frame_id = msg.header.frame_id;
            Timestamp time_obj = timestampSBF(data_, settings_->use_gnss_time);
            msg.header.stamp = timestampToRos(time_obj);
            msg.status.header.stamp = timestampToRos(time_obj);
            ++count_gpsfix_;
            channelstatus_has_arrived_gpsfix_ = false;
            measepoch_has_arrived_gpsfix_ = false;
            dop_has_arrived_gpsfix_ = false;
            insnavgeod_has_arrived_gpsfix_ = false;
            // Wait as long as necessary (only when reading from SBF/PCAP file)
            if (settings_->read_from_sbf_log || settings_->read_from_pcap)
            {
                wait(time_obj);
            }
            publish<GPSFixMsg>("/gpsfix", msg);
            break;
        }
        }
        if (settings_->septentrio_receiver_type == "gnss")
        {
        case evPoseWithCovarianceStamped:
        {
            PoseWithCovarianceStampedMsg msg;
            try
            {
                msg = PoseWithCovarianceStampedCallback();
            } catch (std::runtime_error& e)
            {
                node_->log(LogLevel::DEBUG,
                           "PoseWithCovarianceStampedMsg: " + std::string(e.what()));
                break;
            }
            msg.header.frame_id = settings_->frame_id;
            Timestamp time_obj = timestampSBF(data_, settings_->use_gnss_time);
            msg.header.stamp = timestampToRos(time_obj);
            pvtgeodetic_has_arrived_pose_ = false;
            poscovgeodetic_has_arrived_pose_ = false;
            atteuler_has_arrived_pose_ = false;
            attcoveuler_has_arrived_pose_ = false;
            // Wait as long as necessary (only when reading from SBF/PCAP file)
            if (settings_->read_from_sbf_log || settings_->read_from_pcap)
            {
                wait(time_obj);
            }
            publish<PoseWithCovarianceStampedMsg>("/pose", msg);
            break;
        }
        }
        if (settings_->septentrio_receiver_type == "ins")
        {
        case evINSPoseWithCovarianceStamped:
        {
            PoseWithCovarianceStampedMsg msg;
            try
            {
                msg = PoseWithCovarianceStampedCallback();
            } catch (std::runtime_error& e)
            {
                node_->log(LogLevel::DEBUG,
                           "PoseWithCovarianceStampedMsg: " + std::string(e.what()));
                break;
            }
            if (settings_->ins_use_poi)
            {
                msg.header.frame_id = settings_->poi_frame_id;
            } else
            {
                msg.header.frame_id = settings_->frame_id;
            }
            Timestamp time_obj = timestampSBF(data_, settings_->use_gnss_time);
            msg.header.stamp = timestampToRos(time_obj);
            insnavgeod_has_arrived_pose_ = false;
            // Wait as long as necessary (only when reading from SBF/PCAP file)
            if (settings_->read_from_sbf_log || settings_->read_from_pcap)
            {
                wait(time_obj);
            }
            publish<PoseWithCovarianceStampedMsg>("/pose", msg);
            break;
        }
        }
    case evChannelStatus:
    {
        std::vector<uint8_t> dvec(data_,
                                  data_ + parsing_utilities::getLength(data_));
        if (!ChannelStatusParser(node_, dvec.begin(), dvec.end(),
                                 last_channelstatus_))
        {
            node_->log(LogLevel::ERROR,
                       "septentrio_gnss_driver: parse error in ChannelStatus");
            break;
        }
        channelstatus_has_arrived_gpsfix_ = true;
        break;
    }
    case evMeasEpoch:
    {
        std::vector<uint8_t> dvec(data_,
                                  data_ + parsing_utilities::getLength(data_));
        if (!MeasEpochParser(node_, dvec.begin(), dvec.end(), last_measepoch_))
        {
            node_->log(LogLevel::ERROR,
                       "septentrio_gnss_driver: parse error in MeasEpoch");
            break;
        }
        last_measepoch_.header.frame_id = settings_->frame_id;
        Timestamp time_obj = timestampSBF(data_, settings_->use_gnss_time);
        last_measepoch_.header.stamp = timestampToRos(time_obj);
        measepoch_has_arrived_gpsfix_ = true;
        if (settings_->publish_measepoch)
            publish<MeasEpochMsg>("/measepoch", last_measepoch_);
        break;
    }
    case evDOP:
    {
        std::vector<uint8_t> dvec(data_,
                                  data_ + parsing_utilities::getLength(data_));
        if (!DOPParser(node_, dvec.begin(), dvec.end(), last_dop_))
        {
            dop_has_arrived_gpsfix_ = false;
            node_->log(LogLevel::ERROR,
                       "septentrio_gnss_driver: parse error in DOP");
            break;
        }
        dop_has_arrived_gpsfix_ = true;
        break;
    }
    case evVelCovGeodetic:
    {
        std::vector<uint8_t> dvec(data_,
                                  data_ + parsing_utilities::getLength(data_));
        if (!VelCovGeodeticParser(node_, dvec.begin(), dvec.end(),
                                  last_velcovgeodetic_))
        {
            velcovgeodetic_has_arrived_gpsfix_ = false;
            node_->log(LogLevel::ERROR,
                       "septentrio_gnss_driver: parse error in VelCovGeodetic");
            break;
        }
        last_velcovgeodetic_.header.frame_id = settings_->frame_id;
        Timestamp time_obj = timestampSBF(data_, settings_->use_gnss_time);
        last_velcovgeodetic_.header.stamp = timestampToRos(time_obj);
        velcovgeodetic_has_arrived_gpsfix_ = true;
        // Wait as long as necessary (only when reading from SBF/PCAP file)
        if (settings_->read_from_sbf_log || settings_->read_from_pcap)
        {
            wait(time_obj);
        }
        if (settings_->publish_velcovgeodetic)
            publish<VelCovGeodeticMsg>("/velcovgeodetic", last_velcovgeodetic_);
        if (settings_->publish_twist)
        {
            TwistWithCovarianceStampedMsg twist = TwistCallback();
            publish<TwistWithCovarianceStampedMsg>("/twist", twist);
        }
        break;
    }
    case evDiagnosticArray:
    {
        DiagnosticArrayMsg msg;
        try
        {
            msg = DiagnosticArrayCallback();
        } catch (std::runtime_error& e)
        {
            node_->log(LogLevel::DEBUG,
                       "DiagnosticArrayMsg: " + std::string(e.what()));
            break;
        }
        if (settings_->septentrio_receiver_type == "gnss")
        {
            msg.header.frame_id = settings_->frame_id;
        }
        if (settings_->septentrio_receiver_type == "ins")
        {
            if (settings_->ins_use_poi)
            {
                msg.header.frame_id = settings_->poi_frame_id;
            } else
            {
                msg.header.frame_id = settings_->frame_id;
            }
        }
        Timestamp time_obj = timestampSBF(data_, settings_->use_gnss_time);
        msg.header.stamp = timestampToRos(time_obj);
        receiverstatus_has_arrived_diagnostics_ = false;
        qualityind_has_arrived_diagnostics_ = false;
        // Wait as long as necessary (only when reading from SBF/PCAP file)
        if (settings_->read_from_sbf_log || settings_->read_from_pcap)
        {
            wait(time_obj);
        }
        publish<DiagnosticArrayMsg>("/diagnostics", msg);
        break;
    }
    case evLocalization:
    {
        LocalizationUtmMsg msg;
        try
        {
            msg = LocalizationUtmCallback();
        } catch (std::runtime_error& e)
        {
            node_->log(LogLevel::DEBUG, "LocalizationMsg: " + std::string(e.what()));
            break;
        }
        Timestamp time_obj = timestampSBF(data_, settings_->use_gnss_time);
        msg.header.stamp = timestampToRos(time_obj);
        insnavgeod_has_arrived_localization_ = false;
        // Wait as long as necessary (only when reading from SBF/PCAP file)
        if (settings_->read_from_sbf_log || settings_->read_from_pcap)
        {
            wait(time_obj);
        }
        if (settings_->publish_localization)
            publish<LocalizationUtmMsg>("/localization", msg);
        if (settings_->publish_tf)
            publishTf(msg);
        break;
    }
    case evReceiverStatus:
    {
        std::vector<uint8_t> dvec(data_,
                                  data_ + parsing_utilities::getLength(data_));
        if (!ReceiverStatusParser(node_, dvec.begin(), dvec.end(),
                                  last_receiverstatus_))
        {
            receiverstatus_has_arrived_diagnostics_ = false;
            node_->log(LogLevel::ERROR,
                       "septentrio_gnss_driver: parse error in ReceiverStatus");
            break;
        }
        receiverstatus_has_arrived_diagnostics_ = true;
        break;
    }
    case evQualityInd:
    {
        std::vector<uint8_t> dvec(data_,
                                  data_ + parsing_utilities::getLength(data_));
        if (!QualityIndParser(node_, dvec.begin(), dvec.end(), last_qualityind_))
        {
            qualityind_has_arrived_diagnostics_ = false;
            node_->log(LogLevel::ERROR,
                       "septentrio_gnss_driver: parse error in QualityInd");
            break;
        }
        qualityind_has_arrived_diagnostics_ = true;
        break;
    }
    case evReceiverSetup:
    {
        std::vector<uint8_t> dvec(data_,
                                  data_ + parsing_utilities::getLength(data_));
        if (!ReceiverSetupParser(node_, dvec.begin(), dvec.end(),
                                 last_receiversetup_))
        {
            node_->log(LogLevel::ERROR,
                       "septentrio_gnss_driver: parse error in ReceiverSetup");
            break;
        }
        static int32_t ins_major = 1;
        static int32_t ins_minor = 3;
        static int32_t ins_patch = 2;
        static int32_t gnss_major = 4;
        static int32_t gnss_minor = 10;
        static int32_t gnss_patch = 0;
        boost::tokenizer<> tok(last_receiversetup_.rx_version);
        boost::tokenizer<>::iterator it = tok.begin();
        std::vector<int32_t> major_minor_patch;
        major_minor_patch.reserve(3);
        for (boost::tokenizer<>::iterator it = tok.begin(); it != tok.end(); ++it)
        {
            int32_t v = std::atoi(it->c_str());
            major_minor_patch.push_back(v);
        }
        if (major_minor_patch.size() < 3)
        {
            node_->log(LogLevel::ERROR,
                       "septentrio_gnss_driver: parse error of firmware version.");
        } else
        {
            if ((settings_->septentrio_receiver_type == "ins") ||
                settings_->ins_in_gnss_mode)
            {
                if ((major_minor_patch[0] < ins_major) ||
                    ((major_minor_patch[0] == ins_major) &&
                     (major_minor_patch[1] < ins_minor)) ||
                    ((major_minor_patch[0] == ins_major) &&
                     (major_minor_patch[1] == ins_minor) &&
                     (major_minor_patch[2] < ins_patch)))
                {
                    node_->log(
                        LogLevel::WARN,
                        "INS receiver has firmware version: " +
                            last_receiversetup_.rx_version +
                            ", which does not support all features. Please update to at least " +
                            std::to_string(ins_major) + "." +
                            std::to_string(ins_minor) + "." +
                            std::to_string(ins_patch) + " or consult README.");
                }
            } else if (settings_->septentrio_receiver_type == "gnss")
            {
                if (major_minor_patch[0] < 3)
                {
                    node_->log(
                        LogLevel::WARN,
                        "INS receiver seems to be used as GNSS. Some settings may trigger warnings or errors. Consider using 'ins_in_gnss_mode' as receiver type.");
                } else if ((major_minor_patch[0] < gnss_major) ||
                           ((major_minor_patch[0] == gnss_major) &&
                            (major_minor_patch[1] < gnss_minor)) ||
                           ((major_minor_patch[0] == gnss_major) &&
                            (major_minor_patch[1] == gnss_minor) &&
                            (major_minor_patch[2] < gnss_patch)))
                {
                    node_->log(
                        LogLevel::WARN,
                        "GNSS receiver has firmware version: " +
                            last_receiversetup_.rx_version +
                            ", which may not support all features. Please update to at least " +
                            std::to_string(gnss_major) + "." +
                            std::to_string(gnss_minor) + "." +
                            std::to_string(gnss_patch) + " or consult README.");
                } else
                    node_->log(LogLevel::ERROR, "gnss");
            }
        }

        break;
    }
    case evReceiverTime:
    {
        ReceiverTimeMsg msg;
        std::vector<uint8_t> dvec(data_,
                                  data_ + parsing_utilities::getLength(data_));
        if (!ReceiverTimeParser(node_, dvec.begin(), dvec.end(), msg))
        {
            node_->log(LogLevel::ERROR,
                       "septentrio_gnss_driver: parse error in ReceiverTime");
            break;
        }
        current_leap_seconds_ = msg.delta_ls;
        break;
    }

        // Many more to be implemented...
    }
    return true;
}

void io_comm_rx::RxMessage::wait(Timestamp time_obj)
{
    Timestamp unix_old = unix_time_;
    unix_time_ = time_obj;
    if ((unix_old != 0) && (unix_time_ != unix_old))
    {
        if (unix_time_ > unix_old)
        {
            auto sleep_nsec = unix_time_ - unix_old;

            std::stringstream ss;
            ss << "Waiting for " << sleep_nsec / 1000000 << " milliseconds...";
            node_->log(LogLevel::DEBUG, ss.str());

            std::this_thread::sleep_for(std::chrono::nanoseconds(sleep_nsec));
        }
    }

    // set leap seconds to paramter only if it was not set otherwise (by
    // ReceiverTime)
    if (current_leap_seconds_ == -128)
        current_leap_seconds_ = settings_->leap_seconds;
}

bool io_comm_rx::RxMessage::gnss_gpsfix_complete(uint32_t id)
{
    std::vector<bool> gpsfix_vec = {channelstatus_has_arrived_gpsfix_,
                                    measepoch_has_arrived_gpsfix_,
                                    dop_has_arrived_gpsfix_,
                                    pvtgeodetic_has_arrived_gpsfix_,
                                    poscovgeodetic_has_arrived_gpsfix_,
                                    velcovgeodetic_has_arrived_gpsfix_,
                                    atteuler_has_arrived_gpsfix_,
                                    attcoveuler_has_arrived_gpsfix_};
    return allTrue(gpsfix_vec, id);
}

bool io_comm_rx::RxMessage::ins_gpsfix_complete(uint32_t id)
{
    std::vector<bool> gpsfix_vec = {
        channelstatus_has_arrived_gpsfix_, measepoch_has_arrived_gpsfix_,
        dop_has_arrived_gpsfix_, insnavgeod_has_arrived_gpsfix_};
    return allTrue(gpsfix_vec, id);
}

bool io_comm_rx::RxMessage::gnss_navsatfix_complete(uint32_t id)
{
    std::vector<bool> navsatfix_vec = {pvtgeodetic_has_arrived_navsatfix_,
                                       poscovgeodetic_has_arrived_navsatfix_};
    return allTrue(navsatfix_vec, id);
}

bool io_comm_rx::RxMessage::ins_navsatfix_complete(uint32_t id)
{
    std::vector<bool> navsatfix_vec = {insnavgeod_has_arrived_navsatfix_};
    return allTrue(navsatfix_vec, id);
}

bool io_comm_rx::RxMessage::gnss_pose_complete(uint32_t id)
{
    std::vector<bool> pose_vec = {
        pvtgeodetic_has_arrived_pose_, poscovgeodetic_has_arrived_pose_,
        atteuler_has_arrived_pose_, attcoveuler_has_arrived_pose_};
    return allTrue(pose_vec, id);
}

bool io_comm_rx::RxMessage::ins_pose_complete(uint32_t id)
{
    std::vector<bool> pose_vec = {insnavgeod_has_arrived_pose_};
    return allTrue(pose_vec, id);
}

bool io_comm_rx::RxMessage::diagnostics_complete(uint32_t id)
{
    std::vector<bool> diagnostics_vec = {receiverstatus_has_arrived_diagnostics_,
                                         qualityind_has_arrived_diagnostics_};
    return allTrue(diagnostics_vec, id);
}

bool io_comm_rx::RxMessage::ins_localization_complete(uint32_t id)
{
    std::vector<bool> loc_vec = {insnavgeod_has_arrived_localization_};
    return allTrue(loc_vec, id);
}

bool io_comm_rx::RxMessage::allTrue(std::vector<bool>& vec, uint32_t id)
{
    vec.erase(vec.begin() + id);
    // Checks whether all entries in vec are true
    return (std::all_of(vec.begin(), vec.end(), [](bool v) { return v; }) == true);
}
