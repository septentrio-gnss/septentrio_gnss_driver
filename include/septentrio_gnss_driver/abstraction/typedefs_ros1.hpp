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

#pragma once

// std includes
#include <numeric>
#include <unordered_map>
// ROS includes
#include <ros/ros.h>
// tf2 includes
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
// ROS msg includes
#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <gps_common/GPSFix.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/TimeReference.h>
// GNSS msg includes
#include <septentrio_gnss_driver/AIMPlusStatus.h>
#include <septentrio_gnss_driver/AttCovEuler.h>
#include <septentrio_gnss_driver/AttEuler.h>
#include <septentrio_gnss_driver/BaseVectorCart.h>
#include <septentrio_gnss_driver/BaseVectorGeod.h>
#include <septentrio_gnss_driver/BlockHeader.h>
#include <septentrio_gnss_driver/GALAuthStatus.h>
#include <septentrio_gnss_driver/MeasEpoch.h>
#include <septentrio_gnss_driver/MeasEpochChannelType1.h>
#include <septentrio_gnss_driver/MeasEpochChannelType2.h>
#include <septentrio_gnss_driver/PVTCartesian.h>
#include <septentrio_gnss_driver/PVTGeodetic.h>
#include <septentrio_gnss_driver/PosCovCartesian.h>
#include <septentrio_gnss_driver/PosCovGeodetic.h>
#include <septentrio_gnss_driver/RFBand.h>
#include <septentrio_gnss_driver/RFStatus.h>
#include <septentrio_gnss_driver/ReceiverTime.h>
#include <septentrio_gnss_driver/VectorInfoCart.h>
#include <septentrio_gnss_driver/VectorInfoGeod.h>
#include <septentrio_gnss_driver/VelCovCartesian.h>
#include <septentrio_gnss_driver/VelCovGeodetic.h>
// NMEA msg includes
#include <nmea_msgs/Gpgga.h>
#include <nmea_msgs/Gpgsa.h>
#include <nmea_msgs/Gpgsv.h>
#include <nmea_msgs/Gprmc.h>
// INS msg includes
#include <septentrio_gnss_driver/ExtSensorMeas.h>
#include <septentrio_gnss_driver/IMUSetup.h>
#include <septentrio_gnss_driver/INSNavCart.h>
#include <septentrio_gnss_driver/INSNavGeod.h>
#include <septentrio_gnss_driver/VelSensorSetup.h>
// Rosaic includes
#include <septentrio_gnss_driver/communication/settings.hpp>
#include <septentrio_gnss_driver/parsers/sbf_utilities.hpp>
#include <septentrio_gnss_driver/parsers/string_utilities.hpp>

// Timestamp in nanoseconds (Unix epoch)
typedef uint64_t Timestamp;
// ROS timestamp
typedef ros::Time TimestampRos;

// ROS messages
typedef diagnostic_msgs::DiagnosticArray DiagnosticArrayMsg;
typedef diagnostic_msgs::DiagnosticStatus DiagnosticStatusMsg;
typedef geometry_msgs::Quaternion QuaternionMsg;
typedef geometry_msgs::PoseWithCovarianceStamped PoseWithCovarianceStampedMsg;
typedef geometry_msgs::TwistWithCovarianceStamped TwistWithCovarianceStampedMsg;
typedef geometry_msgs::TransformStamped TransformStampedMsg;
typedef geometry_msgs::Vector3 Vector3Msg;
typedef gps_common::GPSFix GpsFixMsg;
typedef gps_common::GPSStatus GpsStatusMsg;
typedef sensor_msgs::NavSatFix NavSatFixMsg;
typedef sensor_msgs::NavSatStatus NavSatStatusMsg;
typedef sensor_msgs::TimeReference TimeReferenceMsg;
typedef sensor_msgs::Imu ImuMsg;
typedef nav_msgs::Odometry LocalizationMsg;

// Septentrio GNSS SBF messages
typedef septentrio_gnss_driver::AIMPlusStatus AimPlusStatusMsg;
typedef septentrio_gnss_driver::BaseVectorCart BaseVectorCartMsg;
typedef septentrio_gnss_driver::BaseVectorGeod BaseVectorGeodMsg;
typedef septentrio_gnss_driver::BlockHeader BlockHeaderMsg;
typedef septentrio_gnss_driver::GALAuthStatus GalAuthStatusMsg;
typedef septentrio_gnss_driver::RFStatus RfStatusMsg;
typedef septentrio_gnss_driver::RFBand RfBandMsg;
typedef septentrio_gnss_driver::MeasEpoch MeasEpochMsg;
typedef septentrio_gnss_driver::MeasEpochChannelType1 MeasEpochChannelType1Msg;
typedef septentrio_gnss_driver::MeasEpochChannelType2 MeasEpochChannelType2Msg;
typedef septentrio_gnss_driver::AttCovEuler AttCovEulerMsg;
typedef septentrio_gnss_driver::AttEuler AttEulerMsg;
typedef septentrio_gnss_driver::PVTCartesian PVTCartesianMsg;
typedef septentrio_gnss_driver::PVTGeodetic PVTGeodeticMsg;
typedef septentrio_gnss_driver::PosCovCartesian PosCovCartesianMsg;
typedef septentrio_gnss_driver::PosCovGeodetic PosCovGeodeticMsg;
typedef septentrio_gnss_driver::ReceiverTime ReceiverTimeMsg;
typedef septentrio_gnss_driver::VectorInfoCart VectorInfoCartMsg;
typedef septentrio_gnss_driver::VectorInfoGeod VectorInfoGeodMsg;
typedef septentrio_gnss_driver::VelCovCartesian VelCovCartesianMsg;
typedef septentrio_gnss_driver::VelCovGeodetic VelCovGeodeticMsg;

// NMEA messages
typedef nmea_msgs::Gpgga GpggaMsg;
typedef nmea_msgs::Gpgsa GpgsaMsg;
typedef nmea_msgs::Gpgsv GpgsvMsg;
typedef nmea_msgs::Gprmc GprmcMsg;

// Septentrio INS+GNSS SBF messages
typedef septentrio_gnss_driver::INSNavCart INSNavCartMsg;
typedef septentrio_gnss_driver::INSNavGeod INSNavGeodMsg;
typedef septentrio_gnss_driver::IMUSetup IMUSetupMsg;
typedef septentrio_gnss_driver::VelSensorSetup VelSensorSetupMsg;
typedef septentrio_gnss_driver::ExtSensorMeas ExtSensorMeasMsg;

/**
 * @brief Convert nsec timestamp to ROS timestamp
 * @param[in] ts timestamp in nanoseconds (Unix epoch)
 * @return ROS timestamp
 */
inline TimestampRos timestampToRos(Timestamp ts)
{
    TimestampRos tsr;
    tsr.fromNSec(ts);
    return tsr;
}

/**
 * @brief Convert ROS timestamp to nsec timestamp
 * @param[in] ts ROS timestamp
 * @return timestamp in nanoseconds (Unix epoch)
 */
inline Timestamp timestampFromRos(const TimestampRos& tsr) { return tsr.toNSec(); }

/**
 * @brief Log level for ROS logging
 */
namespace log_level {
    enum LogLevel
    {
        DEBUG,
        INFO,
        WARN,
        ERROR,
        FATAL
    };
} // namespace log_level

/**
 * @class ROSaicNodeBase
 * @brief This class is the base class for abstraction
 */
class ROSaicNodeBase
{
public:
    ROSaicNodeBase() :
        pNh_(std::make_shared<ros::NodeHandle>("~")), tfListener_(tfBuffer_),
        lastTfStamp_(0)
    {
    }

    ~ROSaicNodeBase() {}

    bool ok() { return ros::ok(); }

    const Settings* settings() const { return &settings_; }

    void registerSubscriber()
    {
        try
        {
            ros::NodeHandle nh;
            if (settings_.ins_vsm.ros_source == "odometry")
                odometrySubscriber_ = nh.subscribe<nav_msgs::Odometry>(
                    "odometry_vsm", 10, &ROSaicNodeBase::callbackOdometry, this);
            else if (settings_.ins_vsm.ros_source == "twist")
                twistSubscriber_ = nh.subscribe<TwistWithCovarianceStampedMsg>(
                    "twist_vsm", 10, &ROSaicNodeBase::callbackTwist, this);
        } catch (const std::runtime_error& ex)
        {
            this->log(log_level::ERROR, "Subscriber initialization failed due to: " +
                                            std::string(ex.what()) + ".");
        }
    }

    /**
     * @brief Gets an integer or unsigned integer value from the
     * parameter server
     * @param[in] name The key to be used in the parameter server's
     * dictionary
     * @param[out] val Storage for the retrieved value, of type U, which
     * can be either unsigned int or int
     * @param[in] defaultVal Value to use if the server doesn't contain
     * this parameter
     */
    bool getUint32Param(const std::string& name, uint32_t& val,
                        uint32_t defaultVal) const
    {
        int32_t tempVal;
        if ((!pNh_->getParam(name, tempVal)) || (tempVal < 0))
        {
            val = defaultVal;
            return false;
        }
        val = tempVal;
        return true;
    }

    /**
     * @brief Gets parameter of type T from the parameter server
     * @param[in] name The key to be used in the parameter server's
     * dictionary
     * @param[out] val Storage for the retrieved value, of type T
     * @param[in] defaultVal Value to use if the server doesn't contain
     * this parameter
     * @return True if it could be retrieved, false if not
     */
    template <typename T>
    bool param(const std::string& name, T& val, const T& defaultVal) const
    {
        return pNh_->param(name, val, defaultVal);
    };

    /**
     * @brief Log function to provide abstraction of ROS loggers
     * @param[in] logLevel Log level
     * @param[in] s String to log
     */
    void log(log_level::LogLevel logLevel, const std::string& s) const
    {
        switch (logLevel)
        {
        case log_level::DEBUG:
            ROS_DEBUG_STREAM(ros::this_node::getName() << ": " << s);
            break;
        case log_level::INFO:
            ROS_INFO_STREAM(ros::this_node::getName() << ": " << s);
            break;
        case log_level::WARN:
            ROS_WARN_STREAM(ros::this_node::getName() << ": " << s);
            break;
        case log_level::ERROR:
            ROS_ERROR_STREAM(ros::this_node::getName() << ": " << s);
            break;
        case log_level::FATAL:
            ROS_FATAL_STREAM(ros::this_node::getName() << ": " << s);
            break;
        default:
            break;
        }
    }

    /**
     * @brief Gets current timestamp
     * @return Timestamp
     */
    Timestamp getTime() const { return ros::Time::now().toNSec(); }

    /**
     * @brief Publishing function
     * @param[in] topic String of topic
     * @param[in] msg ROS message to be published
     */
    template <typename M>
    void publishMessage(const std::string& topic, const M& msg)
    {
        if constexpr (has_block_header<M>::value)
        {
            if (settings_.publish_only_valid && !validValue(msg.block_header.tow))
                return;
        }

        auto it = topicMap_.find(topic);
        if (it != topicMap_.end())
        {
            it->second.publish(msg);
        } else
        {
            ros::Publisher pub = pNh_->advertise<M>(topic, queueSize_);
            topicMap_.insert(std::make_pair(topic, pub));
            pub.publish(msg);
        }
    }

    /**
     * @brief Publishing function for tf
     * @param[in] msg ROS localization message to be converted to tf
     */
    void publishTf(const LocalizationMsg& loc)
    {
        if (std::isnan(loc.pose.pose.orientation.w))
            return;

        if (lastTfStamp_ == loc.header.stamp)
            return;
        lastTfStamp_ = loc.header.stamp;

        geometry_msgs::TransformStamped transformStamped;
        transformStamped.header.stamp = loc.header.stamp;
        transformStamped.header.frame_id = loc.header.frame_id;
        transformStamped.child_frame_id = loc.child_frame_id;
        transformStamped.transform.translation.x = loc.pose.pose.position.x;
        transformStamped.transform.translation.y = loc.pose.pose.position.y;
        transformStamped.transform.translation.z = loc.pose.pose.position.z;
        transformStamped.transform.rotation.x = loc.pose.pose.orientation.x;
        transformStamped.transform.rotation.y = loc.pose.pose.orientation.y;
        transformStamped.transform.rotation.z = loc.pose.pose.orientation.z;
        transformStamped.transform.rotation.w = loc.pose.pose.orientation.w;

        if (settings_.insert_local_frame)
        {
            geometry_msgs::TransformStamped T_l_b;
            try
            {
                // try to get tf at timestamp of message
                T_l_b = tfBuffer_.lookupTransform(
                    loc.child_frame_id, settings_.local_frame_id, lastTfStamp_);
            } catch (const tf2::TransformException& ex)
            {
                try
                {
                    ROS_INFO_STREAM_THROTTLE(
                        10.0,
                        ros::this_node::getName()
                            << ": No transform for insertion of local frame at t="
                            << lastTfStamp_.toNSec()
                            << ". Exception: " << std::string(ex.what()));
                    // try to get latest tf
                    T_l_b = tfBuffer_.lookupTransform(
                        loc.child_frame_id, settings_.local_frame_id, ros::Time(0));
                } catch (const tf2::TransformException& ex)
                {
                    ROS_WARN_STREAM_THROTTLE(
                        10.0,
                        ros::this_node::getName()
                            << ": No most recent transform for insertion of local frame. Exception: "
                            << std::string(ex.what()));
                    return;
                }
            }

            // T_l_g = T_b_g * T_l_b;
            transformStamped =
                tf2::eigenToTransform(tf2::transformToEigen(transformStamped) *
                                      tf2::transformToEigen(T_l_b));
            transformStamped.header.stamp = loc.header.stamp;
            transformStamped.header.frame_id = loc.header.frame_id;
            transformStamped.child_frame_id = settings_.local_frame_id;
        }

        tf2Publisher_.sendTransform(transformStamped);
    }

    /**
     * @brief Set INS to true
     */
    void setIsIns() { capabilities_.is_ins = true; }

    /**
     * @brief Set has heading to true
     */
    void setHasHeading() { capabilities_.has_heading = true; }

    /**
     * @brief Set improved VSM handling to true
     */
    void setImprovedVsmHandling() { capabilities_.has_improved_vsm_handling = true; }

    /**
     * @brief Check if Rx is INS
     */
    bool isIns() { return capabilities_.is_ins; }

    /**
     * @brief Check if Rx has heading
     */
    bool hasHeading() { return capabilities_.has_heading; }

    /**
     * @brief Check if Rx has improved VSM handling
     */
    bool hasImprovedVsmHandling() { return capabilities_.has_improved_vsm_handling; }

private:
    void callbackOdometry(const nav_msgs::Odometry::ConstPtr& odo)
    {
        Timestamp stamp = timestampFromRos(odo->header.stamp);

        processTwist(stamp, odo->twist);
    }

    void callbackTwist(const TwistWithCovarianceStampedMsg::ConstPtr& twist)
    {
        Timestamp stamp = timestampFromRos(twist->header.stamp);

        processTwist(stamp, twist->twist);
    }

    void processTwist(Timestamp stamp,
                      const geometry_msgs::TwistWithCovariance& twist)
    {
        // in case stamp was not set
        if (stamp == 0)
            stamp = getTime();

        thread_local Eigen::Vector3d vel = Eigen::Vector3d::Zero();
        thread_local Eigen::Vector3d var = Eigen::Vector3d::Zero();
        thread_local uint64_t ctr = 0;
        thread_local Timestamp lastStamp = 0;

        ++ctr;
        vel[0] += twist.twist.linear.x;
        vel[1] += twist.twist.linear.y;
        vel[2] += twist.twist.linear.z;
        var[0] += twist.covariance[0];
        var[1] += twist.covariance[7];
        var[2] += twist.covariance[14];

        // Rx expects averaged velocity at a rate of 2 Hz
        if ((stamp - lastStamp) >= 495000000) // allow for 5 ms jitter
        {
            vel /= ctr;
            var /= ctr;
            time_t epochSeconds = stamp / 1000000000;
            struct tm* tm_temp = std::gmtime(&epochSeconds);
            std::stringstream timeUtc;
            timeUtc << std::setfill('0') << std::setw(2)
                    << std::to_string(tm_temp->tm_hour) << std::setw(2)
                    << std::to_string(tm_temp->tm_min) << std::setw(2)
                    << std::to_string(tm_temp->tm_sec) << "." << std::setw(3)
                    << std::to_string((stamp - (stamp / 1000000000) * 1000000000) /
                                      1000000);

            std::string v_x;
            std::string v_y;
            std::string v_z;
            std::string std_x;
            std::string std_y;
            std::string std_z;
            if (settings_.ins_vsm.ros_config[0])
            {
                v_x = string_utilities::trimDecimalPlaces(vel[0]);
                if (settings_.ins_vsm.ros_variances_by_parameter)
                    std_x = string_utilities::trimDecimalPlaces(
                        settings_.ins_vsm.ros_variances[0]);
                else if (var[0] > 0.0)
                    std_x = string_utilities::trimDecimalPlaces(std::sqrt(var[0]));
                else if (!capabilities_.has_improved_vsm_handling)
                {
                    log(log_level::ERROR, "Invalid covariance value for v_x: " +
                                              std::to_string(var[0]) +
                                              ". Ignoring measurement.");
                    v_x = "";
                    std_x = string_utilities::trimDecimalPlaces(1000000.0);
                }
            } else
                std_x = std::to_string(1000000.0);
            if (settings_.ins_vsm.ros_config[1])
            {
                if (settings_.use_ros_axis_orientation)
                    v_y = "-";
                v_y += string_utilities::trimDecimalPlaces(vel[1]);
                if (settings_.ins_vsm.ros_variances_by_parameter)
                    std_y = string_utilities::trimDecimalPlaces(
                        settings_.ins_vsm.ros_variances[1]);
                else if (var[1] > 0.0)
                    std_y = string_utilities::trimDecimalPlaces(std::sqrt(var[1]));
                else if (!capabilities_.has_improved_vsm_handling)
                {
                    log(log_level::ERROR, "Invalid covariance value for v_y: " +
                                              std::to_string(var[1]) +
                                              ". Ignoring measurement.");
                    v_y = "";
                    std_y = string_utilities::trimDecimalPlaces(1000000.0);
                }
            } else
                std_y = string_utilities::trimDecimalPlaces(1000000.0);
            if (settings_.ins_vsm.ros_config[2])
            {
                if (settings_.use_ros_axis_orientation)
                    v_z = "-";
                v_z += string_utilities::trimDecimalPlaces(vel[2]);
                if (settings_.ins_vsm.ros_variances_by_parameter)
                    std_z = string_utilities::trimDecimalPlaces(
                        settings_.ins_vsm.ros_variances[2]);
                else if (var[2] > 0.0)
                    std_z = string_utilities::trimDecimalPlaces(std::sqrt(var[2]));
                else if (!capabilities_.has_improved_vsm_handling)
                {
                    log(log_level::ERROR, "Invalid covariance value for v_z: " +
                                              std::to_string(var[2]) +
                                              ". Ignoring measurement.");
                    v_z = "";
                    std_z = string_utilities::trimDecimalPlaces(1000000.0);
                }
            } else
                std_z = string_utilities::trimDecimalPlaces(1000000.0);

            std::string velNmea = "$PSSN,VSM," + timeUtc.str() + "," + v_x + "," +
                                  v_y + "," + std_x + "," + std_y + "," + v_z + "," +
                                  std_z;

            char crc = std::accumulate(velNmea.begin() + 1, velNmea.end(), 0,
                                       [](char sum, char ch) { return sum ^ ch; });

            std::stringstream crcss;
            crcss << std::hex << static_cast<int32_t>(crc);

            velNmea += "*" + crcss.str() + "\r\n";
            sendVelocity(velNmea);

            vel = Eigen::Vector3d::Zero();
            var = Eigen::Vector3d::Zero();
            ctr = 0;
            lastStamp = stamp;
        }
    }

protected:
    //! Node handle pointer
    std::shared_ptr<ros::NodeHandle> pNh_;
    //! Settings
    Settings settings_;
    //! Send velocity to communication layer (virtual)
    virtual void sendVelocity(const std::string& velNmea) = 0;

private:
    //! Map of topics and publishers
    std::unordered_map<std::string, ros::Publisher> topicMap_;
    //! Publisher queue size
    uint32_t queueSize_ = 1;
    //! Transform publisher
    tf2_ros::TransformBroadcaster tf2Publisher_;
    //! Odometry subscriber
    ros::Subscriber odometrySubscriber_;
    //! Twist subscriber
    ros::Subscriber twistSubscriber_;
    //! Last tf stamp
    TimestampRos lastTfStamp_;
    //! tf buffer
    tf2_ros::Buffer tfBuffer_;
    // tf listener
    tf2_ros::TransformListener tfListener_;
    // Capabilities of Rx
    Capabilities capabilities_;
};