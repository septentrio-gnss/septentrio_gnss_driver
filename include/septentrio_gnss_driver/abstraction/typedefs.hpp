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
#include <any>
#include <iomanip>
#include <sstream>
#include <unordered_map>
// ROS includes
#include <rclcpp/rclcpp.hpp>
// tf2 includes
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#ifdef ROS2_VER_N250
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#else
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#endif
// ROS msg includes
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/twist_with_covariance.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <gps_msgs/msg/gps_fix.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/time_reference.hpp>
// GNSS msg includes
#include <septentrio_gnss_driver/msg/aim_plus_status.hpp>
#include <septentrio_gnss_driver/msg/att_cov_euler.hpp>
#include <septentrio_gnss_driver/msg/att_euler.hpp>
#include <septentrio_gnss_driver/msg/base_vector_cart.hpp>
#include <septentrio_gnss_driver/msg/base_vector_geod.hpp>
#include <septentrio_gnss_driver/msg/block_header.hpp>
#include <septentrio_gnss_driver/msg/gal_auth_status.hpp>
#include <septentrio_gnss_driver/msg/meas_epoch.hpp>
#include <septentrio_gnss_driver/msg/meas_epoch_channel_type1.hpp>
#include <septentrio_gnss_driver/msg/meas_epoch_channel_type2.hpp>
#include <septentrio_gnss_driver/msg/pos_cov_cartesian.hpp>
#include <septentrio_gnss_driver/msg/pos_cov_geodetic.hpp>
#include <septentrio_gnss_driver/msg/pvt_cartesian.hpp>
#include <septentrio_gnss_driver/msg/pvt_geodetic.hpp>
#include <septentrio_gnss_driver/msg/receiver_time.hpp>
#include <septentrio_gnss_driver/msg/rf_band.hpp>
#include <septentrio_gnss_driver/msg/rf_status.hpp>
#include <septentrio_gnss_driver/msg/vector_info_cart.hpp>
#include <septentrio_gnss_driver/msg/vector_info_geod.hpp>
#include <septentrio_gnss_driver/msg/vel_cov_cartesian.hpp>
#include <septentrio_gnss_driver/msg/vel_cov_geodetic.hpp>
// NMEA msg includes
#include <nmea_msgs/msg/gpgga.hpp>
#include <nmea_msgs/msg/gpgsa.hpp>
#include <nmea_msgs/msg/gpgsv.hpp>
#include <nmea_msgs/msg/gprmc.hpp>
// INS msg includes
#include <septentrio_gnss_driver/msg/ext_sensor_meas.hpp>
#include <septentrio_gnss_driver/msg/imu_setup.hpp>
#include <septentrio_gnss_driver/msg/ins_nav_cart.hpp>
#include <septentrio_gnss_driver/msg/ins_nav_geod.hpp>
#include <septentrio_gnss_driver/msg/vel_sensor_setup.hpp>
// Rosaic includes
#include <septentrio_gnss_driver/communication/settings.hpp>
#include <septentrio_gnss_driver/parsers/sbf_utilities.hpp>
#include <septentrio_gnss_driver/parsers/string_utilities.hpp>

// Timestamp in nanoseconds (Unix epoch)
typedef uint64_t Timestamp;
// ROS timestamp
typedef rclcpp::Time TimestampRos;

// ROS messages
typedef diagnostic_msgs::msg::DiagnosticArray DiagnosticArrayMsg;
typedef diagnostic_msgs::msg::DiagnosticStatus DiagnosticStatusMsg;
typedef geometry_msgs::msg::Quaternion QuaternionMsg;
typedef geometry_msgs::msg::PoseWithCovarianceStamped PoseWithCovarianceStampedMsg;
typedef geometry_msgs::msg::TwistWithCovarianceStamped TwistWithCovarianceStampedMsg;
typedef geometry_msgs::msg::TransformStamped TransformStampedMsg;
typedef gps_msgs::msg::GPSFix GpsFixMsg;
typedef gps_msgs::msg::GPSStatus GpsStatusMsg;
typedef sensor_msgs::msg::NavSatFix NavSatFixMsg;
typedef sensor_msgs::msg::NavSatStatus NavSatStatusMsg;
typedef sensor_msgs::msg::TimeReference TimeReferenceMsg;
typedef sensor_msgs::msg::Imu ImuMsg;
typedef nav_msgs::msg::Odometry LocalizationMsg;

// Septentrio GNSS SBF messages
typedef septentrio_gnss_driver::msg::AIMPlusStatus AimPlusStatusMsg;
typedef septentrio_gnss_driver::msg::BaseVectorCart BaseVectorCartMsg;
typedef septentrio_gnss_driver::msg::BaseVectorGeod BaseVectorGeodMsg;
typedef septentrio_gnss_driver::msg::BlockHeader BlockHeaderMsg;
typedef septentrio_gnss_driver::msg::GALAuthStatus GalAuthStatusMsg;
typedef septentrio_gnss_driver::msg::RFStatus RfStatusMsg;
typedef septentrio_gnss_driver::msg::RFBand RfBandMsg;
typedef septentrio_gnss_driver::msg::MeasEpoch MeasEpochMsg;
typedef septentrio_gnss_driver::msg::MeasEpochChannelType1 MeasEpochChannelType1Msg;
typedef septentrio_gnss_driver::msg::MeasEpochChannelType2 MeasEpochChannelType2Msg;
typedef septentrio_gnss_driver::msg::AttCovEuler AttCovEulerMsg;
typedef septentrio_gnss_driver::msg::AttEuler AttEulerMsg;
typedef septentrio_gnss_driver::msg::PVTCartesian PVTCartesianMsg;
typedef septentrio_gnss_driver::msg::PVTGeodetic PVTGeodeticMsg;
typedef septentrio_gnss_driver::msg::PosCovCartesian PosCovCartesianMsg;
typedef septentrio_gnss_driver::msg::PosCovGeodetic PosCovGeodeticMsg;
typedef septentrio_gnss_driver::msg::ReceiverTime ReceiverTimeMsg;
typedef septentrio_gnss_driver::msg::VectorInfoCart VectorInfoCartMsg;
typedef septentrio_gnss_driver::msg::VectorInfoGeod VectorInfoGeodMsg;
typedef septentrio_gnss_driver::msg::VelCovCartesian VelCovCartesianMsg;
typedef septentrio_gnss_driver::msg::VelCovGeodetic VelCovGeodeticMsg;

// NMEA message
typedef nmea_msgs::msg::Gpgga GpggaMsg;
typedef nmea_msgs::msg::Gpgsa GpgsaMsg;
typedef nmea_msgs::msg::Gpgsv GpgsvMsg;
typedef nmea_msgs::msg::Gprmc GprmcMsg;

// Septentrio INS+GNSS SBF messages
typedef septentrio_gnss_driver::msg::INSNavCart INSNavCartMsg;
typedef septentrio_gnss_driver::msg::INSNavGeod INSNavGeodMsg;
typedef septentrio_gnss_driver::msg::IMUSetup IMUSetupMsg;
typedef septentrio_gnss_driver::msg::VelSensorSetup VelSensorSetupMsg;
typedef septentrio_gnss_driver::msg::ExtSensorMeas ExtSensorMeasMsg;

/**
 * @brief Convert nsec timestamp to ROS timestamp
 * @param[in] ts timestamp in nanoseconds (Unix epoch)
 * @return ROS timestamp
 */
inline TimestampRos timestampToRos(Timestamp ts) { return TimestampRos(ts); }

/**
 * @brief Convert ROS timestamp to nsec timestamp
 * @param[in] ts ROS timestamp
 * @return timestamp in nanoseconds (Unix epoch)
 */
inline Timestamp timestampFromRos(const TimestampRos& tsr)
{
    return tsr.nanoseconds();
}

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
class ROSaicNodeBase : public rclcpp::Node
{
public:
    ROSaicNodeBase(const rclcpp::NodeOptions& options) :
        Node("septentrio_gnss", options), tf2Publisher_(this),
        tfBuffer_(this->get_clock()), tfListener_(tfBuffer_)
    {
    }

    ~ROSaicNodeBase() {}

    bool ok() { return rclcpp::ok(); }

    const Settings* settings() const { return &settings_; }

    void registerSubscriber()
    {
        try
        {
            if (settings_.ins_vsm.ros_source == "odometry")
                odometrySubscriber_ =
                    this->create_subscription<nav_msgs::msg::Odometry>(
                        "odometry_vsm",
                        rclcpp::QoS(rclcpp::KeepLast(1))
                            .durability_volatile()
                            .best_effort(),
                        std::bind(&ROSaicNodeBase::callbackOdometry, this,
                                  std::placeholders::_1));
            else if (settings_.ins_vsm.ros_source == "twist")
                twistSubscriber_ =
                    this->create_subscription<TwistWithCovarianceStampedMsg>(
                        "twist_vsm",
                        rclcpp::QoS(rclcpp::KeepLast(1))
                            .durability_volatile()
                            .best_effort(),
                        std::bind(&ROSaicNodeBase::callbackTwist, this,
                                  std::placeholders::_1));
        } catch (const std::runtime_error& ex)
        {
            this->log(log_level::ERROR, "Subscriber initialization failed due to: " +
                                            std::string(ex.what()) + ".");
        }
    }

    /**
     * @brief Gets an integer or unsigned integer value from the parameter server
     * @param[in] name The key to be used in the parameter server's dictionary
     * @param[out] val Storage for the retrieved value, of type U, which can be
     * either unsigned int or int
     * @param[in] defaultVal Value to use if the server doesn't contain this
     * parameter
     */
    bool getUint32Param(const std::string& name, uint32_t& val, uint32_t defaultVal)
    {
        int32_t tempVal;
        if ((!this->param(name, tempVal, -1)) || (tempVal < 0))
        {
            val = defaultVal;
            return false;
        }
        val = tempVal;
        return true;
    }

    /**
     * @brief Gets parameter of type T from the parameter server
     * @param[in] name The key to be used in the parameter server's dictionary
     * @param[out] val Storage for the retrieved value, of type T
     * @param[in] defaultVal Value to use if the server doesn't contain this
     * parameter
     * @return True if it could be retrieved, false if not
     */
    template <typename T>
    bool param(const std::string& name, T& val, const T& defaultVal)
    {
        if (this->has_parameter(name))
            this->undeclare_parameter(name);

        try
        {
            val = this->declare_parameter<T>(name, defaultVal);
        } catch (std::runtime_error& e)
        {
            RCLCPP_WARN_STREAM(this->get_logger(), e.what());
            return false;
        }
        return true;
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
            RCLCPP_DEBUG_STREAM(this->get_logger(), s);
            break;
        case log_level::INFO:
            RCLCPP_INFO_STREAM(this->get_logger(), s);
            break;
        case log_level::WARN:
            RCLCPP_WARN_STREAM(this->get_logger(), s);
            break;
        case log_level::ERROR:
            RCLCPP_ERROR_STREAM(this->get_logger(), s);
            break;
        case log_level::FATAL:
            RCLCPP_FATAL_STREAM(this->get_logger(), s);
            break;
        default:
            break;
        }
    }

    /**
     * @brief Gets current timestamp
     * @return Timestamp
     */
    Timestamp getTime() const { return this->now().nanoseconds(); }

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
            typename rclcpp::Publisher<M>::SharedPtr ptr =
                std::any_cast<typename rclcpp::Publisher<M>::SharedPtr>(it->second);
            ptr->publish(msg);
        } else
        {
            if (this->ok())
            {
                typename rclcpp::Publisher<M>::SharedPtr pub =
                    this->create_publisher<M>(
                        topic, rclcpp::QoS(rclcpp::KeepLast(queueSize_))
                                   .durability_volatile()
                                   .reliable());
                topicMap_.insert(std::make_pair(topic, pub));
                pub->publish(msg);
            }
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

        Timestamp currentStamp = timestampFromRos(loc.header.stamp);
        if (lastTfStamp_ == currentStamp)
            return;

        lastTfStamp_ = currentStamp;

        geometry_msgs::msg::TransformStamped transformStamped;

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
            geometry_msgs::msg::TransformStamped T_l_b;
            try
            {
                // try to get tf at timestamp of message
                T_l_b = tfBuffer_.lookupTransform(
                    loc.child_frame_id, settings_.local_frame_id, loc.header.stamp);
            } catch (const tf2::TransformException& ex)
            {
                try
                {
                    RCLCPP_INFO_STREAM_THROTTLE(
                        this->get_logger(), *this->get_clock(), 10000,
                        ": No transform for insertion of local frame at t="
                            << std::to_string(currentStamp)
                            << ". Exception: " << std::string(ex.what()));
                    // try to get latest tf
                    T_l_b = tfBuffer_.lookupTransform(loc.child_frame_id,
                                                      settings_.local_frame_id,
                                                      rclcpp::Time(0));
                } catch (const tf2::TransformException& ex)
                {
                    RCLCPP_WARN_STREAM_THROTTLE(
                        this->get_logger(), *this->get_clock(), 10000,
                        ": No most recent transform for insertion of local frame. Exception: "
                            << std::string(ex.what()));
                    return;
                }
            }

            // T_l_g = T_b_l^-1 * T_b_g;
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
    void callbackOdometry(const nav_msgs::msg::Odometry::ConstSharedPtr odo)
    {
        Timestamp stamp = timestampFromRos(odo->header.stamp);

        processTwist(stamp, odo->twist);
    }

    void callbackTwist(const TwistWithCovarianceStampedMsg::ConstSharedPtr twist)
    {
        Timestamp stamp = timestampFromRos(twist->header.stamp);

        processTwist(stamp, twist->twist);
    }

    void processTwist(Timestamp stamp,
                      const geometry_msgs::msg::TwistWithCovariance& twist)
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
    //! Settings
    Settings settings_;
    //! Send velocity to communication layer (virtual)
    virtual void sendVelocity(const std::string& velNmea) = 0;

private:
    //! Map of topics and publishers
    std::unordered_map<std::string, std::any> topicMap_;
    //! Publisher queue size
    uint32_t queueSize_ = 1;
    //! Transform publisher
    tf2_ros::TransformBroadcaster tf2Publisher_;
    //! Odometry subscriber
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometrySubscriber_;
    //! Twist subscriber
    rclcpp::Subscription<TwistWithCovarianceStampedMsg>::SharedPtr twistSubscriber_;
    //! Last tf stamp
    Timestamp lastTfStamp_ = 0;
    //! tf buffer
    tf2_ros::Buffer tfBuffer_;
    // tf listener
    tf2_ros::TransformListener tfListener_;
    // Capabilities of Rx
    Capabilities capabilities_;
};