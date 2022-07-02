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

#ifndef Typedefs_HPP
#define Typedefs_HPP

// std includes
#include <unordered_map>
#include <any>
// ROS includes
#include <rclcpp/rclcpp.hpp>
// tf2 includes
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#ifdef ROS2_VER_N250
    #include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#else
    #include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#endif
#include <tf2_eigen/tf2_eigen.h>
// ROS msg includes
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <gps_msgs/msg/gps_fix.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/time_reference.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
// GNSS msg includes
#include <septentrio_gnss_driver/msg/block_header.hpp>
#include <septentrio_gnss_driver/msg/meas_epoch.hpp>
#include <septentrio_gnss_driver/msg/meas_epoch_channel_type1.hpp>
#include <septentrio_gnss_driver/msg/meas_epoch_channel_type2.hpp>
#include <septentrio_gnss_driver/msg/att_cov_euler.hpp>
#include <septentrio_gnss_driver/msg/att_euler.hpp>
#include <septentrio_gnss_driver/msg/pvt_cartesian.hpp>
#include <septentrio_gnss_driver/msg/pvt_geodetic.hpp>
#include <septentrio_gnss_driver/msg/pos_cov_cartesian.hpp>
#include <septentrio_gnss_driver/msg/pos_cov_geodetic.hpp>
#include <septentrio_gnss_driver/msg/receiver_time.hpp>
#include <septentrio_gnss_driver/msg/vel_cov_cartesian.hpp>
#include <septentrio_gnss_driver/msg/vel_cov_geodetic.hpp>
// NMEA msg includes
#include <nmea_msgs/msg/gpgga.hpp>
#include <nmea_msgs/msg/gpgsa.hpp>
#include <nmea_msgs/msg/gpgsv.hpp>
#include <nmea_msgs/msg/gprmc.hpp>
// INS msg includes
#include <septentrio_gnss_driver/msg/ins_nav_cart.hpp>
#include <septentrio_gnss_driver/msg/ins_nav_geod.hpp>
#include <septentrio_gnss_driver/msg/imu_setup.hpp>
#include <septentrio_gnss_driver/msg/vel_sensor_setup.hpp>
#include <septentrio_gnss_driver/msg/ext_sensor_meas.hpp>

// Timestamp in nanoseconds (Unix epoch)
typedef uint64_t     Timestamp;
// ROS timestamp
typedef rclcpp::Time TimestampRos;

// ROS messages
typedef diagnostic_msgs::msg::DiagnosticArray         DiagnosticArrayMsg;
typedef diagnostic_msgs::msg::DiagnosticStatus        DiagnosticStatusMsg;
typedef geometry_msgs::msg::Quaternion                QuaternionMsg;
typedef geometry_msgs::msg::PoseWithCovarianceStamped PoseWithCovarianceStampedMsg;
typedef geometry_msgs::msg::TransformStamped          TransformStampedMsg;
typedef gps_msgs::msg::GPSFix                         GPSFixMsg;
typedef gps_msgs::msg::GPSStatus                      GPSStatusMsg;
typedef sensor_msgs::msg::NavSatFix                   NavSatFixMsg;
typedef sensor_msgs::msg::NavSatStatus                NavSatStatusMsg;
typedef sensor_msgs::msg::TimeReference               TimeReferenceMsg;
typedef sensor_msgs::msg::Imu                         ImuMsg;
typedef nav_msgs::msg::Odometry                       LocalizationUtmMsg;

// Septentrio GNSS SBF messages
typedef septentrio_gnss_driver::msg::BlockHeader           BlockHeaderMsg;
typedef septentrio_gnss_driver::msg::MeasEpoch             MeasEpochMsg;
typedef septentrio_gnss_driver::msg::MeasEpochChannelType1 MeasEpochChannelType1Msg;
typedef septentrio_gnss_driver::msg::MeasEpochChannelType2 MeasEpochChannelType2Msg;
typedef septentrio_gnss_driver::msg::AttCovEuler           AttCovEulerMsg;
typedef septentrio_gnss_driver::msg::AttEuler              AttEulerMsg;
typedef septentrio_gnss_driver::msg::PVTCartesian          PVTCartesianMsg;
typedef septentrio_gnss_driver::msg::PVTGeodetic           PVTGeodeticMsg;
typedef septentrio_gnss_driver::msg::PosCovCartesian       PosCovCartesianMsg;
typedef septentrio_gnss_driver::msg::PosCovGeodetic        PosCovGeodeticMsg;
typedef septentrio_gnss_driver::msg::ReceiverTime          ReceiverTimeMsg;
typedef septentrio_gnss_driver::msg::VelCovCartesian       VelCovCartesianMsg;
typedef septentrio_gnss_driver::msg::VelCovGeodetic        VelCovGeodeticMsg;

// NMEA message
typedef nmea_msgs::msg::Gpgga GpggaMsg;
typedef nmea_msgs::msg::Gpgsa GpgsaMsg;
typedef nmea_msgs::msg::Gpgsv GpgsvMsg;
typedef nmea_msgs::msg::Gprmc GprmcMsg;;

// Septentrio INS+GNSS SBF messages
typedef septentrio_gnss_driver::msg::INSNavCart     INSNavCartMsg;
typedef septentrio_gnss_driver::msg::INSNavGeod     INSNavGeodMsg;
typedef septentrio_gnss_driver::msg::IMUSetup       IMUSetupMsg;
typedef septentrio_gnss_driver::msg::VelSensorSetup VelSensorSetupMsg;
typedef septentrio_gnss_driver::msg::ExtSensorMeas  ExtSensorMeasMsg;

/**
 * @brief Convert nsec timestamp to ROS timestamp
 * @param[in] ts timestamp in nanoseconds (Unix epoch)
 * @return ROS timestamp
 */
inline TimestampRos timestampToRos(Timestamp ts)
{
   return TimestampRos(ts);
}

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
enum LogLevel
{
    DEBUG,
    INFO,
    WARN,
    ERROR,
    FATAL
};

/**
 * @class ROSaicNodeBase
 * @brief This class is the base class for abstraction
 */
class ROSaicNodeBase : public rclcpp::Node
{
public:
    ROSaicNodeBase(const rclcpp::NodeOptions &options) :
    Node("septentrio_gnss", options),
    tf2Publisher_(this),
    tfBuffer_(this->get_clock()),
    tfListener_(tfBuffer_)
    {}

    virtual ~ROSaicNodeBase(){}

    /**
     * @brief Gets an integer or unsigned integer value from the parameter server
     * @param[in] name The key to be used in the parameter server's dictionary
     * @param[out] val Storage for the retrieved value, of type U, which can be either
     * unsigned int or int
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
    template<typename T>
    bool param(const std::string& name, T& val, const T& defaultVal)
    {
        if (this->has_parameter(name))
            this->undeclare_parameter(name);

        try
        {
            val = this->declare_parameter<T>(name, defaultVal);
        }
        catch (std::runtime_error& e)
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
    void log(LogLevel logLevel, const std::string& s)
    {
        switch (logLevel)
        {
        case LogLevel::DEBUG:
            RCLCPP_DEBUG_STREAM(this->get_logger(), s);
            break;
        case LogLevel::INFO:
            RCLCPP_INFO_STREAM(this->get_logger(), s);
            break;
        case LogLevel::WARN:
            RCLCPP_WARN_STREAM(this->get_logger(), s);
            break;
        case LogLevel::ERROR:
            RCLCPP_ERROR_STREAM(this->get_logger(), s);
            break;
        case LogLevel::FATAL:
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
    Timestamp getTime()
    {
        return this->now().nanoseconds();
    }    

    /**
     * @brief Publishing function
     * @param[in] topic String of topic
     * @param[in] msg ROS message to be published
     */
    template <typename M>
    void publishMessage(const std::string& topic, const M& msg)
    {
        auto it = topicMap_.find(topic);
        if (it != topicMap_.end())
        {
            typename rclcpp::Publisher<M>::SharedPtr ptr = std::any_cast<typename rclcpp::Publisher<M>::SharedPtr>(it->second);
            ptr->publish(msg);
        }
        else
        {
            typename rclcpp::Publisher<M>::SharedPtr pub = this->create_publisher<M>(topic, queueSize_);
            topicMap_.insert(std::make_pair(topic, pub));
            pub->publish(msg);
        }
    }

    /**
     * @brief Publishing function for tf
     * @param[in] msg ROS localization message to be converted to tf
     */
    void publishTf(const LocalizationUtmMsg& loc)
    {
        if (std::isnan(loc.pose.pose.orientation.w))        
            return;

        Timestamp currentStamp = timestampFromRos(loc.header.stamp);
        if (lastTfStamp_ == currentStamp)
            return;

        lastTfStamp_ = currentStamp;

        geometry_msgs::msg::TransformStamped transformStamped;

        transformStamped.header.stamp            = loc.header.stamp;
        transformStamped.header.frame_id         = loc.header.frame_id;
        transformStamped.child_frame_id          = loc.child_frame_id;
        transformStamped.transform.translation.x = loc.pose.pose.position.x;
        transformStamped.transform.translation.y = loc.pose.pose.position.y;
        transformStamped.transform.translation.z = loc.pose.pose.position.z;
        transformStamped.transform.rotation.x    = loc.pose.pose.orientation.x;
        transformStamped.transform.rotation.y    = loc.pose.pose.orientation.y;
        transformStamped.transform.rotation.z    = loc.pose.pose.orientation.z;
        transformStamped.transform.rotation.w    = loc.pose.pose.orientation.w;
        
        if (insert_local_frame_)
        {
            geometry_msgs::msg::TransformStamped T_b_l;
            try
            {
                // try to get tf at timestamp of message
                T_b_l = tfBuffer_.lookupTransform(local_frame_id_, loc.child_frame_id, loc.header.stamp);
            }
            catch (const tf2::TransformException& ex)
            {
                try
                {
                    // TODO throttled debug message;
                    // try to get latest tf
                    T_b_l = tfBuffer_.lookupTransform(local_frame_id_, loc.child_frame_id, loc.header.stamp);
                }
                catch (const tf2::TransformException& ex)
                {
                    // TODO throttled error message;
                    return;
                }
            }
            
            // T_l_g = T_b_l.inverse() * A_b_g;
            transformStamped = tf2::eigenToTransform(tf2::transformToEigen(T_b_l).inverse() *
                                                     tf2::transformToEigen(transformStamped));
            transformStamped.header.stamp    = loc.header.stamp;
            transformStamped.header.frame_id = loc.header.frame_id;
            transformStamped.child_frame_id  = local_frame_id_;
        }
       
        tf2Publisher_.sendTransform(transformStamped); 
    }

protected:
    //! Wether local frame should be inserted into tf
    bool insert_local_frame_ = false;
    //! Frame id of the local frame to be inserted
    std::string local_frame_id_;

private:
    //! Map of topics and publishers
    std::unordered_map<std::string, std::any> topicMap_;
    //! Publisher queue size
    uint32_t queueSize_ = 1;
    //! Transform publisher
    tf2_ros::TransformBroadcaster tf2Publisher_;
    //! Last tf stamp
    Timestamp lastTfStamp_;
    //! tf buffer
    tf2_ros::Buffer tfBuffer_;
	// tf listener
    tf2_ros::TransformListener tfListener_;
};

#endif // Typedefs_HPP