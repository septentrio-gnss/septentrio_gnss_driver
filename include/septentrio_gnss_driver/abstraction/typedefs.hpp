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
// ROS includes
#include <ros/ros.h>
// tf2 includes
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
// ROS msg includes
#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <gps_common/GPSFix.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/TimeReference.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
// GNSS msg includes
#include <septentrio_gnss_driver/BlockHeader.h>
#include <septentrio_gnss_driver/MeasEpoch.h>
#include <septentrio_gnss_driver/MeasEpochChannelType1.h>
#include <septentrio_gnss_driver/MeasEpochChannelType2.h>
#include <septentrio_gnss_driver/AttCovEuler.h>
#include <septentrio_gnss_driver/AttEuler.h>
#include <septentrio_gnss_driver/PVTCartesian.h>
#include <septentrio_gnss_driver/PVTGeodetic.h>
#include <septentrio_gnss_driver/PosCovCartesian.h>
#include <septentrio_gnss_driver/PosCovGeodetic.h>
#include <septentrio_gnss_driver/VelCovCartesian.h>
#include <septentrio_gnss_driver/VelCovGeodetic.h>
// NMEA msg includes
#include <nmea_msgs/Gpgga.h>
#include <nmea_msgs/Gpgsa.h>
#include <nmea_msgs/Gpgsv.h>
#include <nmea_msgs/Gprmc.h>
// INS msg includes
#include <septentrio_gnss_driver/INSNavCart.h>
#include <septentrio_gnss_driver/INSNavGeod.h>
#include <septentrio_gnss_driver/IMUSetup.h>
#include <septentrio_gnss_driver/VelSensorSetup.h>
#include <septentrio_gnss_driver/ExtSensorMeas.h>

// Timestamp in nanoseconds (Unix epoch)
typedef uint64_t  Timestamp;
// ROS timestamp
typedef ros::Time TimestampRos;

// ROS messages
typedef diagnostic_msgs::DiagnosticArray            DiagnosticArrayMsg;
typedef diagnostic_msgs::DiagnosticStatus           DiagnosticStatusMsg;
typedef geometry_msgs::Quaternion                   QuaternionMsg;
typedef geometry_msgs::PoseWithCovarianceStamped    PoseWithCovarianceStampedMsg;
typedef geometry_msgs::TransformStamped             TransformStampedMsg;
typedef gps_common::GPSFix                          GPSFixMsg;
typedef gps_common::GPSStatus                       GPSStatusMsg;
typedef sensor_msgs::NavSatFix                      NavSatFixMsg;
typedef sensor_msgs::NavSatStatus                   NavSatStatusMsg;
typedef sensor_msgs::TimeReference                  TimeReferenceMsg;
typedef sensor_msgs::Imu                            ImuMsg;
typedef nav_msgs::Odometry                          LocalizationUtmMsg;

// Septentrio GNSS SBF messages
typedef septentrio_gnss_driver::BlockHeader           BlockHeaderMsg;
typedef septentrio_gnss_driver::MeasEpoch             MeasEpochMsg;
typedef septentrio_gnss_driver::MeasEpochChannelType1 MeasEpochChannelType1Msg;
typedef septentrio_gnss_driver::MeasEpochChannelType2 MeasEpochChannelType2Msg;
typedef septentrio_gnss_driver::AttCovEuler           AttCovEulerMsg;
typedef septentrio_gnss_driver::AttEuler              AttEulerMsg;
typedef septentrio_gnss_driver::PVTCartesian          PVTCartesianMsg;
typedef septentrio_gnss_driver::PVTGeodetic           PVTGeodeticMsg;
typedef septentrio_gnss_driver::PosCovCartesian       PosCovCartesianMsg;
typedef septentrio_gnss_driver::PosCovGeodetic        PosCovGeodeticMsg;
typedef septentrio_gnss_driver::VelCovCartesian       VelCovCartesianMsg;
typedef septentrio_gnss_driver::VelCovGeodetic        VelCovGeodeticMsg;

// NMEA messages
typedef nmea_msgs::Gpgga    GpggaMsg;
typedef nmea_msgs::Gpgsa    GpgsaMsg;
typedef nmea_msgs::Gpgsv    GpgsvMsg;
typedef nmea_msgs::Gprmc    GprmcMsg;

// Septentrio INS+GNSS SBF messages
typedef septentrio_gnss_driver::INSNavCart            INSNavCartMsg;
typedef septentrio_gnss_driver::INSNavGeod            INSNavGeodMsg;
typedef septentrio_gnss_driver::IMUSetup              IMUSetupMsg;
typedef septentrio_gnss_driver::VelSensorSetup        VelSensorSetupMsg;
typedef septentrio_gnss_driver::ExtSensorMeas         ExtSensorMeasMsg;

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
inline Timestamp timestampFromRos(const TimestampRos& tsr)
{
    return tsr.toNSec();
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
class ROSaicNodeBase
{
public:
    ROSaicNodeBase() :
    pNh_(new ros::NodeHandle("~"))
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
     * @param[in] name The key to be used in the parameter server's dictionary
     * @param[out] val Storage for the retrieved value, of type T
     * @param[in] defaultVal Value to use if the server doesn't contain this
     * parameter
     * @return True if it could be retrieved, false if not
     */
    template<typename T>
    bool param(const std::string& name, T& val, const T& defaultVal)
    {
        return pNh_->param(name, val, defaultVal);
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
            ROS_DEBUG_STREAM(ros::this_node::getName() << ": " << s);
            break;
        case LogLevel::INFO:
            ROS_INFO_STREAM(ros::this_node::getName() << ": " << s);
            break;
        case LogLevel::WARN:
            ROS_WARN_STREAM(ros::this_node::getName() << ": " << s);
            break;
        case LogLevel::ERROR:
            ROS_ERROR_STREAM(ros::this_node::getName() << ": " << s);
            break;
        case LogLevel::FATAL:
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
    Timestamp getTime()
    {
        return ros::Time::now().toNSec();
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
            it->second.publish(msg);
        }
        else
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
    void publishTf(const LocalizationUtmMsg& loc)
    {
        if (std::isnan(loc.pose.pose.orientation.w))
            return;
        geometry_msgs::TransformStamped transformStamped;
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

        tf2Publisher_.sendTransform(transformStamped);
    }

protected:
    //! Node handle pointer
    std::shared_ptr<ros::NodeHandle> pNh_;    

private:
    //! Map of topics and publishers
    std::unordered_map<std::string, ros::Publisher> topicMap_;
    //! Publisher queue size
    uint32_t queueSize_ = 1;
    //! Transform publisher
    tf2_ros::TransformBroadcaster tf2Publisher_;
};

#endif // Typedefs_HPP