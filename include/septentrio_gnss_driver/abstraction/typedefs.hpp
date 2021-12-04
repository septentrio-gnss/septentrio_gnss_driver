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
// ROS msg includes
#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <gps_common/GPSFix.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/TimeReference.h>
// GNSS msg includes
#include <septentrio_gnss_driver/AttCovEuler.h>
#include <septentrio_gnss_driver/AttEuler.h>
#include <septentrio_gnss_driver/PVTCartesian.h>
#include <septentrio_gnss_driver/PVTGeodetic.h>
#include <septentrio_gnss_driver/PosCovCartesian.h>
#include <septentrio_gnss_driver/PosCovGeodetic.h>
#include <septentrio_gnss_driver/VelCovGeodetic.h>
// NMEA msg includes
#include <septentrio_gnss_driver/Gpgga.h>
#include <septentrio_gnss_driver/Gpgsa.h>
#include <septentrio_gnss_driver/Gpgsv.h>
#include <septentrio_gnss_driver/Gprmc.h>
// INS msg includes
#include <septentrio_gnss_driver/INSNavCart.h>
#include <septentrio_gnss_driver/INSNavGeod.h>
#include <septentrio_gnss_driver/IMUSetup.h>
#include <septentrio_gnss_driver/VelSensorSetup.h>
#include <septentrio_gnss_driver/ExtEventINSNavGeod.h>
#include <septentrio_gnss_driver/ExtEventINSNavCart.h>
#include <septentrio_gnss_driver/ExtSensorMeas.h>

// Timestamp in nanoseconds (Unix epoch)
typedef uint64_t  Timestamp;
// ROS timestamp
typedef ros::Time TimestampRos;

// ROS messages
typedef diagnostic_msgs::DiagnosticArray            DiagnosticArrayMsg;
typedef diagnostic_msgs::DiagnosticArrayPtr         DiagnosticArrayMsgPtr;
typedef diagnostic_msgs::DiagnosticStatus           DiagnosticStatusMsg;
typedef diagnostic_msgs::DiagnosticStatusPtr        DiagnosticStatusMsgPtr;
typedef geometry_msgs::Quaternion                   QuaternionMsg;
typedef geometry_msgs::PoseWithCovarianceStamped    PoseWithCovarianceStampedMsg;
typedef geometry_msgs::PoseWithCovarianceStampedPtr PoseWithCovarianceStampedMsgPtr;
typedef gps_common::GPSFix                          GPSFixMsg;
typedef gps_common::GPSFixPtr                       GPSFixMsgPtr;
typedef gps_common::GPSStatus                       GPSStatusMsg;
typedef sensor_msgs::NavSatFix                      NavSatFixMsg;
typedef sensor_msgs::NavSatFixPtr                   NavSatFixMsgPtr;
typedef sensor_msgs::NavSatStatus                   NavSatStatusMsg;
typedef sensor_msgs::TimeReference                  TimeReferenceMsg;
typedef sensor_msgs::TimeReferencePtr               TimeReferenceMsgPtr;

// Septentrio GNSS SBF messages
typedef septentrio_gnss_driver::AttCovEuler        AttCovEulerMsg;
typedef septentrio_gnss_driver::AttCovEulerPtr     AttCovEulerMsgPtr;
typedef septentrio_gnss_driver::AttEuler           AttEulerMsg;
typedef septentrio_gnss_driver::AttEulerPtr        AttEulerMsgPtr;
typedef septentrio_gnss_driver::PVTCartesian       PVTCartesianMsg;
typedef septentrio_gnss_driver::PVTCartesianPtr    PVTCartesianMsgPtr;
typedef septentrio_gnss_driver::PVTGeodetic        PVTGeodeticMsg;
typedef septentrio_gnss_driver::PVTGeodeticPtr     PVTGeodeticMsgPtr;
typedef septentrio_gnss_driver::PosCovCartesian    PosCovCartesianMsg;
typedef septentrio_gnss_driver::PosCovCartesianPtr PosCovCartesianMsgPtr;
typedef septentrio_gnss_driver::PosCovGeodetic     PosCovGeodeticMsg;
typedef septentrio_gnss_driver::PosCovGeodeticPtr  PosCovGeodeticMsgPtr;
typedef septentrio_gnss_driver::VelCovGeodetic     VelCovGeodeticMsg;
typedef septentrio_gnss_driver::VelCovGeodeticPtr  VelCovGeodeticMsgPtr;

// Septentrio GNSS NMEA messages
typedef septentrio_gnss_driver::Gpgga    GpggaMsg;
typedef septentrio_gnss_driver::GpggaPtr GpggaMsgPtr;
typedef septentrio_gnss_driver::Gpgsa    GpgsaMsg;
typedef septentrio_gnss_driver::GpgsaPtr GpgsaMsgPtr;
typedef septentrio_gnss_driver::Gpgsv    GpgsvMsg;
typedef septentrio_gnss_driver::GpgsvPtr GpgsvMsgPtr;
typedef septentrio_gnss_driver::Gprmc    GprmcMsg;
typedef septentrio_gnss_driver::GprmcPtr GprmcMsgPtr;

// Septentrio INS+GNSS SBF messages
typedef septentrio_gnss_driver::INSNavCart            INSNavCartMsg;
typedef septentrio_gnss_driver::INSNavCartPtr         INSNavCartMsgPtr;
typedef septentrio_gnss_driver::INSNavGeod            INSNavGeodMsg;
typedef septentrio_gnss_driver::INSNavGeodPtr         INSNavGeodMsgPtr;
typedef septentrio_gnss_driver::IMUSetup              IMUSetupMsg;
typedef septentrio_gnss_driver::IMUSetupPtr           IMUSetupMsgPtr;
typedef septentrio_gnss_driver::VelSensorSetup        VelSensorSetupMsg;
typedef septentrio_gnss_driver::VelSensorSetupPtr     VelSensorSetupMsgPtr;
typedef septentrio_gnss_driver::ExtEventINSNavGeod    ExtEventINSNavGeodMsg;
typedef septentrio_gnss_driver::ExtEventINSNavGeodPtr ExtEventINSNavGeodMsgPtr;
typedef septentrio_gnss_driver::ExtEventINSNavCart    ExtEventINSNavCartMsg;
typedef septentrio_gnss_driver::ExtEventINSNavCartPtr ExtEventINSNavCartMsgPtr;
typedef septentrio_gnss_driver::ExtSensorMeas         ExtSensorMeasMsg;
typedef septentrio_gnss_driver::ExtSensorMeasPtr      ExtSensorMeasMsgPtr;

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
     * @brief Checks whether the parameter is in the given range
     * @param[in] val The value to check
     * @param[in] min The minimum for this value
     * @param[in] max The maximum for this value
     * @param[in] name The name of the parameter
     * @throws std::runtime_error if the parameter is out of bounds
     */
    template <typename V, typename T>
    void checkRange(V val, T min, T max, std::string name)
    {
        if (val < min || val > max)
        {
            std::stringstream ss;
            ss << "Invalid settings: " << name << " must be in range [" << min
               << ", " << max << "].";
            throw std::runtime_error(ss.str());
        }
    } 

    /**
     * @brief Gets an integer or unsigned integer value from the parameter server
     * @param[in] key The key to be used in the parameter server's dictionary
     * @param[out] u Storage for the retrieved value, of type U, which can be either
     * unsigned int or int
     * @return True if found and valid, false if not
     */
    template <typename U>
    bool getROSInt(const std::string& key, U& u)
    {
        int param;
        if (!pNh_->getParam(key, param))
            return false;
        U min = std::numeric_limits<U>::lowest();
        U max = std::numeric_limits<U>::max();
        try
        {
            checkRange((U)param, min, max, key);
        } catch (std::runtime_error& e)
        {
            ROS_ERROR_STREAM(e.what());
            return false;
        }
        u = (U)param;
        return true;
    }

    /**
     * @brief Gets an integer or unsigned integer value from the parameter server
     * @param[in] key The key to be used in the parameter server's dictionary
     * @param[out] u Storage for the retrieved value, of type U, which can be either
     * unsigned int or int
     * @param[in] default_val Value to use if the server doesn't contain this
     * parameter
     */
    template <typename U>
    bool getIntParam(const std::string& key, U& u, U default_val)
    {
        if (!getROSInt(key, u))
        {
            u = default_val;
            return false;
        }
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

protected:
    //! Node handle pointer
    std::shared_ptr<ros::NodeHandle> pNh_;    

private:
    //! Map of topics and publishers
    std::unordered_map<std::string, ros::Publisher> topicMap_;
    //! Publisher queue size
    uint32_t queueSize_ = 1;
};

#endif // Typedefs_HPP