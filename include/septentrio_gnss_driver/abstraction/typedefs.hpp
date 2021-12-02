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
// ROS msg includes
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <geometry_msgs/msg/quaternion.h>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <gps_msgs/msg/gps_fix.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/time_reference.hpp>
// GNSS msg includes
#include <septentrio_gnss_driver/msg/att_cov_euler.hpp>
#include <septentrio_gnss_driver/msg/att_euler.hpp>
#include <septentrio_gnss_driver/msg/pvt_cartesian.hpp>
#include <septentrio_gnss_driver/msg/pvt_geodetic.hpp>
#include <septentrio_gnss_driver/msg/pos_cov_cartesian.hpp>
#include <septentrio_gnss_driver/msg/pos_cov_geodetic.hpp>
#include <septentrio_gnss_driver/msg/vel_cov_geodetic.hpp>
// NMEA msg includes
#include <septentrio_gnss_driver/msg/gpgga.hpp>
#include <septentrio_gnss_driver/msg/gpgsa.hpp>
#include <septentrio_gnss_driver/msg/gpgsv.hpp>
#include <septentrio_gnss_driver/msg/gprmc.hpp>
// INS msg includes
#include <septentrio_gnss_driver/msg/ins_nav_cart.hpp>
#include <septentrio_gnss_driver/msg/ins_nav_geod.hpp>
#include <septentrio_gnss_driver/msg/imu_setup.hpp>
#include <septentrio_gnss_driver/msg/vel_sensor_setup.hpp>
#include <septentrio_gnss_driver/msg/ext_event_ins_nav_geod.hpp>
#include <septentrio_gnss_driver/msg/ext_event_ins_nav_cart.hpp>
#include <septentrio_gnss_driver/msg/ext_sensor_meas.hpp>

typedef uint64_t     Timestamp;
typedef rclcpp::Time TimestampRos;

typedef diagnostic_msgs::msg::DiagnosticArray                    DiagnosticArrayMsg;
typedef diagnostic_msgs::msg::DiagnosticArray::SharedPtr         DiagnosticArrayMsgPtr;
typedef diagnostic_msgs::msg::DiagnosticStatus                   DiagnosticStatusMsg;
typedef diagnostic_msgs::msg::DiagnosticStatus::SharedPtr        DiagnosticStatusMsgPtr;
typedef geometry_msgs::msg::Quaternion                           QuaternionMsg;
typedef geometry_msgs::msg::PoseWithCovarianceStamped            PoseWithCovarianceStampedMsg;
typedef geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr PoseWithCovarianceStampedMsgPtr;
typedef gps_msgs::msg::GPSFix                                    GPSFixMsg;
typedef gps_msgs::msg::GPSFix::SharedPtr                         GPSFixMsgPtr;
typedef gps_msgs::msg::GPSStatus                                 GPSStatusMsg;
typedef sensor_msgs::msg::NavSatFix                              NavSatFixMsg;
typedef sensor_msgs::msg::NavSatFix::SharedPtr                   NavSatFixMsgPtr;
typedef sensor_msgs::msg::NavSatStatus                           NavSatStatusMsg;
typedef sensor_msgs::msg::TimeReference                          TimeReferenceMsg;
typedef sensor_msgs::msg::TimeReference::SharedPtr               TimeReferenceMsgPtr;

typedef septentrio_gnss_driver::msg::AttCovEuler                AttCovEulerMsg;
typedef septentrio_gnss_driver::msg::AttCovEuler::SharedPtr     AttCovEulerMsgPtr;
typedef septentrio_gnss_driver::msg::AttEuler                   AttEulerMsg;
typedef septentrio_gnss_driver::msg::AttEuler::SharedPtr        AttEulerMsgPtr;
typedef septentrio_gnss_driver::msg::PVTCartesian               PVTCartesianMsg;
typedef septentrio_gnss_driver::msg::PVTCartesian::SharedPtr    PVTCartesianMsgPtr;
typedef septentrio_gnss_driver::msg::PVTGeodetic                PVTGeodeticMsg;
typedef septentrio_gnss_driver::msg::PVTGeodetic::SharedPtr     PVTGeodeticMsgPtr;
typedef septentrio_gnss_driver::msg::PosCovCartesian            PosCovCartesianMsg;
typedef septentrio_gnss_driver::msg::PosCovCartesian::SharedPtr PosCovCartesianMsgPtr;
typedef septentrio_gnss_driver::msg::PosCovGeodetic             PosCovGeodeticMsg;
typedef septentrio_gnss_driver::msg::PosCovGeodetic::SharedPtr  PosCovGeodeticMsgPtr;
typedef septentrio_gnss_driver::msg::VelCovGeodetic             VelCovGeodeticMsg;
typedef septentrio_gnss_driver::msg::VelCovGeodetic::SharedPtr  VelCovGeodeticMsgPtr;

typedef septentrio_gnss_driver::msg::Gpgga            GpggaMsg;
typedef septentrio_gnss_driver::msg::Gpgga::SharedPtr GpggaMsgPtr;
typedef septentrio_gnss_driver::msg::Gpgsa            GpgsaMsg;
typedef septentrio_gnss_driver::msg::Gpgsa::SharedPtr GpgsaMsgPtr;
typedef septentrio_gnss_driver::msg::Gpgsv            GpgsvMsg;
typedef septentrio_gnss_driver::msg::Gpgsv::SharedPtr GpgsvMsgPtr;
typedef septentrio_gnss_driver::msg::Gprmc            GprmcMsg;
typedef septentrio_gnss_driver::msg::Gprmc::SharedPtr GprmcMsgPtr;

typedef septentrio_gnss_driver::msg::INSNavCart                    INSNavCartMsg;
typedef septentrio_gnss_driver::msg::INSNavCart::SharedPtr         INSNavCartMsgPtr;
typedef septentrio_gnss_driver::msg::INSNavGeod                    INSNavGeodMsg;
typedef septentrio_gnss_driver::msg::INSNavGeod::SharedPtr         INSNavGeodMsgPtr;
typedef septentrio_gnss_driver::msg::IMUSetup                      IMUSetupMsg;
typedef septentrio_gnss_driver::msg::IMUSetup::SharedPtr           IMUSetupMsgPtr;
typedef septentrio_gnss_driver::msg::VelSensorSetup                VelSensorSetupMsg;
typedef septentrio_gnss_driver::msg::VelSensorSetup::SharedPtr     VelSensorSetupMsgPtr;
typedef septentrio_gnss_driver::msg::ExtEventINSNavGeod            ExtEventINSNavGeodMsg;
typedef septentrio_gnss_driver::msg::ExtEventINSNavGeod::SharedPtr ExtEventINSNavGeodMsgPtr;
typedef septentrio_gnss_driver::msg::ExtEventINSNavCart            ExtEventINSNavCartMsg;
typedef septentrio_gnss_driver::msg::ExtEventINSNavCart::SharedPtr ExtEventINSNavCartMsgPtr;
typedef septentrio_gnss_driver::msg::ExtSensorMeas                 ExtSensorMeasMsg;
typedef septentrio_gnss_driver::msg::ExtSensorMeas::SharedPtr      ExtSensorMeasMsgPtr;

/**
 * @brief Convert nsec timestamp to ROS timestamp
 * @param[in] ts timestamp in nanoseconds
 * @return ROS timestamp
 */
inline TimestampRos timestampToRos(Timestamp ts)
{
   return TimestampRos(ts);
}

/**
 * @brief Convert ROS timestamp to nsec timestamp 
 * @param[in] ts ROS timestamp
 * @return timestamp in nanoseconds
 */
inline Timestamp timestampFromRos(const TimestampRos& tsr)
{
    return tsr.nanoseconds();
}

enum LogLevel
{
    DEBUG,
    INFO,
    ERROR,
    FATAL
};

class ROSaicNodeBase : public rclcpp::Node
{
public:
    ROSaicNodeBase(const rclcpp::NodeOptions &options) :
    Node("septentrio_gnss", options)
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
        if (!this->param(key, param, -1))
            return false;
        U min = std::numeric_limits<U>::lowest();
        U max = std::numeric_limits<U>::max();
        try
        {
            checkRange((U)param, min, max, key);
        } catch (std::runtime_error& e)
        {
            RCLCPP_ERROR_STREAM(this->get_logger(), e.what());
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

private:
    //! Map of topics and publishers
    std::unordered_map<std::string, std::any> topicMap_;
    //! Publisher queue size
    uint32_t queueSize_ = 1;
};

#endif // Typedefs_HPP