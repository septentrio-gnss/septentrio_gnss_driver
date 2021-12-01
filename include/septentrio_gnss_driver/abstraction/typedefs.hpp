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

// ROS includes
#include <ros/ros.h>
// ROS msg includes
#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
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
// INS msg includes
#include <septentrio_gnss_driver/INSNavCart.h>
#include <septentrio_gnss_driver/INSNavGeod.h>
#include <septentrio_gnss_driver/IMUSetup.h>
#include <septentrio_gnss_driver/VelSensorSetup.h>
#include <septentrio_gnss_driver/ExtEventINSNavGeod.h>
#include <septentrio_gnss_driver/ExtEventINSNavCart.h>
#include <septentrio_gnss_driver/ExtSensorMeas.h>

typedef ros::Time Timestamp;

typedef diagnostic_msgs::DiagnosticArray            DiagnosticArrayMsg;
typedef diagnostic_msgs::DiagnosticArrayPtr         DiagnosticArrayMsgPtr;
typedef diagnostic_msgs::DiagnosticStatus           DiagnosticStatusMsg;
typedef diagnostic_msgs::DiagnosticStatusPtr        DiagnosticStatusMsgPtr;
typedef geometry_msgs::PoseWithCovarianceStamped    PoseWithCovarianceStampedMsg;
typedef geometry_msgs::PoseWithCovarianceStampedPtr PoseWithCovarianceStampedMsgPtr;
typedef gps_common::GPSFix                          GPSFixMsg;
typedef gps_common::GPSFixPtr                       GPSFixMsgPtr;
typedef sensor_msgs::NavSatFix                      NavSatFixMsg;
typedef sensor_msgs::NavSatFixPtr                   NavSatFixMsgPtr;
typedef sensor_msgs::NavSatStatus                   NavSatStatusMsg;
typedef sensor_msgs::TimeReference                  TimeReferenceMsg;
typedef sensor_msgs::TimeReferencePtr               TimeReferenceMsgPtr;

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

enum LogLevel
{
    DEBUG,
    INFO,
    ERROR,
    FATAL
};

class ROSaicNodeBase
{
public:
    ROSaicNodeBase() :
    pNh_(new ros::NodeHandle("~"))
    {}

    virtual ~ROSaicNodeBase(){}

    /**
     * @brief Log function to provide abstraction of ROS loggers
     * @param[in] s String to log
     * @param[in] logLevel Log level
     */
    void log(LogLevel logLevel, const std::string& s)
    {
        switch (logLevel)
        {
        case LogLevel::DEBUG:
            ROS_DEBUG_STREAM(s);
            break;
        case LogLevel::INFO:
            ROS_INFO_STREAM(s);
            break;
        case LogLevel::ERROR:
            ROS_ERROR_STREAM(s);
            break;
        case LogLevel::FATAL:
            ROS_FATAL_STREAM(s);
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
        return ros::Time::now();
    }

protected:
    //! Node handle pointer
    std::shared_ptr<ros::NodeHandle> pNh_;
};

#endif // Typedefs_HPP