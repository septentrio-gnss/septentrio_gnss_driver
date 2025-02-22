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

// C++ library includes
#include <cmath>   // C++ header, corresponds to <math.h> in C
#include <cstdint> // C++ header, corresponds to <stdint.h> in C
#include <ctime>   // C++ header, corresponds to <time.h> in C
#include <string>  // C++ header, corresponds to <string.h> in C
// Eigen Includes
#include <Eigen/Core>
#include <Eigen/LU>
// Boost includes
#include <boost/math/constants/constants.hpp>
// ROS includes
#ifdef ROS2
#include <septentrio_gnss_driver/abstraction/typedefs.hpp>
#endif
#ifdef ROS1
#include <septentrio_gnss_driver/abstraction/typedefs_ros1.hpp>
#endif

/**
 * @file parsing_utilities.hpp
 * @brief Declares utility functions used when parsing messages
 * @date 17/08/20
 */

namespace parsing_utilities {

    const double pihalf = boost::math::constants::half_pi<double>();

    /***********************************************************************
     * Square value
     **********************************************************************/
    template <class T>
    [[nodiscard]] inline T square(T val)
    {
        return val * val;
    }

    /***********************************************************************
     * Convert degrees to radians
     **********************************************************************/
    template <class T>
    [[nodiscard]] inline T deg2rad(T deg)
    {
        return deg * boost::math::constants::degree<T>();
    }

    /***********************************************************************
     * Convert degrees^2 to radians^2
     **********************************************************************/
    template <class T>
    [[nodiscard]] inline T deg2radSq(T deg)
    {
        return deg * boost::math::constants::degree<T>() *
               boost::math::constants::degree<T>();
    }

    /***********************************************************************
     * Convert radians to degree
     **********************************************************************/
    template <class T>
    [[nodiscard]] inline T rad2deg(T rad)
    {
        return rad * boost::math::constants::radian<T>();
    }

    /***********************************************************************
     * Convert Euler angles to rotation matrix
     **********************************************************************/
    [[nodiscard]] inline Eigen::Matrix3d rpyToRot(double roll, double pitch,
                                                  double yaw)
    {
        Eigen::Matrix3d M;
        double sa, ca, sb, cb, sc, cc;
        sa = std::sin(roll);
        ca = std::cos(roll);
        sb = std::sin(pitch);
        cb = std::cos(pitch);
        sc = std::sin(yaw);
        cc = std::cos(yaw);

        M << cb * cc, -ca * sc + sa * sb * cc, sc * sa + ca * sb * cc, cb * sc,
            ca * cc + sa * sb * sc, -sa * cc + ca * sb * sc, -sb, sa * cb, ca * cb;
        return M;
    }

    /**
     * @brief Wraps an angle between -180 and 180 degrees
     * @param[in] angle The angle to be wrapped
     */
    [[nodiscard]] inline double wrapAngle180to180(double angle)
    {
        return std::remainder(angle, 360.0);
    }

    /**
     * @brief Converts an 8-byte-buffer into a double
     * @param[in] buffer A pointer to a buffer containing 8 bytes of data
     * @return The double extracted from the data in the buffer
     */
    [[nodiscard]] double parseDouble(const uint8_t* buffer);

    /**
     * @brief Interprets the contents of "string" as a floating point number of type
     * double
     *
     * It stores the "string"'s value in "value" and returns whether or not all went
     * well.
     * @param[in] string The string whose content should be interpreted as a floating
     * point number
     * @param[out] value The double variable that should be overwritten by the
     * floating point number found in "string"
     * @return True if all went fine, false if not
     */
    [[nodiscard]] bool parseDouble(const std::string& string, double& value);

    /**
     * @brief Converts a 4-byte-buffer into a float
     * @param[in] buffer A pointer to a buffer containing 4 bytes of data
     * @return The float extracted from the data in the buffer
     */
    [[nodiscard]] float parseFloat(const uint8_t* buffer);

    /**
     * @brief Interprets the contents of "string" as a floating point number of type
     * float
     *
     * It stores the "string"'s value in "value" and returns whether or not all went
     * well.
     * @param[in] string The string whose content should be interpreted as a floating
     * point number
     * @param[out] value The float variable that should be overwritten by the
     * floating point number found in "string"
     * @return True if all went fine, false if not
     */
    [[nodiscard]] bool parseFloat(const std::string& string, float& value);

    /**
     * @brief Converts a 2-byte-buffer into a signed 16-bit integer
     * @param[in] buffer A pointer to a buffer containing 2 bytes of data
     * @return The int16_t value extracted from the data in the buffer
     */
    [[nodiscard]] int16_t parseInt16(const uint8_t* buffer);

    /**
     * @brief Interprets the contents of "string" as a integer number of type
     * int16_t.
     *
     * It stores the "string"'s value in "value" and returns whether or not all went
     * well.
     * @param[in] string The string whose content should be interpreted as an integer
     * number
     * @param[out] value The int16_t variable that should be overwritten by the
     * integer number found in "string"
     * @param[in] base The numerical base of the integer in the string, default being
     * 10
     * @return True if all went fine, false if not
     */
    [[nodiscard]] bool parseInt16(const std::string& string, int16_t& value,
                                  int32_t base = 10);

    /**
     * @brief Converts a 4-byte-buffer into a signed 32-bit integer
     * @param[in] buffer A pointer to a buffer containing 4 bytes of data
     * @return The int32_t value extracted from the data in the buffer
     */
    [[nodiscard]] int32_t parseInt32(const uint8_t* buffer);

    /**
     * @brief Interprets the contents of "string" as a integer number of type
     * int32_t.
     *
     * It stores the "string"'s value in "value" and returns whether or not all went
     * well.
     * @param[in] string The string whose content should be interpreted as an integer
     * number
     * @param[out] value The int32_t variable that should be overwritten by the
     * integer number found in "string"
     * @param[in] base The numerical base of the integer in the string, default being
     * 10
     * @return True if all went fine, false if not
     */
    [[nodiscard]] bool parseInt32(const std::string& string, int32_t& value,
                                  int32_t base = 10);

    /**
     * @brief Interprets the contents of "string" as a unsigned integer number of
     * type uint8_t.
     *
     * It stores the "string"'s value in "value" and returns whether or not all went
     * well.
     * @param[in] string The string whose content should be interpreted as an integer
     * number
     * @param[out] value The uint8_t variable that should be overwritten by the
     * integer number found in "string"
     * @param[in] base The numerical base of the integer in the string, default being
     * 10
     * @return True if all went fine, false if not
     */
    [[nodiscard]] bool parseUInt8(const std::string& string, uint8_t& value,
                                  int32_t base = 10);

    /**
     * @brief Converts a 2-byte-buffer into an unsigned 16-bit integer
     * @param[in] buffer A pointer to a buffer containing 2 bytes of data
     * @return The uint16_t value extracted from the data in the buffer
     */
    [[nodiscard]] uint16_t parseUInt16(const uint8_t* buffer);

    /**
     * @brief Interprets the contents of "string" as a unsigned integer number of
     * type uint16_t.
     *
     * It stores the "string"'s value in "value" and returns whether or not all went
     * well.
     * @param[in] string The string whose content should be interpreted as an integer
     * number
     * @param[out] value The uint16_t variable that should be overwritten by the
     * integer number found in "string"
     * @param[in] base The numerical base of the integer in the string, default being
     * 10
     * @return True if all went fine, false if not
     */
    [[nodiscard]] bool parseUInt16(const std::string& string, uint16_t& value,
                                   int32_t base = 10);

    /**
     * @brief Converts a 4-byte-buffer into an unsigned 32-bit integer
     * @param[in] buffer A pointer to a buffer containing 4 bytes of data
     * @return The uint32_t value extracted from the data in the buffer
     */
    [[nodiscard]] uint32_t parseUInt32(const uint8_t* buffer);

    /**
     * @brief Interprets the contents of "string" as a unsigned integer number of
     * type uint32_t.
     *
     * It stores the "string"'s value in "value" and returns whether or not all went
     * well.
     * @param[in] string The string whose content should be interpreted as an integer
     * number
     * @param[out] value The uint32_t variable that should be overwritten by the
     * integer number found in "string"
     * @param[in] base The numerical base of the integer in the string, default being
     * 10
     * @return True if all went fine, false if not
     */
    [[nodiscard]] bool parseUInt32(const std::string& string, uint32_t& value,
                                   int32_t base = 10);

    /**
     * @brief Converts UTC time from the without-colon-delimiter format to the
     * number-of-seconds-since-midnight format
     * @param[in] utc_double Rrepresents UTC time in the without-colon-delimiter
     * format
     * @return Represents UTC time in the number-of-seconds-since-midnight format
     */
    [[nodiscard]] double convertUTCDoubleToSeconds(double utc_double);

    /**
     * @brief Converts UTC time from the without-colon-delimiter format to Unix Epoch
     * time (a number-of-seconds-since-1970/01/01 format)
     *
     * Note that the type "std::time_t" is usually 32 bits, which also leads to the
     * "Year 2038 Problem".
     * @param[in] utc_double Represents UTC time in the without-colon-delimiter
     * format
     * @return The time_t variable representing Unix Epoch time
     */
    [[nodiscard]] std::time_t convertUTCtoUnix(double utc_double);

    /**
     * @brief Converts latitude or longitude from the DMS notation (in the
     * without-colon-delimiter format), to the pure degree notation
     *
     * Note that DMS stands for "Degrees, Minutes, Seconds".
     * @param[in] dms Represents latitude or longitude in the DMS notation (in the
     * without-colon-delimiter format)
     * @return Represents latitude or longitude in the pure degree notation
     */
    [[nodiscard]] double convertDMSToDegrees(double dms);

    /**
     * @brief Transforms Euler angles to a quaternion
     * @param[in] yaw Yaw, i.e. heading, about the z-axis [rad]
     * @param[in] pitch Pitch about the new y-axis [rad]
     * @param[in] roll Roll about the new y-axis [rad]
     * @return quaternion
     */
    //! The rotational sequence convention we adopt here (and Septentrio receivers'
    //! pitch, roll, yaw definition too) is the yaw-pitch-roll sequence, i.e. the
    //! 3-2-1 sequence: The body first does yaw around the z-axis, then pitches
    //! around the new y-axis and finally rolls around the new x-axis.
    [[nodiscard]] inline Eigen::Quaterniond
    convertEulerToQuaternion(double roll, double pitch, double yaw)
    {
        double cy = std::cos(yaw * 0.5);
        double sy = std::sin(yaw * 0.5);
        double cp = std::cos(pitch * 0.5);
        double sp = std::sin(pitch * 0.5);
        double cr = std::cos(roll * 0.5);
        double sr = std::sin(roll * 0.5);

        return Eigen::Quaterniond(
            cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy);
    }

    /**
     * @brief Transforms Euler angles to a QuaternionMsg
     * @param[in] yaw Yaw, i.e. heading, about the z-axis [rad]
     * @param[in] pitch Pitch about the new y-axis [rad]
     * @param[in] roll Roll about the new x-axis [rad]
     * @return ROS message representing a quaternion
     */
    [[nodiscard]] inline QuaternionMsg
    quaternionToQuaternionMsg(const Eigen::Quaterniond& q)
    {
        QuaternionMsg qm;

        qm.w = q.w();
        qm.x = q.x();
        qm.y = q.y();
        qm.z = q.z();

        return qm;
    }

    /**
     * @brief Convert Eigen quaternion to a QuaternionMsg
     * @param[in] q Eigen quaternion
     * @return ROS message representing a quaternion
     */
    [[nodiscard]] inline QuaternionMsg
    convertEulerToQuaternionMsg(double roll, double pitch, double yaw)
    {
        return quaternionToQuaternionMsg(convertEulerToQuaternion(roll, pitch, yaw));
    }

    inline void setQuaternionNaN(QuaternionMsg& quaternion)
    {
        quaternion.w = std::numeric_limits<double>::quiet_NaN();
        quaternion.x = std::numeric_limits<double>::quiet_NaN();
        quaternion.y = std::numeric_limits<double>::quiet_NaN();
        quaternion.z = std::numeric_limits<double>::quiet_NaN();
    }

    inline void setVector3NaN(Vector3Msg& v)
    {
        v.x = std::numeric_limits<double>::quiet_NaN();
        v.y = std::numeric_limits<double>::quiet_NaN();
        v.z = std::numeric_limits<double>::quiet_NaN();
    }

    /**
     * @brief Generates the quaternion to rotate from local ENU to ECEF
     * @param[in] lat gedoetic latitude [rad]
     * @param[in] lon geodetic longitude [rad]
     * @return quaternion
     */
    [[nodiscard]] inline Eigen::Quaterniond q_enu_ecef(double lat, double lon)
    {
        double sr = sin((pihalf - lat) / 2.0);
        double cr = cos((pihalf - lat) / 2.0);
        double sy = sin((lon + pihalf) / 2.0);
        double cy = cos((lon + pihalf) / 2.0);

        return Eigen::Quaterniond(cr * cy, sr * cy, sr * sy, cr * sy);
    }

    /**
     * @brief Generates the quaternion to rotate from local NED to ECEF
     * @param[in] lat geodetic latitude [rad]
     * @param[in] lon geodetic longitude [rad]
     * @return rotation matrix
     */
    [[nodiscard]] inline Eigen::Quaterniond q_ned_ecef(double lat, double lon)
    {
        double sp = sin((-lat - pihalf) / 2.0);
        double cp = cos((-lat - pihalf) / 2.0);
        double sy = sin(lon / 2.0);
        double cy = cos(lon / 2.0);

        return Eigen::Quaterniond(cp * cy, -sp * sy, sp * cy, cp * sy);
    }

    /**
     * @brief Generates the matrix to rotate from local ENU to ECEF
     * @param[in] lat geodetic latitude [rad]
     * @param[in] lon geodetic longitude [rad]
     * @return rotation matrix
     */
    [[nodiscard]] inline Eigen::Matrix3d R_enu_ecef(double lat, double lon)
    {
        Eigen::Matrix3d R;

        double sin_lat = sin(lat);
        double cos_lat = cos(lat);
        double sin_lon = sin(lon);
        double cos_lon = cos(lon);

        R(0, 0) = -sin_lon;
        R(0, 1) = -cos_lon * sin_lat;
        R(0, 2) = cos_lon * cos_lat;
        R(1, 0) = cos_lon;
        R(1, 1) = -sin_lon * sin_lat;
        R(1, 2) = sin_lon * cos_lat;
        R(2, 0) = 0.0;
        R(2, 1) = cos_lat;
        R(2, 2) = sin_lat;

        return R;
    }

    /**
     * @brief Generates the matrix to rotate from local NED to ECEF
     * @param[in] lat geodetic latitude [rad]
     * @param[in] lon geodetic longitude [rad]
     * @return rotation matrix
     */
    [[nodiscard]] inline Eigen::Matrix3d R_ned_ecef(double lat, double lon)
    {
        Eigen::Matrix3d R;

        double sin_lat = sin(lat);
        double cos_lat = cos(lat);
        double sin_lon = sin(lon);
        double cos_lon = cos(lon);

        R(0, 0) = -cos_lon * sin_lat;
        R(0, 1) = -sin_lon;
        R(0, 2) = -cos_lon * cos_lat;
        R(1, 0) = -sin_lon * sin_lat;
        R(1, 1) = cos_lon;
        R(1, 2) = -sin_lon * cos_lat;
        R(2, 0) = cos_lat;
        R(2, 1) = 0.0;
        R(2, 2) = -sin_lat;

        return R;
    }

    /**
     * @brief Transforms the input polling period [milliseconds] into a std::string
     * number that can be appended to either sec or msec for Rx commands
     * @param[in] period_user Polling period in milliseconds as specified by the
     * ROSaic user
     * @return Number to be appended to either msec, sec or min when sending commands
     * to the Rx
     */
    [[nodiscard]] std::string convertUserPeriodToRxCommand(uint32_t period_user);

    /**
     * @brief Get the CRC of the SBF message
     *
     * @param buffer A pointer to a buffer containing an SBF message
     * @return SBF message CRC
     */
    [[nodiscard]] uint16_t getCrc(const std::vector<uint8_t>& message);

    /**
     * @brief Get the ID of the SBF message
     *
     * @param buffer A pointer to a buffer containing an SBF message
     * @return SBF message ID
     */
    [[nodiscard]] uint16_t getId(const std::vector<uint8_t>& message);

    /**
     * @brief Get the length of the SBF message
     *
     * @param buffer A pointer to a buffer containing an SBF message
     * @return SBF message length
     */
    [[nodiscard]] uint16_t getLength(const std::vector<uint8_t>& message);

    /**
     * @brief Get the time of week in ms of the SBF message
     *
     * @param[in] buffer A pointer to a buffer containing an SBF message
     * @return SBF time of week in ms
     */
    [[nodiscard]] uint32_t getTow(const std::vector<uint8_t>& message);

    /**
     * @brief Get the GPS week counter of the SBF message
     *
     * @param buffer A pointer to a buffer containing an SBF message
     * @return SBF GPS week counter
     */
    [[nodiscard]] uint16_t getWnc(const std::vector<uint8_t>& message);

    inline double convertAutoCovariance(double val)
    {
        return std::isnan(val) ? -1.0 : deg2radSq(val);
    }

    inline double convertCovariance(double val)
    {
        return std::isnan(val) ? 0.0 : deg2radSq(val);
    }
} // namespace parsing_utilities