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

// ROSaic includes
#include <septentrio_gnss_driver/parsers/parsing_utilities.hpp>
#include <septentrio_gnss_driver/parsers/string_utilities.h>
// C++ library includes
#include <limits>
// Boost
#include <boost/spirit/include/qi_binary.hpp>

/**
 * @file parsing_utilities.cpp
 * @brief Declares utility functions used when parsing messages
 * @date 17/08/20
 */

namespace parsing_utilities {

    namespace qi = boost::spirit::qi;

    double wrapAngle180to180(double angle)
    {
        while (angle > 180.0)
        {
            angle -= 360.0;
        }
        while (angle < -180.0)
        {
            angle += 360.0;
        }
        return angle;
    }

    double parseDouble(const uint8_t* buffer)
    {
        double val;
        qi::parse(buffer, buffer + 8, qi::little_bin_double, val);
        return val;
    }

    /**
     * It checks whether an error occurred (via errno) and whether junk characters
     * exist within "string", and returns true if the latter two tests are negative
     * or when the string is empty, false otherwise.
     */
    bool parseDouble(const std::string& string, double& value)
    {
        return string_utilities::toDouble(string, value) || string.empty();
    }

    float parseFloat(const uint8_t* buffer)
    {
        float val;
        qi::parse(buffer, buffer + 4, qi::little_bin_float, val);
        return val;
    }

    /**
     * It checks whether an error occurred (via errno) and whether junk characters
     * exist within "string", and returns true if the latter two tests are negative
     * or when the string is empty, false otherwise.
     */
    bool parseFloat(const std::string& string, float& value)
    {
        return string_utilities::toFloat(string, value) || string.empty();
    }

    /**
     * The function assumes that the bytes in the buffer are already arranged with
     * the same endianness as the local platform. It copies the elements in the range
     * [buffer,buffer + 2) into the range beginning at
     * reinterpret_cast<uint8_t*>(&x). Recall: data_type *var_name = reinterpret_cast
     * <data_type *>(pointer_variable) converts the pointer type, no return type
     */
    int16_t parseInt16(const uint8_t* buffer)
    {
        int16_t val;
        qi::parse(buffer, buffer + 2, qi::little_word, val);
        return val;
    }

    /**
     * It checks whether an error occurred (via errno) and whether junk characters
     * exist within "string", and returns true if the latter two tests are negative
     * or when the string is empty, false otherwise.
     */
    bool parseInt16(const std::string& string, int16_t& value, int32_t base)
    {
        value = 0;
        if (string.empty())
        {
            return true;
        }

        int32_t intermd;
        if (string_utilities::toInt32(string, intermd, base) &&
            intermd <= std::numeric_limits<int16_t>::max() &&
            intermd >= std::numeric_limits<int16_t>::min())
        {
            value = static_cast<int16_t>(intermd);
            return true;
        }

        return false;
    }

    int32_t parseInt32(const uint8_t* buffer)
    {
        int32_t val;
        qi::parse(buffer, buffer + 4, qi::little_dword, val);
        return val;
    }

    /**
     * It checks whether an error occurred (via errno) and whether junk characters
     * exist within "string", and returns true if the latter two tests are negative
     * or when the string is empty, false otherwise.
     */
    bool parseInt32(const std::string& string, int32_t& value, int32_t base)
    {
        return string_utilities::toInt32(string, value, base) || string.empty();
    }

    /**
     * It checks whether an error occurred (via errno) and whether junk characters
     * exist within "string", and returns true if the latter two tests are negative
     * or when the string is empty, false otherwise.
     */
    bool parseUInt8(const std::string& string, uint8_t& value, int32_t base)
    {
        value = 0;
        if (string.empty())
        {
            return true;
        }

        uint32_t intermd;
        if (string_utilities::toUInt32(string, intermd, base) &&
            intermd <= std::numeric_limits<uint8_t>::max())
        {
            value = static_cast<uint8_t>(intermd);
            return true;
        }

        return false;
    }

    uint16_t parseUInt16(const uint8_t* buffer)
    {
        uint16_t val;
        qi::parse(buffer, buffer + 2, qi::little_word, val);
        return val;
    }

    /**
     * It checks whether an error occurred (via errno) and whether junk characters
     * exist within "string", and returns true if the latter two tests are negative
     * or when the string is empty, false otherwise.
     */
    bool parseUInt16(const std::string& string, uint16_t& value, int32_t base)
    {
        value = 0;
        if (string.empty())
        {
            return true;
        }

        uint32_t intermd;
        if (string_utilities::toUInt32(string, intermd, base) &&
            intermd <= std::numeric_limits<uint16_t>::max())
        {
            value = static_cast<uint16_t>(intermd);
            return true;
        }

        return false;
    }

    uint32_t parseUInt32(const uint8_t* buffer)
    {
        uint32_t val;
        qi::parse(buffer, buffer + 4, qi::little_dword, val);
        return val;
    }

    /**
     * It checks whether an error occurred (via errno) and whether junk characters
     * exist within "string", and returns true if the latter two tests are negative
     * or when the string is empty, false otherwise.
     */
    bool parseUInt32(const std::string& string, uint32_t& value, int32_t base)
    {
        return string_utilities::toUInt32(string, value, base) || string.empty();
    }

    /**
     * The UTC precision in NMEA messages is down to a tenth of a second, naturally
     * in both the without-colon-delimiter and the number-of-seconds-since-midnight
     * formats.
     */
    double convertUTCDoubleToSeconds(double utc_double)
    {
        uint32_t hours = static_cast<uint32_t>(utc_double) / 10000;
        uint32_t minutes = (static_cast<uint32_t>(utc_double) - hours * 10000) / 100;
        double seconds =
            utc_double - static_cast<double>(hours * 10000 + minutes * 100);
        seconds += static_cast<double>(hours * 3600 + minutes * 60);
        return seconds;
    }

    /**
     * Recall: One degree is divided into 60 minutes (of arc), and in turn one minute
     * into 60 seconds (of arc). Use of the degrees-minutes-seconds system is also
     * called DMS notation.
     */
    double convertDMSToDegrees(double dms)
    {
        uint32_t whole_degrees = static_cast<uint32_t>(dms) / 100;
        double minutes = dms - static_cast<double>(whole_degrees * 100);
        double degrees = static_cast<double>(whole_degrees) + minutes / 60.0;
        return degrees;
    }

    /**
     * Time information (hours, minutes, seconds) is extracted from the given double
     * and augmented with the date, which is taken from the current system time on
     * the host computer (i.e. current UTC+some_shift time via time(0)). The date
     * ambiguity is resolved by adding/subtracting a day to the current date if the
     * host time is more than 12 hours behind/ahead the NMEA time (i.e. UTC time).
     * Recall time(0), time(NULL): If argument is a null pointer, the parameter is
     * not used (the function still returns the current calendar time of type
     * time_t). Otherwise, the return value is the same as the one stored in the
     * location pointed by the argument. Note that the function assumes that
     * utc_double has two significant digits after the decimal point, i.e. hhmmss.ss,
     * yet it does not round the number of seconds to the nearest unsigned integer,
     * but instead disregards ss. This is since we use this function for the
     * "header.stamp.sec" field of ROS messages, while "header.stamp.nsec" is taken
     * care of separately.
     */
    time_t convertUTCtoUnix(double utc_double)
    {
        time_t time_now = time(0);
        struct tm* timeinfo;

        // The function gmtime uses the value at &time_now to fill a tm structure
        // with the values that represent the corresponding time, expressed as a UTC
        // time.
        timeinfo = gmtime(&time_now);

        uint32_t hours = static_cast<uint32_t>(utc_double) / 10000;
        uint32_t minutes = (static_cast<uint32_t>(utc_double) - hours * 10000) / 100;
        uint32_t seconds =
            (static_cast<uint32_t>(utc_double) - hours * 10000 - minutes * 100);

        // Overwriting timeinfo with UTC time as extracted from utc_double..
        timeinfo->tm_hour = hours;  // hours since midnight - [0,23]
        timeinfo->tm_min = minutes; // minutes after the hour - [0,59]
        timeinfo->tm_sec = seconds; // seconds after the minute - [0,59]

        /* // If you are doing a simulation, add year, month and day here:
        uint32_t year; // year, starting from 1900
        uint32_t month; // months since January - [0,11]
        uint32_t day;  //day of the month - [1,31]
        timeinfo->tm_year = year;
        timeinfo->tm_mon = month;
        timeinfo->tm_mday = day;
        */

        // Inverse of gmtime, the latter converts time_t (Unix time) to tm (UTC time)
        time_t date = timegm(timeinfo);

        // ROS_DEBUG("Since 1970/01/01 %jd seconds have passed.\n", (intmax_t) date);
        return date;
    }

    //! The rotational sequence convention we adopt here (and Septentrio receivers'
    //! pitch, roll, yaw definition too) is the yaw-pitch-roll sequence, i.e. the
    //! 3-2-1 sequence: The body first does yaw around the Z=Down-axis, then pitches
    //! around the new Y=East=right-axis and finally rolls around the new
    //! X=North=forward-axis.
    QuaternionMsg convertEulerToQuaternion(double yaw, double pitch, double roll)
    {
        // Abbreviations for the angular functions
        double cy = std::cos(yaw * 0.5);
        double sy = std::sin(yaw * 0.5);
        double cp = std::cos(pitch * 0.5);
        double sp = std::sin(pitch * 0.5);
        double cr = std::cos(roll * 0.5);
        double sr = std::sin(roll * 0.5);

        QuaternionMsg q;
        q.w = cr * cp * cy + sr * sp * sy;
        q.x = sr * cp * cy - cr * sp * sy;
        q.y = cr * sp * cy + sr * cp * sy;
        q.z = cr * cp * sy - sr * sp * cy;

        return q;
    }

    std::string convertUserPeriodToRxCommand(uint32_t period_user)
    {
        std::string cmd;

        if (period_user == 0)
            return "OnChange";
        else if (period_user < 1000)
            return "msec" + std::to_string(period_user);
        else if (period_user <= 60000)
            return "sec" + std::to_string(period_user / 1000);
        else
            return "min" + std::to_string(period_user / 60000);
    }

    uint16_t getCrc(const uint8_t* buffer) { return parseUInt16(buffer + 2); }

    uint16_t getId(const uint8_t* buffer)
    {
        // Defines bit mask..
        // Highest three bits are for revision and rest for block number
        static uint16_t mask = 8191;
        // Bitwise AND gives us all but highest 3 bits set to zero, rest unchanged

        return parseUInt16(buffer + 4) & mask;
    }

    uint16_t getLength(const uint8_t* buffer) { return parseUInt16(buffer + 6); }

    uint32_t getTow(const uint8_t* buffer) { return parseUInt32(buffer + 8); }

    uint16_t getWnc(const uint8_t* buffer) { return parseUInt16(buffer + 12); }
} // namespace parsing_utilities
