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
#include <septentrio_gnss_driver/parsers/string_utilities.h>
// C++ library includes
#include <cerrno>
#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <limits>
#include <sstream>

/**
 * @file string_utilities.cpp
 * @brief Defines lower-level string utility functions used when parsing messages
 * @date 13/08/20
 */

namespace string_utilities {
    /**
     * It checks whether an error occurred (via errno) and whether junk characters
     * exist within "string", and returns true if the latter two tests are negative
     * and the string is non-empty, false otherwise.
     */
    bool toDouble(const std::string& string, double& value)
    {
        if (string.empty())
        {
            return false;
        }

        char* end;
        errno = 0;

        double value_new = std::strtod(string.c_str(), &end);

        if (errno != 0 || end != string.c_str() + string.length())
        {
            return false;
        }

        value = value_new;
        return true;
    }

    /**
     * It checks whether an error occurred (via errno) and whether junk characters
     * exist within "string", and returns true if the latter two tests are negative
     * and the string is non-empty, false otherwise.
     */
    bool toFloat(const std::string& string, float& value)
    {
        if (string.empty())
        {
            return false;
        }

        char* end;
        errno = 0;
        float value_new = std::strtof(string.c_str(), &end);

        if (errno != 0 || end != string.c_str() + string.length())
        {
            return false;
        }

        value = value_new;
        return true;
    }

    /**
     * It checks whether an error occurred (via errno) and whether junk characters
     * exist within "string", and returns true if the latter two tests are negative
     * and the string is non-empty, false otherwise.
     */
    bool toInt32(const std::string& string, int32_t& value, int32_t base)
    {
        if (string.empty())
        {
            return false;
        }

        char* end;
        errno = 0;
        int64_t value_new = std::strtol(string.c_str(), &end, base);

        if (errno != 0 || end != string.c_str() + string.length())
        {
            return false;
        }

        if (value_new > std::numeric_limits<int32_t>::max() ||
            value_new < std::numeric_limits<int32_t>::min())
        {
            return false;
        }

        value = (int32_t)value_new;
        return true;
    }

    /**
     * It checks whether an error occurred (via errno) and whether junk characters
     * exist within "string", and returns true if the latter two tests are negative
     * and the string is non-empty, false otherwise.
     */
    bool toUInt32(const std::string& string, uint32_t& value, int32_t base)
    {
        if (string.empty())
        {
            return false;
        }

        char* end;
        errno = 0;
        int64_t value_new = std::strtol(string.c_str(), &end, base);

        if (errno != 0 || end != string.c_str() + string.length())
        {
            return false;
        }

        if (value_new > std::numeric_limits<uint32_t>::max() || value_new < 0)
        {
            return false;
        }

        value = (uint32_t)value_new;
        return true;
    }

    /**
     * Not used as of now..
     */
    int8_t toInt8(const std::string& string, int8_t& value, int32_t base)
    {
        char* end;
        errno = 0;
        int64_t value_new = std::strtol(string.c_str(), &end, base);

        value = (int8_t)value_new;
        return value;
    }

    /**
     * Not used as of now..
     */
    uint8_t toUInt8(const std::string& string, uint8_t& value, int32_t base)
    {
        char* end;
        errno = 0;
        int64_t value_new = std::strtol(string.c_str(), &end, base);

        value = (uint8_t)value_new;
        return true;
    }

    std::string trimDecimalPlaces(double num)
    {
        num = std::round(num * 1000);
        num = num / 1000;
        std::stringstream ss;
        ss << std::fixed;
        ss << std::setprecision(3);
        ss << num;
        return ss.str();
    }

    bool containsSpace(const std::string str)
    {
        for (size_t i = 0; i < str.size(); ++i)
        {
            if (std::isspace(str[i]))
                return true;
        }
        return false;
    }
} // namespace string_utilities
