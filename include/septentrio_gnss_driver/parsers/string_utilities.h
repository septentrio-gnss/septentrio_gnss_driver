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

#ifndef STRING_UTILITIES_H
#define STRING_UTILITIES_H

// C and C++ library includes
#include <cstdint>
#include <locale> // Merely for "isdigit()" function, also available in <cctype.h> C header..
#include <string>

/**
 * @file string_utilities.h
 * @brief Declares lower-level string utility functions used when parsing messages
 * @date 13/08/20
 */

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @namespace string_utilities
 * This namespace is for the functions that encapsulate basic string manipulation and
 * conversion techniques.
 */
namespace string_utilities {
    /**
     * @brief Interprets the contents of "string" as a floating point number of type
     * double It stores the "string"'s value in "value" and returns whether or not
     * all went well.
     * @param[in] string The string whose content should be interpreted as a floating
     * point number
     * @param[out] value The double variable that should be overwritten by the
     * floating point number found in "string"
     * @return True if all went fine, false if not
     */
    bool toDouble(const std::string& string, double& value);

    /**
     * @brief Interprets the contents of "string" as a floating point number of type
     * float.
     *
     * It stores the "string"'s value in "value" and returns whether or not all went
     * well.
     * @param[in] string The string whose content should be interpreted as a floating
     * point number
     * @param[out] value The float variable that should be overwritten by the
     * floating point number found in "string"
     * @return True if all went fine, false if not
     */
    bool toFloat(const std::string& string, float& value);

    /**
     * @brief Interprets the contents of "string" as a floating point number of
     * whatever integer type your system has that is exactly 32 bits.
     *
     * It stores the "string"'s value in "value" and returns whether or not all went
     * well.
     * @param[in] string The string whose content should be interpreted as a floating
     * point number
     * @param[out] value The int32_t variable that should be overwritten by the
     * floating point number found in "string"
     * @param[in] base The conversion assumes this base, here: decimal
     * @return True if all went fine, false if not
     */
    bool toInt32(const std::string& string, int32_t& value, int32_t base = 10);

    /**
     * @brief Interprets the contents of "string" as a floating point number of
     * whatever unsigned integer type your system has that is exactly 32 bits.
     *
     * It stores the "string"'s value in "value" and returns whether or not all went
     * well.
     * @param[in] string The string whose content should be interpreted as a floating
     * point number
     * @param[out] value The uint32_t variable that should be overwritten by the
     * floating point number found in "string"
     * @param[in] base The conversion assumes this base, here: decimal
     * @return True if all went fine, false if not
     */
    bool toUInt32(const std::string& string, uint32_t& value, int32_t base = 10);

    /**
     * @brief Interprets the contents of "string" as a floating point number of
     * whatever integer type your system has that is exactly 8 bits.
     *
     * It stores the "string"'s value in "value".
     * @param[in] string The string whose content should be interpreted as a floating
     * point number
     * @param[out] value The int8_t variable that should be overwritten by the
     * floating point number found in "string"
     * @param[in] base The conversion assumes this base, here: decimal
     * @return The value found in "string"
     */
    int8_t toInt8(const std::string& string, int8_t& value, int32_t base = 10);

    /**
     * @brief Interprets the contents of "string" as a floating point number of
     * whatever unsigned integer type your system has that is exactly 8 bits.
     *
     * It stores the "string"'s value in "value".
     * @param[in] string The string whose content should be interpreted as a floating
     * point number
     * @param[out] value The uint8_t variable that should be overwritten by the
     * floating point number found in "string"
     * @param[in] base The conversion assumes this base, here: decimal
     * @return The value found in "string"
     */
    uint8_t toUInt8(const std::string& string, uint8_t& value, int32_t base = 10);

    /**
     * @brief Trims decimal places to two
     * @param[in] num The double who shall be trimmed
     * @return The string
     */
    std::string trimDecimalPlaces(double num);

    /**
     * @brief Checks if a string contains spaces
     * @param[in] str the string to be analyzed
     * @return true if string contains space
     */
    bool containsSpace(const std::string str);
} // namespace string_utilities

#ifdef __cplusplus
}
#endif

#endif // STRING_UTILITIES_H
