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

#ifndef MINROS_DRIVER_PARSING_UTILITIES_HPP
#define MINROS_DRIVER_PARSING_UTILITIES_HPP

#include <cstdint> // C++ header, corresponds to <stdint.h> in C
#include <string> // C++ header, corresponds to <string.h> in C
#include <ctime> // C++ header, corresponds to <time.h> in C
#include <ros/ros.h>

/**
 * @file parsing_utilities.hpp
 * @brief Declares utility functions used when parsing messages
 * @date 17/08/20 
*/

namespace minros_driver
{
   
	/**
	 * @brief Converts an 8-byte-buffer into a double
	 * @param[in] buffer A pointer to a buffer containing 8 bytes of data
	 * @return The double extracted from the data in the buffer
	 */
	double ParseDouble(const uint8_t* buffer);

	/**
	 * @brief Interprets the contents of "string" as a floating point number of type double, stores its value in "value" and returns whether or not all went well
	 * @param[in] string The string whose content should be interpreted as a floating point number
	 * @param[out] value The double variable that should be overwritten by the floating point number found in "string"
	 * @return True if all went fine, false if not
	 */
	bool ParseDouble(const std::string& string, double& value);

	/**
	 * @brief Converts a 4-byte-buffer into a float
	 * @param[in] buffer A pointer to a buffer containing 4 bytes of data
	 * @return The float extracted from the data in the buffer
	 */
	float ParseFloat(const uint8_t* buffer);

	/**
	 * @brief Interprets the contents of "string" as a floating point number of type float, stores its value in "value" and returns whether or not all went well
	 * @param[in] string The string whose content should be interpreted as a floating point number
	 * @param[out] value The float variable that should be overwritten by the floating point number found in "string"
	 * @return True if all went fine, false if not
	 */
	bool ParseFloat(const std::string& string, float& value);

	/**
	 * @brief Converts a 2-byte-buffer into a signed 16-bit integer
	 * @param[in] buffer A pointer to a buffer containing 2 bytes of data
	 * @return The int16_t value extracted from the data in the buffer
	 */
	int16_t ParseInt16(const uint8_t* buffer);

	/**
	 * @brief Interprets the contents of "string" as a integer number of type int16_t, stores its value in "value" and returns whether or not all went well
	 * @param[in] string The string whose content should be interpreted as an integer number
	 * @param[out] value The int16_t variable that should be overwritten by the integer number found in "string"
	 * @param[in] base The numerical base of the integer in the string, default being 10
	 * @return True if all went fine, false if not
	 */
	bool ParseInt16(const std::string& string, int16_t& value, int32_t base = 10);
	
	/**
	 * @brief Converts a 4-byte-buffer into a signed 32-bit integer
	 * @param[in] buffer A pointer to a buffer containing 4 bytes of data
	 * @return The int32_t value extracted from the data in the buffer
	 */
	int32_t ParseInt32(const uint8_t* buffer);

	/**
	 * @brief Interprets the contents of "string" as a integer number of type int32_t, stores its value in "value" and returns whether or not all went well
	 * @param[in] string The string whose content should be interpreted as an integer number
	 * @param[out] value The int32_t variable that should be overwritten by the integer number found in "string"
	 * @param[in] base The numerical base of the integer in the string, default being 10
	 * @return True if all went fine, false if not
	 */
	bool ParseInt32(const std::string& string, int32_t& value, int32_t base = 10);

	/**
	 * @brief Interprets the contents of "string" as a unsigned integer number of type uint8_t, stores its value in "value" and returns whether or not all went well
	 * @param[in] string The string whose content should be interpreted as an integer number
	 * @param[out] value The uint8_t variable that should be overwritten by the integer number found in "string"
	 * @param[in] base The numerical base of the integer in the string, default being 10
	 * @return True if all went fine, false if not
	 */
	bool ParseUInt8(const std::string& string, uint8_t& value, int32_t base = 10);

	/**
	 * @brief Converts a 2-byte-buffer into an unsigned 16-bit integer
	 * @param[in] buffer A pointer to a buffer containing 2 bytes of data
	 * @return The uint16_t value extracted from the data in the buffer
	 */
	uint16_t ParseUInt16(const uint8_t* buffer);

	/**
	 * @brief Interprets the contents of "string" as a unsigned integer number of type uint16_t, stores its value in "value" and returns whether or not all went well
	 * @param[in] string The string whose content should be interpreted as an integer number
	 * @param[out] value The uint16_t variable that should be overwritten by the integer number found in "string"
	 * @param[in] base The numerical base of the integer in the string, default being 10
	 * @return True if all went fine, false if not
	 */
	bool ParseUInt16(const std::string& string, uint16_t& value, int32_t base = 10);

	/**
	 * @brief Converts a 4-byte-buffer into an unsigned 32-bit integer
	 * @param[in] buffer A pointer to a buffer containing 4 bytes of data
	 * @return The uint32_t value extracted from the data in the buffer
	 */
	uint32_t ParseUInt32(const uint8_t* buffer);

	/**
	 * @brief Interprets the contents of "string" as a unsigned integer number of type uint32_t, stores its value in "value" and returns whether or not all went well
	 * @param[in] string The string whose content should be interpreted as an integer number
	 * @param[out] value The uint32_t variable that should be overwritten by the integer number found in "string"
	 * @param[in] base The numerical base of the integer in the string, default being 10
	 * @return True if all went fine, false if not
	 */
	bool ParseUInt32(const std::string& string, uint32_t& value, int32_t base = 10);

	/**
	 * @brief Converts the UTC time from the without-colon-delimiter format, type double, to the number-of-seconds-since-midnight format, type double
	 * @param[in] utc_float The double variable representing UTC time in the without-colon-delimiter format
	 * @return The double variable representing UTC time in the number-of-seconds-since-midnight format
	 */
	double UTCDoubleToSeconds(double utc_double);
	
	/**
	 * @brief Converts UTC time from the without-colon-delimiter format, type double, to Unix Epoch time (a number-of-seconds-since-1970/01/01 format), type time_t (usually 32 bits)
	 * @param[in] utc_float The double variable representing UTC time in the without-colon-delimiter format
	 * @return The time_t variable representing Unix Epoch time
	 */
	time_t UTCtoUnix(double utc_double);
	
	
	/**
	 * @brief Converts latitude or longitude from the DMS notation (in the without-colon-delimiter format), type double, to the pure degree notation, type double
	 * @param dms The double variable representing latitude or longitude in the DMS notation (in the without-colon-delimiter format)
	 * @return The double variable representing latitude or longitude in the pure degree notation
	 */
	double ConvertDMSToDegrees(double dms);

}

#endif //MINROS_DRIVER_PARSING_UTILITIES_HPP



