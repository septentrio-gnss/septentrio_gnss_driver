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

#include <rosaic/parsers/nmea_parsers/gpgga.hpp>
#include <boost/make_shared.hpp>
#include <rosaic/parsers/string_utilities.h>
#include <ros/ros.h>

/**
 * @file gpgga.cpp
 * @brief Derived class for parsing GGA messages
 * @date 17/08/20 
 */

const std::string rosaic_driver::GpggaParser::MESSAGE_ID = "$GPGGA";

const std::string rosaic_driver::GpggaParser::GetMessageID() const
{
	return rosaic_driver::GpggaParser::MESSAGE_ID;
}

/**
 * Caution: Due to the occurrence of the throw keyword, this method ParseAscii should be called within a try / catch framework...
 * Note: This method assumes that "sentence" does not include the checksum part of the GGA message.
 */
nmea_msgs::GpggaPtr rosaic_driver::GpggaParser::ParseASCII(const rosaic_driver::NMEASentence& sentence) noexcept(false)
{
	//ROS_DEBUG("Just testing that first entry is indeed what we expect it to: %s", sentence.get_body()[0].c_str());
	// Check the length first, which should be 15 elements (if station ID available, otherwise missing field, not empty field) or 14 elements (if station ID not avaiable).
	const size_t MAX_LEN = 15;
	const size_t MIN_LEN = 14;
	if (sentence.get_body().size() > MAX_LEN || sentence.get_body().size() < MIN_LEN)
	{
		std::stringstream error;
		error << "Expected GPGGA length " << MIN_LEN << "  <= length <= "
			  << MAX_LEN << ", actual length = " << sentence.get_body().size();
		throw ParseException(error.str());
	}

	nmea_msgs::GpggaPtr msg = boost::make_shared<nmea_msgs::Gpgga>();
	msg->header.frame_id = frame_id;

	msg->message_id = sentence.get_body()[0];

	if (sentence.get_body()[1].empty() || sentence.get_body()[1] == "0")
	{
		msg->utc_seconds = 0;
	}
	else
	{
		double utc_double;
		if (string_utilities::ToDouble(sentence.get_body()[1], utc_double))
		{
			//ROS_DEBUG("utc_double is %f", (float) utc_double);
			msg->utc_seconds = UTCDoubleToSeconds(utc_double);
			
			// The Header's Unix Epoch time stamp
			time_t unix_time_seconds = UTCtoUnix(utc_double);
			uint32_t unix_time_nanoseconds = (static_cast<uint32_t>(utc_double*100)%100)*10000; // Assumes there are two digits after the decimal point in NMEA UTC time
			msg->header.stamp.sec = unix_time_seconds;
			msg->header.stamp.nsec = unix_time_nanoseconds;
		}
		else
		{
			throw rosaic_driver::ParseException("Error parsing UTC seconds in GPGGA"); // E.g. if one of the time fields of the string is empty
		}
	}

	bool valid = true;

	double latitude = 0.0;
	valid = valid && rosaic_driver::ParseDouble(sentence.get_body()[2], latitude);
	msg->lat = ConvertDMSToDegrees(latitude);

	double longitude = 0.0;
	valid = valid && rosaic_driver::ParseDouble(sentence.get_body()[4], longitude);
	msg->lon = ConvertDMSToDegrees(longitude);

	msg->lat_dir = sentence.get_body()[3];
	msg->lon_dir = sentence.get_body()[5];
	valid = valid && rosaic_driver::ParseUInt32(sentence.get_body()[6], msg->gps_qual);
	valid = valid && rosaic_driver::ParseUInt32(sentence.get_body()[7], msg->num_sats);
	//ROS_DEBUG("Valid is %s so far with number of satellites in use being %s", valid ? "true" : "false", sentence.get_body()[7].c_str());

	valid = valid && rosaic_driver::ParseFloat(sentence.get_body()[8], msg->hdop);
	valid = valid && rosaic_driver::ParseFloat(sentence.get_body()[9], msg->alt);
	msg->altitude_units = sentence.get_body()[10];
	valid = valid && rosaic_driver::ParseFloat(sentence.get_body()[11], msg->undulation);
	msg->undulation_units = sentence.get_body()[12];
	valid = valid && rosaic_driver::ParseUInt32(sentence.get_body()[13], msg->diff_age);
	if (sentence.get_body().size() == MAX_LEN)
	{
		msg->station_id = sentence.get_body()[14];
	}
	else
	{
		msg->station_id = "";
	}

	if (!valid)
	{
		was_last_gpgga_valid_ = false;
		throw ParseException("GPGGA message was invalid.");
	}

	// If we made it this far, we successfully parsed the message and will consider it to be valid.
	was_last_gpgga_valid_ = true;

	return msg;
}

bool rosaic_driver::GpggaParser::WasLastGPGGAValid() const
{
	return was_last_gpgga_valid_;
}
