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

#include <septentrio_gnss_driver/parsers/nmea_parsers/gpgga.hpp>

/**
 * @file gpgga.cpp
 * @brief Derived class for parsing GGA messages
 * @date 17/08/20
 */

const std::string GpggaParser::MESSAGE_ID = "$GPGGA";

const std::string GpggaParser::getMessageID() const
{
    return GpggaParser::MESSAGE_ID;
}

/**
 * Caution: Due to the occurrence of the throw keyword, this method parseASCII should
 * be called within a try / catch framework... Note: This method is called from
 * within the read() method of the RxMessage class by including the checksum part in
 * the argument "sentence" here, though the checksum is never parsed: It would be
 * sentence.get_body()[15] if anybody ever needs it.
 */
GpggaMsg GpggaParser::parseASCII(const NMEASentence& sentence,
                                 const std::string& frame_id, bool use_gnss_time,
                                 Timestamp time_obj) noexcept(false)
{
    // ROS_DEBUG("Just testing that first entry is indeed what we expect it to be:
    // %s", sentence.get_body()[0].c_str());
    // Check the length first, which should be 16 elements.
    const size_t LEN = 16;
    if (sentence.get_body().size() > LEN || sentence.get_body().size() < LEN)
    {
        std::stringstream error;
        error << "GGA parsing failed: Expected GPGGA length is " << LEN
              << ", but actual length is " << sentence.get_body().size();
        throw ParseException(error.str());
    }

    GpggaMsg msg;
    msg.header.frame_id = frame_id;

    msg.message_id = sentence.get_body()[0];

    if (sentence.get_body()[1].empty() || sentence.get_body()[1] == "0")
    {
        msg.utc_seconds = 0;
    } else
    {
        double utc_double;
        if (string_utilities::toDouble(sentence.get_body()[1], utc_double))
        {
            if (use_gnss_time)
            {
                // ROS_DEBUG("utc_double is %f", (float) utc_double);
                msg.utc_seconds =
                    parsing_utilities::convertUTCDoubleToSeconds(utc_double);

                // The Header's Unix Epoch time stamp
                time_t unix_time_seconds =
                    parsing_utilities::convertUTCtoUnix(utc_double);
                // The following assumes that there are two digits after the decimal
                // point in utc_double, i.e. in the NMEA UTC time.
                Timestamp unix_time_nanoseconds =
                    unix_time_seconds * 1000000000 +
                    (static_cast<Timestamp>(utc_double * 100) % 100) * 10000;
                msg.header.stamp = timestampToRos(unix_time_nanoseconds);
            } else
            {
                msg.header.stamp = timestampToRos(time_obj);
            }
        } else
        {
            throw ParseException(
                "Error parsing UTC seconds in GPGGA"); // E.g. if one of the fields
                                                       // of the NMEA UTC string is
                                                       // empty
        }
    }

    bool valid = true;

    double latitude = 0.0;
    valid =
        valid && parsing_utilities::parseDouble(sentence.get_body()[2], latitude);
    msg.lat = parsing_utilities::convertDMSToDegrees(latitude);

    double longitude = 0.0;
    valid =
        valid && parsing_utilities::parseDouble(sentence.get_body()[4], longitude);
    msg.lon = parsing_utilities::convertDMSToDegrees(longitude);

    msg.lat_dir = sentence.get_body()[3];
    msg.lon_dir = sentence.get_body()[5];
    valid = valid &&
            parsing_utilities::parseUInt32(sentence.get_body()[6], msg.gps_qual);
    valid = valid &&
            parsing_utilities::parseUInt32(sentence.get_body()[7], msg.num_sats);
    // ROS_INFO("Valid is %s so far with number of satellites in use being %s", valid
    // ? "true" : "false", sentence.get_body()[7].c_str());

    valid = valid && parsing_utilities::parseFloat(sentence.get_body()[8], msg.hdop);
    valid = valid && parsing_utilities::parseFloat(sentence.get_body()[9], msg.alt);
    msg.altitude_units = sentence.get_body()[10];
    valid = valid &&
            parsing_utilities::parseFloat(sentence.get_body()[11], msg.undulation);
    msg.undulation_units = sentence.get_body()[12];
    double diff_age_temp;
    valid = valid &&
            parsing_utilities::parseDouble(sentence.get_body()[13], diff_age_temp);
    msg.diff_age = static_cast<uint32_t>(round(diff_age_temp));
    msg.station_id = sentence.get_body()[14];

    if (!valid)
    {
        was_last_gpgga_valid_ = false;
        throw ParseException("GPGGA message was invalid.");
    }

    // If we made it this far, we successfully parsed the message and will consider
    // it to be valid.
    was_last_gpgga_valid_ = true;

    return msg;
}

bool GpggaParser::wasLastGPGGAValid() const { return was_last_gpgga_valid_; }
