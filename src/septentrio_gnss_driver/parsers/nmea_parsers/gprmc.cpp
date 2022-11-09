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

#include <septentrio_gnss_driver/parsers/nmea_parsers/gprmc.hpp>

/**
 * @file gprmc.cpp
 * @brief Derived class for parsing RMC messages
 * @date 28/09/20
 */

const std::string GprmcParser::MESSAGE_ID = "$GPRMC";

const std::string GprmcParser::getMessageID() const
{
    return GprmcParser::MESSAGE_ID;
}

/**
 * Caution: Due to the occurrence of the throw keyword, this method ParseASCII should
 * be called within a try / catch framework... Note: This method is called from
 * within the read() method of the RxMessage class by including the checksum part in
 * the argument "sentence" here, though the checksum is never parsed: It would be
 * sentence.get_body()[13] if anybody ever needs it. The status character can be 'A'
 * (for Active) or 'V' (for Void), signaling whether the GPS was active when the
 * positioning was made. If it is void, the GPS could not make a good positioning and
 * you should thus ignore it. This usually occurs when the GPS is still searching for
 * satellites. WasLastGPRMCValid() will return false in this case.
 */
GprmcMsg GprmcParser::parseASCII(const NMEASentence& sentence,
                                 const std::string& frame_id, bool use_gnss_time,
                                 Timestamp time_obj) noexcept(false)
{

    // Checking the length first, it should be between 13 and 14 elements
    const size_t LEN_MIN = 13;
    const size_t LEN_MAX = 14;

    if (sentence.get_body().size() > LEN_MAX || sentence.get_body().size() < LEN_MIN)
    {
        std::stringstream error;
        error << "Expected GPRMC length is between " << LEN_MIN << " and " << LEN_MAX
              << ". The actual length is " << sentence.get_body().size();
        throw ParseException(error.str());
    }

    GprmcMsg msg;

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
            msg.utc_seconds =
                parsing_utilities::convertUTCDoubleToSeconds(utc_double);
            if (use_gnss_time)
            {
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
                "Error parsing UTC seconds in GPRMC"); // E.g. if one of the fields
                                                       // of the NMEA UTC string is
                                                       // empty
        }
    }
    bool valid = true;
    bool to_be_ignored = false;

    msg.position_status = sentence.get_body()[2];
    // Check to see whether this message should be ignored
    to_be_ignored &= !(sentence.get_body()[2].compare("A") ==
                       0); // 0 : if both strings are equal.
    to_be_ignored &=
        (sentence.get_body()[3].empty() || sentence.get_body()[5].empty());

    double latitude = 0.0;
    valid =
        valid && parsing_utilities::parseDouble(sentence.get_body()[3], latitude);
    msg.lat = parsing_utilities::convertDMSToDegrees(latitude);

    double longitude = 0.0;
    valid =
        valid && parsing_utilities::parseDouble(sentence.get_body()[5], longitude);
    msg.lon = parsing_utilities::convertDMSToDegrees(longitude);

    msg.lat_dir = sentence.get_body()[4];
    msg.lon_dir = sentence.get_body()[6];

    valid =
        valid && parsing_utilities::parseFloat(sentence.get_body()[7], msg.speed);
    msg.speed *= KNOTS_TO_MPS;

    valid =
        valid && parsing_utilities::parseFloat(sentence.get_body()[8], msg.track);

    std::string date_str = sentence.get_body()[9];
    if (!date_str.empty())
    {
        msg.date = std::string("20") + date_str.substr(4, 2) + std::string("-") +
                   date_str.substr(2, 2) + std::string("-") + date_str.substr(0, 2);
    }
    valid =
        valid && parsing_utilities::parseFloat(sentence.get_body()[10], msg.mag_var);
    msg.mag_var_direction = sentence.get_body()[11];
    if (sentence.get_body().size() == LEN_MAX)
    {
        msg.mode_indicator = sentence.get_body()[12];
    }

    if (!valid)
    {
        was_last_gprmc_valid_ = false;
        throw ParseException("Error parsing GPRMC message.");
    }

    was_last_gprmc_valid_ = !to_be_ignored;

    return msg;
}

bool GprmcParser::wasLastGPRMCValid() const { return was_last_gprmc_valid_; }
