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

#include <septentrio_gnss_driver/parsers/nmea_parsers/gpgsv.hpp>

/**
 * @file gpgsv.cpp
 * @brief Derived class for parsing GSV messages
 * @date 29/09/20
 */

const std::string GpgsvParser::MESSAGE_ID = "$GPGSV";

const std::string GpgsvParser::getMessageID() const
{
    return GpgsvParser::MESSAGE_ID;
}

/**
 * Caution: Due to the occurrence of the throw keyword, this method parseASCII should
 * be called within a try / catch framework... Note: This method is called from
 * within the read() method of the RxMessage class by including the checksum part in
 * the argument "sentence" here, though the checksum is never parsed: E.g. for
 * message with 4 Svs it would be sentence.get_body()[20] if anybody ever needs it.
 */
GpgsvMsg GpgsvParser::parseASCII(const NMEASentence& sentence,
                                 const std::string& frame_id, bool /*use_gnss_time*/,
                                 Timestamp /*time_obj*/) noexcept(false)
{

    const size_t MIN_LENGTH = 4;
    // Checking that the message is at least as long as a GPGSV with no satellites
    if (sentence.get_body().size() < MIN_LENGTH)
    {
        std::stringstream error;
        error << "Expected GSV length is at least " << MIN_LENGTH
              << ". The actual length is " << sentence.get_body().size();
        throw ParseException(error.str());
    }
    GpgsvMsg msg;
    msg.header.frame_id = frame_id;
    msg.message_id = sentence.get_body()[0];
    if (!parsing_utilities::parseUInt8(sentence.get_body()[1], msg.n_msgs))
    {
        throw ParseException("Error parsing n_msgs in GSV.");
    }
    if (msg.n_msgs >
        9) // Checking that the number of messages is smaller or equal to 9
    {
        std::stringstream error;
        error << "n_msgs in GSV is too large: " << msg.n_msgs << ".";
        throw ParseException(error.str());
    }

    if (!parsing_utilities::parseUInt8(sentence.get_body()[2], msg.msg_number))
    {
        throw ParseException("Error parsing msg_number in GSV.");
    }
    if (msg.msg_number >
        msg.n_msgs) // Checking that this message is within the sequence range
    {
        std::stringstream error;
        error << "msg_number in GSV is larger than n_msgs: " << msg.msg_number
              << " > " << msg.n_msgs << ".";
        throw ParseException(error.str());
    }
    if (!parsing_utilities::parseUInt8(sentence.get_body()[3], msg.n_satellites))
    {
        throw ParseException("Error parsing n_satellites in GSV.");
    }
    // Figuring out how many satellites should be described in this sentence
    size_t n_sats_in_sentence = 4;
    if (msg.msg_number == msg.n_msgs)
    {
        n_sats_in_sentence = msg.n_satellites % static_cast<uint8_t>(4);
        if (msg.n_satellites % static_cast<uint8_t>(4) == 0)
        {
            n_sats_in_sentence = 4;
        }
        if (msg.n_satellites == 0)
        {
            n_sats_in_sentence = 0;
        }
        if (msg.msg_number == 1)
        {
            n_sats_in_sentence = msg.n_satellites;
        }
    }
    // Checking that the sentence is the right length for the number of satellites
    size_t expected_length = MIN_LENGTH + 4 * n_sats_in_sentence + 1;
    // Note that we add +1 due to the checksum data being part of the argument
    // "sentence".
    if (n_sats_in_sentence == 0)
    {
        // Even if the number of sats is 0, the message will still have enough
        // blank fields for 1 satellite.
        expected_length += 4;
    }
    // ROS_DEBUG("number of sats is %u but nsats in sentence if msg_number = max is
    // %u and msg.msg_number == msg.n_msgs is %s and nsats in sentence is %li",
    // msg.n_satellites, msg.n_satellites % static_cast<uint8_t>(4),
    // msg.msg_number
    // == msg.n_msgs ? "true" : "false", n_sats_in_sentence);
    if (sentence.get_body().size() != expected_length &&
        sentence.get_body().size() != expected_length - 1)
    {
        std::stringstream ss;
        for (size_t i = 0; i < sentence.get_body().size(); ++i)
        {
            ss << sentence.get_body()[i];
            if ((i + 1) < sentence.get_body().size())
            {
                ss << ",";
            }
        }
        std::stringstream error;
        error << "Expected GSV length is " << expected_length << " for message with "
              << n_sats_in_sentence << " satellites. The actual length is "
              << sentence.get_body().size() << ".\n"
              << ss.str().c_str();
        throw ParseException(error.str());
    }

    // Parsing information about n_sats_in_sentence SVs..
    msg.satellites.resize(n_sats_in_sentence);
    for (size_t sat = 0, index = MIN_LENGTH; sat < n_sats_in_sentence;
         ++sat, index += 4)
    {
        if (!parsing_utilities::parseUInt8(sentence.get_body()[index],
                                           msg.satellites[sat].prn))
        {
            std::stringstream error;
            error << "Error parsing PRN for satellite " << sat << " in GSV.";
            throw ParseException(error.str());
        }
        float elevation;
        if (!parsing_utilities::parseFloat(sentence.get_body()[index + 1],
                                           elevation))
        {
            std::stringstream error;
            error << "Error parsing elevation for satellite " << sat << " in GSV.";
            throw ParseException(error.str());
        }
        msg.satellites[sat].elevation = static_cast<uint8_t>(elevation);

        float azimuth;
        if (!parsing_utilities::parseFloat(sentence.get_body()[index + 2], azimuth))
        {
            std::stringstream error;
            error << "Error parsing azimuth for satellite " << sat << " in GSV.";
            throw ParseException(error.str());
        }
        msg.satellites[sat].azimuth = static_cast<uint16_t>(azimuth);

        if ((index + 3) >= sentence.get_body().size() ||
            sentence.get_body()[index + 3].empty())
        {
            msg.satellites[sat].snr = -1;
        } else
        {
            uint8_t snr;
            if (!parsing_utilities::parseUInt8(sentence.get_body()[index + 3], snr))
            {
                std::stringstream error;
                error << "Error parsing snr for satellite " << sat << " in GSV.";
                throw ParseException(error.str());
            }
            msg.satellites[sat].snr = static_cast<int8_t>(snr);
        }
    }
    return msg;
}