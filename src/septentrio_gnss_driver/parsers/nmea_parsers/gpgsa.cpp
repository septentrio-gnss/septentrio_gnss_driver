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

#include <septentrio_gnss_driver/parsers/nmea_parsers/gpgsa.hpp>

/**
 * @file gpgsa.cpp
 * @brief Derived class for parsing GSA messages
 * @date 29/09/20
 */

const std::string GpgsaParser::MESSAGE_ID = "$GPGSA";

const std::string GpgsaParser::getMessageID() const
{
    return GpgsaParser::MESSAGE_ID;
}

/**
 * Caution: Due to the occurrence of the throw keyword, this method ParseASCII should
 * be called within a try / catch framework... Note: This method is called from
 * within the read() method of the RxMessage class by including the checksum part in
 * the argument "sentence" here, though the checksum is never parsed: It would be
 * sentence.get_body()[18] if anybody ever needs it.
 */
GpgsaMsg GpgsaParser::parseASCII(const NMEASentence& sentence,
                                 const std::string& frame_id, bool /*use_gnss_time*/,
                                 Timestamp /*time_obj*/) noexcept(false)
{

    // Checking the length first, it should be 19 elements
    const size_t LENGTH = 19;
    if (sentence.get_body().size() != LENGTH)
    {
        std::stringstream error;
        error << "Expected GPGSA length is " << LENGTH << ". The actual length is "
              << sentence.get_body().size();
        throw ParseException(error.str());
    }

    GpgsaMsg msg;
    msg.header.frame_id = frame_id;
    msg.message_id = sentence.get_body()[0];
    msg.auto_manual_mode = sentence.get_body()[1];
    parsing_utilities::parseUInt8(sentence.get_body()[2], msg.fix_mode);
    // Words 3-14 of the sentence are SV PRNs. Copying only the non-null strings..
    // 0 is the character needed to fill the new character space, in case 12 (first
    // argument) is larger than sv_ids.
    msg.sv_ids.resize(12, 0);
    size_t n_svs = 0;
    for (std::vector<std::string>::const_iterator id =
             sentence.get_body().begin() + 3;
         id < sentence.get_body().begin() + 15; ++id)
    {
        if (!id->empty())
        {
            parsing_utilities::parseUInt8(*id, msg.sv_ids[n_svs]);
            ++n_svs;
        }
    }
    msg.sv_ids.resize(n_svs);

    parsing_utilities::parseFloat(sentence.get_body()[15], msg.pdop);
    parsing_utilities::parseFloat(sentence.get_body()[16], msg.hdop);
    parsing_utilities::parseFloat(sentence.get_body()[17], msg.vdop);
    return msg;
}