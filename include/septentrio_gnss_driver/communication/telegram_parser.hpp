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

// C++
#include <cstdint>

// ROSaic
#include <septentrio_gnss_driver/communication/telegram.hpp>
#include <septentrio_gnss_driver/parsers/parsing_utilities.hpp>

/**
 * @file telegram_parser.hpp
 * @brief Protocol knowledge shared by the stream and datagram telegram parsers
 */

namespace telegram_parser {

    //! Classifies a telegram by its 2nd sync byte, the leading SYNC_BYTE_1 has
    //! to be checked by the caller
    [[nodiscard]] inline telegram_type::TelegramType classifySync2(uint8_t byte2)
    {
        switch (byte2)
        {
        case SBF_SYNC_BYTE_2:
            return telegram_type::SBF;
        case NMEA_SYNC_BYTE_2:
            return telegram_type::NMEA;
        case NMEA_INS_SYNC_BYTE_2:
            return telegram_type::NMEA_INS;
        case RESPONSE_SYNC_BYTE_2:
            return telegram_type::RESPONSE;
        default:
            return telegram_type::UNKNOWN;
        }
    }

    //! Whether the 3rd sync byte belongs to an NMEA talker
    [[nodiscard]] inline bool isNmeaSync3(uint8_t byte3)
    {
        return (byte3 == NMEA_SYNC_BYTE_3) || (byte3 == NMEA_SYNC_BYTE_3a) ||
               (byte3 == NMEA_SYNC_BYTE_3b) || (byte3 == NMEA_SYNC_BYTE_3c) ||
               (byte3 == NMEA_SYNC_BYTE_3d);
    }

    //! Whether the 3rd sync byte belongs to the INS NMEA talker
    [[nodiscard]] inline bool isNmeaInsSync3(uint8_t byte3)
    {
        return byte3 == NMEA_INS_SYNC_BYTE_3;
    }

    //! Whether the 3rd sync byte belongs to a command response
    [[nodiscard]] inline bool isResponseSync3(uint8_t byte3)
    {
        return (byte3 == RESPONSE_SYNC_BYTE_3) || (byte3 == RESPONSE_SYNC_BYTE_3a);
    }

    //! Whether the 3rd sync byte belongs to an error response
    [[nodiscard]] inline bool isErrorSync3(uint8_t byte3)
    {
        return byte3 == ERROR_SYNC_BYTE_3;
    }

    //! Extracts the SBF block length, block must point to the start of the block
    [[nodiscard]] inline uint16_t getSbfLength(const uint8_t* block)
    {
        return parsing_utilities::parseUInt16(block + 6);
    }

    //! Per the SBF spec the length covers the header and is a multiple of 4
    [[nodiscard]] inline bool isValidSbfLength(uint16_t length)
    {
        return (length >= SBF_HEADER_SIZE) && ((length % 4) == 0);
    }

    //! Whether the current byte terminates an NMEA sentence or response
    [[nodiscard]] inline bool isNmeaEnd(uint8_t prevByte, uint8_t currByte)
    {
        return (currByte == LF) && (prevByte == CR);
    }
} // namespace telegram_parser
