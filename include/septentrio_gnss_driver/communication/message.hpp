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
#include <byte>
#include <vector>

// TBB
#include <tbb/concurrent_queue.h>

// ROSaic
#include <septentrio_gnss_driver/abstraction/typedefs.hpp>

//! 0x24 is ASCII for $ - 1st byte in each message
static const std::byte SYNC_0 = 0x24;
//! 0x40 is ASCII for @ - 2nd byte to indicate SBF block
static const std::byte SBF_SYNC_BYTE_1 = 0x40;
//! 0x47 is ASCII for G - 2nd byte to indicate NMEA-type ASCII message
static const std::byte NMEA_SYNC_BYTE_1 0x47;
//! 0x50 is ASCII for P - 2nd byte to indicate proprietary ASCII message
static const std::byte NMEA_SYNC_BYTE_2 0x50;
//! 0x52 is ASCII for R (for "Response") - 2nd byte in each response from the Rx
static const std::byte RESPONSE_SYNC_BYTE_1 0x52;
//! 0x3F is ASCII for ? - 3rd byte in the response message from the Rx in case the
//! command was invalid
static const std::byte RESPONSE_SYNC_BYTE_2 0x3F;
//! 0x0D is ASCII for "Carriage Return", i.e. "Enter"
static const std::byte CR 0x0D;
//! 0x0A is ASCII for "Line Feed", i.e. "New Line"
static const std::byte LF 0x0A;
//! 0x49 is ASCII for I - 1st character of connection descriptor sent by the Rx after
//! initiating TCP connection
static const std::byte CONNECTION_DESCRIPTOR_BYTE_1 0x49;
//! 0x50 is ASCII for P - 2nd character of connection descriptor sent by the Rx after
//! initiating TCP connection
static const std::byte CONNECTION_DESCRIPTOR_BYTE_2 0x50;

static const uint16_t SBF_HEADER_SIZE = 8;
static const uint16_t MAX_SBF_SIZE 65535;

enum MessageType
{
    INVALID,
    SBF,
    NMEA,
    RESPONSE,
    CONNECTION_DESCRIPTOR
};

struct Message
{
    Timestamp stamp = 0;
    MessageType type = INVALID;
    uint16_t sbfId = 0;
    std::vector<std::byte> data(3);

    Message() : stamp(0), type(INVALID), sbfId(0), data(std::vector<std::byte>(3))
    {
        data.reserve(MAX_SBF_SIZE);
    }
};

typedef tbb::concurrent_bounded_queue<std::shared_ptr<Message>> MessageQueue;
