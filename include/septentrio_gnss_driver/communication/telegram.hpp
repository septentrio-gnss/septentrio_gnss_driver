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
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <queue>
#include <vector>

// ROSaic
#ifdef ROS2
#include <septentrio_gnss_driver/abstraction/typedefs.hpp>
#endif
#ifdef ROS1
#include <septentrio_gnss_driver/abstraction/typedefs_ros1.hpp>
#endif

//! 0x24 is ASCII for $ - 1st byte in each message
static const uint8_t SYNC_BYTE_1 = 0x24;
//! 0x40 is ASCII for @ - 2nd byte to indicate SBF block
static const uint8_t SBF_SYNC_BYTE_2 = 0x40;
//! 0x47 is ASCII for G - 2nd byte to indicate NMEA-type ASCII message
static const uint8_t NMEA_SYNC_BYTE_2 = 0x47;
//! 0x50 is ASCII for P - 3rd byte to indicate NMEA-type ASCII message
static const uint8_t NMEA_SYNC_BYTE_3 = 0x50;
//! 0x49 is ASCII for I - 2nd byte to indicate INS NMEA-type ASCII message
static const uint8_t NMEA_INS_SYNC_BYTE_2 = 0x49;
//! 0x4E is ASCII for N - 3rd byte to indicate NMEA-type ASCII message
static const uint8_t NMEA_INS_SYNC_BYTE_3 = 0x4E;
//! 0x52 is ASCII for R (for "Response") - 2nd byte in each response from the Rx
static const uint8_t RESPONSE_SYNC_BYTE_2 = 0x52;
//! 0x3A is ASCII for : - 3rd byte in the response message from the Rx
static const uint8_t RESPONSE_SYNC_BYTE_3 = 0x3A;
//! 0x21 is ASCII for ! 3rd byte in the response message from the Rx
static const uint8_t RESPONSE_SYNC_BYTE_3a = 0x21;
//! 0x3F is ASCII for ? - 3rd byte in the response message from the Rx in case the
//! command was invalid
static const uint8_t ERROR_SYNC_BYTE_3 = 0x3F;
//! 0x0D is ASCII for "Carriage Return", i.e. "Enter"
static const uint8_t CR = 0x0D;
//! 0x0A is ASCII for "Line Feed", i.e. "New Line"
static const uint8_t LF = 0x0A;
//! 0x49 is ASCII for I - 1st character of connection descriptor sent by the Rx after
//! initiating IP connection
static const uint8_t CONNECTION_DESCRIPTOR_BYTE_I = 0x49;
//! 0x43 is ASCII for C - 1st character of connection descriptor sent by the Rx after
//! initiating COM connection
static const uint8_t CONNECTION_DESCRIPTOR_BYTE_C = 0x43;
//! 0x55 is ASCII for U - 1st character of connection descriptor sent by the Rx after
//! initiating USB connection
static const uint8_t CONNECTION_DESCRIPTOR_BYTE_U = 0x55;
//! 0x4E is ASCII for N - 1st character of connection descriptor sent by the Rx after
//! initiating NTRIP connection
static const uint8_t CONNECTION_DESCRIPTOR_BYTE_N = 0x4E;
//! 0x44 is ASCII for D - 1st character of connection descriptor sent by the Rx after
//! initiating DSK connection
static const uint8_t CONNECTION_DESCRIPTOR_BYTE_D = 0x44;
//! 0x3E is ASCII for > - end character of connection descriptor
static const uint8_t CONNECTION_DESCRIPTOR_FOOTER = 0x3E;

static const uint16_t SBF_HEADER_SIZE = 8;
static const uint16_t MAX_SBF_SIZE = 65535;
static const uint16_t MAX_UDP_PACKET_SIZE = 65535;

namespace telegram_type {
    enum TelegramType
    {
        EMPTY,
        SBF,
        NMEA,
        NMEA_INS,
        RESPONSE,
        ERROR_RESPONSE,
        CONNECTION_DESCRIPTOR,
        UNKNOWN
    };
}

struct Telegram
{
    Timestamp stamp;
    telegram_type::TelegramType type;
    std::vector<uint8_t> message;

    Telegram(size_t preallocate = 3) noexcept :
        stamp(0), type(telegram_type::EMPTY),
        message(std::vector<uint8_t>(preallocate))
    {
    }

    ~Telegram() {}

    Telegram(const Telegram& other) noexcept :
        stamp(other.stamp), type(other.type), message(other.message)
    {
    }

    Telegram(Telegram&& other) noexcept :
        stamp(other.stamp), type(other.type), message(other.message)
    {
    }

    Telegram& operator=(const Telegram& other) noexcept
    {
        if (this != &other)
        {
            this->stamp = other.stamp;
            this->type = other.type;
            this->message = other.message;
        }
        return *this;
    }

    Telegram& operator=(Telegram&& other) noexcept
    {
        if (this != &other)
        {
            this->stamp = other.stamp;
            this->type = other.type;
            this->message = other.message;
        }
        return *this;
    }
};

template <typename T>
class ConcurrentQueue
{
public:
    [[nodiscard]] bool empty() const noexcept;
    [[nodiscard]] size_t size() const noexcept;
    void push(const T& input) noexcept;
    void pop(T& output) noexcept;

private:
    std::queue<T> queue_;
    std::condition_variable cond_;
    mutable std::mutex mtx_;
};

template <typename T>
[[nodiscard]] bool ConcurrentQueue<T>::empty() const noexcept
{
    std::lock_guard<std::mutex> lck(mtx_);
    return queue_.empty();
}

template <typename T>
[[nodiscard]] size_t ConcurrentQueue<T>::size() const noexcept
{
    std::lock_guard<std::mutex> lck(mtx_);
    return queue_.size();
}

template <typename T>
void ConcurrentQueue<T>::push(const T& input) noexcept
{
    {
        std::lock_guard<std::mutex> lck(mtx_);
        queue_.push(input);
    }
    cond_.notify_one();
}

template <typename T>
void ConcurrentQueue<T>::pop(T& output) noexcept
{
    std::unique_lock<std::mutex> lck(mtx_);
    cond_.wait(lck, [this] { return !queue_.empty(); });
    output = queue_.front();
    queue_.pop();
}

typedef ConcurrentQueue<std::shared_ptr<Telegram>> TelegramQueue;