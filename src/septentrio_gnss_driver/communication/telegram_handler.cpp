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

#include <septentrio_gnss_driver/communication/telegram_handler.hpp>

/**
 * @file telegram_handler.cpp
 * @date 22/08/20
 * @brief Handles callbacks when reading NMEA/SBF messages
 */

namespace io {

    void TelegramHandler::handle() {}

    void TelegramHandler::handleSbf(const std::shared_ptr<Telegram>& telegram) {}

    void TelegramHandler::handleNmea(const std::shared_ptr<Telegram>& telegram)
    {
        boost::char_separator<char> sep("\r"); // Carriage Return (CR)
        typedef boost::tokenizer<boost::char_separator<char>> tokenizer;
        std::size_t nmea_size = rx_message_.messageSize();
        // Syntax: new_string_name (const char* s, size_t n); size_t is
        // either 2 or 8 bytes, depending on your system
        std::string block_in_string(
            reinterpret_cast<const char*>(rx_message_.getPosBuffer()), nmea_size);
        tokenizer tokens(block_in_string, sep);
        node_->log(
            LogLevel::DEBUG,
            "The NMEA message contains " + std::to_string(nmea_size) +
                " bytes and is ready to be parsed. It reads: " + *tokens.begin());
    }

    void TelegramHandler::handleResponse(const std::shared_ptr<Telegram>& telegram)
    {
        std::size_t response_size = rx_message_.messageSize();
        std::string block_in_string(
            reinterpret_cast<const char*>(rx_message_.getPosBuffer()),
            response_size);
        node_->log(LogLevel::DEBUG, "The Rx's response contains " +
                                        std::to_string(response_size) +
                                        " bytes and reads:\n " + block_in_string);
        {
            std::mutex::scoped_lock lock(response_sync.mutex);
            response_sync.received = true;
            lock.unlock();
            response_sync.condition.notify_one();
        }
        if (rx_message_.isErrorTelegram())
        {
            node_->log(
                LogLevel::ERROR,
                "Invalid command just sent to the Rx! The Rx's response contains " +
                    std::to_string(response_size) + " bytes and reads:\n " +
                    block_in_string);
        }
    }

    void TelegramHandler::handleCd(const std::shared_ptr<Telegram>& telegram)
    {
        std::string cd(reinterpret_cast<const char*>(rx_message_.getPosBuffer()), 4);
        rx_tcp_port = cd;
        if (cd_count == 0)
        {
            node_->log(LogLevel::INFO,
                       "The connection descriptor for the TCP connection is " + cd);
        }
        if (cd_count < 3)
            ++cd_count;
        if (cd_count == 2)
        {
            std::mutex::scoped_lock lock(cd_sync.mutex);
            cd_sync.received = true;
            lock.unlock();
            cd_sync.condition.notify_one();
        };
    }
} // namespace io