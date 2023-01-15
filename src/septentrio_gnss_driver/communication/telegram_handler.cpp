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

    void TelegramHandler::handleTelegram(const std::shared_ptr<Telegram>& telegram)
    {
        switch (telegram->type)
        {
        case message_type::SBF:
        {
            handleSbf(telegram);
            break;
        }
        case message_type::NMEA:
        {
            handleNmea(telegram);
            break;
        }
        case message_type::NMEA_INS:
        {
            handleNmea(telegram);
            break;
        }
        case message_type::RESPONSE:
        {
            handleResponse(telegram);
            break;
        }
        case message_type::ERROR_RESPONSE:
        {
            handleResponse(telegram);
            break;
        }
        case message_type::CONNECTION_DESCRIPTOR:
        {
            handleCd(telegram);
            break;
        }
        case message_type::UNKNOWN:
        {
            node_->log(LogLevel::DEBUG, "Unhandeled message received: " +
                                            std::string(telegram->message.begin(),
                                                        telegram->message.end()));
            break;
        }
        default:
        {
            node_->log(LogLevel::DEBUG,
                       "TelegramHandler received an invalid message to handle");
            break;
        }
        }
    }

    void TelegramHandler::handleSbf(const std::shared_ptr<Telegram>& telegram)
    {
        messageParser_.parseSbf(telegram);
    }

    void TelegramHandler::handleNmea(const std::shared_ptr<Telegram>& telegram)
    {
        messageParser_.parseNmea(telegram);
    }

    void TelegramHandler::handleResponse(const std::shared_ptr<Telegram>& telegram)
    {
        std::string block_in_string(telegram->message.begin(),
                                    telegram->message.end());

        if (telegram->type == message_type::ERROR_RESPONSE)
        {
            node_->log(
                LogLevel::ERROR,
                "Invalid command just sent to the Rx! The Rx's response contains " +
                    std::to_string(block_in_string.size()) + " bytes and reads:\n " +
                    block_in_string);
        } else
        {
            node_->log(LogLevel::DEBUG, "The Rx's response contains " +
                                            std::to_string(block_in_string.size()) +
                                            " bytes and reads:\n " +
                                            block_in_string);
        }
        try
        {
            responseSemaphore_.notify();
        } catch (std::exception& e)
        {
            node_->log(LogLevel::DEBUG, "handleResponse " + std::string(e.what()));
        }
    }

    void TelegramHandler::handleCd(const std::shared_ptr<Telegram>& telegram)
    {
        if (cdCtr_ < 2)
        {
            mainConnectionDescriptor_ =
                std::string(telegram->message.begin(), telegram->message.end() - 1);

            ++cdCtr_;
            if (cdCtr_ == 2)
            {
                node_->log(LogLevel::INFO, "The connection descriptor is " +
                                               mainConnectionDescriptor_);
                try
                {
                    cdSemaphore_.notify();
                } catch (std::exception& e)
                {
                    node_->log(LogLevel::DEBUG,
                               "handleCd cd " + std::string(e.what()));
                }
            }
        }
    }
} // namespace io