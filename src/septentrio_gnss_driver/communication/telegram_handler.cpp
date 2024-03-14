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
        case telegram_type::SBF:
        {
            handleSbf(telegram);
            break;
        }
        case telegram_type::NMEA:
        {
            handleNmea(telegram);
            break;
        }
        case telegram_type::NMEA_INS:
        {
            handleNmea(telegram);
            break;
        }
        case telegram_type::RESPONSE:
        {
            handleResponse(telegram);
            break;
        }
        case telegram_type::ERROR_RESPONSE:
        {
            handleResponse(telegram);
            break;
        }
        case telegram_type::CONNECTION_DESCRIPTOR:
        {
            handleCd(telegram);
            break;
        }
        case telegram_type::UNKNOWN:
        {
            std::string block_in_string(telegram->message.begin(),
                                        telegram->message.end());

            node_->log(log_level::DEBUG, "A message received: " + block_in_string);
            if (block_in_string.find("ReceiverCapabilities") != std::string::npos)
            {
                if (block_in_string.find("INS") != std::string::npos)
                {
                    node_->setIsIns();
                }

                if (block_in_string.find("Heading") != std::string::npos)
                {
                    node_->setHasHeading();
                }
                capabilitiesSemaphore_.notify();
            }
            break;
        }
        default:
        {
            node_->log(log_level::DEBUG,
                       "TelegramHandler received an invalid message to handle");
            break;
        }
        }
    }

    void TelegramHandler::handleSbf(const std::shared_ptr<Telegram>& telegram)
    {
        messageHandler_.parseSbf(telegram);
    }

    void TelegramHandler::handleNmea(const std::shared_ptr<Telegram>& telegram)
    {
        messageHandler_.parseNmea(telegram);
    }

    void TelegramHandler::handleResponse(const std::shared_ptr<Telegram>& telegram)
    {
        std::string block_in_string(telegram->message.begin(),
                                    telegram->message.end());

        if (telegram->type == telegram_type::ERROR_RESPONSE)
        {
            if (block_in_string ==
                std::string(
                    "$R? setGNSSAttitude: Argument 'Source' is invalid!\r\n"))
            {
                node_->log(
                    log_level::WARN,
                    "Rx does not support dual antenna mode, set parameter multi_antenna to false and/or disable publishing of atteuler.");
            } else if (block_in_string ==
                       std::string("$R? sptp, on : Invalid command!\r\n"))
            {
                node_->log(
                    log_level::WARN,
                    "Rx does not support PTP server clock. GNSS needs firmare >= 4.14., INS does not support it yet.");
            } else
            {
                node_->log(
                    log_level::ERROR,
                    "Invalid command just sent to the Rx! The Rx's response contains " +
                        std::to_string(block_in_string.size()) +
                        " bytes and reads:\n " + block_in_string);
            }
        } else
        {
            node_->log(log_level::DEBUG, "The Rx's response contains " +
                                             std::to_string(block_in_string.size()) +
                                             " bytes and reads:\n " +
                                             block_in_string);
        }
        responseSemaphore_.notify();
    }

    void TelegramHandler::handleCd(const std::shared_ptr<Telegram>& telegram)
    {
        node_->log(log_level::DEBUG,
                   "handleCd: " + std::string(telegram->message.begin(),
                                              telegram->message.end()));
        if (telegram->message.back() == CONNECTION_DESCRIPTOR_FOOTER)
        {
            mainConnectionDescriptor_ =
                std::string(telegram->message.begin(), telegram->message.end() - 1);

            cdSemaphore_.notify();
        }
    }
} // namespace io