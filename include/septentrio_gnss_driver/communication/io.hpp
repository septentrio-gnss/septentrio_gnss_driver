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
#include <thread>

// Linux
#include <linux/input.h>
#include <linux/serial.h>

// Boost
#include <boost/asio.hpp>
#include <boost/asio/deadline_timer.hpp>
#include <boost/lambda/bind.hpp>
#include <boost/lambda/lambda.hpp>

// pcap
#include <pcap.h>

// ROSaic
#ifdef ROS2
#include <septentrio_gnss_driver/abstraction/typedefs.hpp>
#endif
#ifdef ROS1
#include <septentrio_gnss_driver/abstraction/typedefs_ros1.hpp>
#endif
#include <septentrio_gnss_driver/communication/telegram.hpp>

//! Possible baudrates for the Rx
const static std::array<uint32_t, 21> baudrates = {
    1200,    2400,    4800,    9600,    19200,   38400,   57600,
    115200,  230400,  460800,  500000,  576000,  921600,  1000000,
    1152000, 1500000, 2000000, 2500000, 3000000, 3500000, 4000000};

namespace io {

    class UdpClient
    {
    public:
        UdpClient(ROSaicNodeBase* node, int16_t port, TelegramQueue* telegramQueue) :
            node_(node), running_(true), port_(port), telegramQueue_(telegramQueue)
        {
            connect();
            watchdogThread_ =
                std::thread(boost::bind(&UdpClient::runWatchdog, this));
        }

        ~UdpClient()
        {
            running_ = false;

            node_->log(log_level::INFO, "UDP client shutting down threads");
            ioContext_.stop();
            ioThread_.join();
            watchdogThread_.join();
            node_->log(log_level::INFO, " UDP client threads stopped");
        }

    private:
        void connect()
        {
            socket_ = std::make_unique<boost::asio::ip::udp::socket>(
                ioContext_,
                boost::asio::ip::udp::endpoint(boost::asio::ip::udp::v4(), port_));

            asyncReceive();

            ioThread_ = std::thread(boost::bind(&UdpClient::runIoContext, this));

            node_->log(log_level::INFO,
                       "Listening on UDP port " + std::to_string(port_));
        }

        void asyncReceive()

        {
            socket_->async_receive_from(
                boost::asio::buffer(buffer_, MAX_UDP_PACKET_SIZE), eP_,
                boost::bind(&UdpClient::handleReceive, this,
                            boost::asio::placeholders::error,
                            boost::asio::placeholders::bytes_transferred));
        }

        void handleReceive(const boost::system::error_code& error,
                           size_t bytes_recvd)
        {
            Timestamp stamp = node_->getTime();
            size_t idx = 0;

            if (!error && (bytes_recvd > 0))
            {
                while ((bytes_recvd - idx) > 2)
                {
                    auto telegram = std::make_shared<Telegram>();
                    telegram->stamp = stamp;
                    /*node_->log(log_level::DEBUG,
                               "Buffer: " + std::string(telegram->message.begin(),
                                                        telegram->message.end()));*/
                    if (buffer_[idx] == SYNC_BYTE_1)
                    {
                        if (buffer_[idx + 1] == SBF_SYNC_BYTE_2)
                        {
                            if ((bytes_recvd - idx) > SBF_HEADER_SIZE)
                            {
                                uint16_t length = parsing_utilities::parseUInt16(
                                    &buffer_[idx + 6]);
                                telegram->message.assign(&buffer_[idx],
                                                         &buffer_[idx + length]);
                                if (crc::isValid(telegram->message))
                                {
                                    telegram->type = telegram_type::SBF;
                                    telegramQueue_->push(telegram);
                                } else
                                    node_->log(
                                        log_level::DEBUG,
                                        "AsyncManager crc failed for SBF  " +
                                            std::to_string(parsing_utilities::getId(
                                                telegram->message)) +
                                            ".");

                                idx += length;
                            }

                        } else if ((buffer_[idx + 1] == NMEA_SYNC_BYTE_2) &&
                                   (buffer_[idx + 2] == NMEA_SYNC_BYTE_3))
                        {
                            size_t idx_end = findNmeaEnd(idx, bytes_recvd);
                            telegram->message.assign(&buffer_[idx],
                                                     &buffer_[idx_end + 1]);
                            telegram->type = telegram_type::NMEA;
                            telegramQueue_->push(telegram);
                            idx = idx_end + 1;

                        } else if ((buffer_[idx + 1] == NMEA_INS_SYNC_BYTE_2) &&
                                   (buffer_[idx + 2] == NMEA_INS_SYNC_BYTE_3))
                        {
                            size_t idx_end = findNmeaEnd(idx, bytes_recvd);
                            telegram->message.assign(&buffer_[idx],
                                                     &buffer_[idx_end + 1]);
                            telegram->type = telegram_type::NMEA_INS;
                            telegramQueue_->push(telegram);
                            idx = idx_end + 1;
                        } else
                        {
                            node_->log(log_level::DEBUG,
                                       "head: " +
                                           std::string(std::string(
                                               telegram->message.begin(),
                                               telegram->message.begin() + 2)));
                        }
                    } else
                    {
                        node_->log(log_level::DEBUG, "UDP msg resync.");
                        ++idx;
                    }
                }
            } else
            {
                node_->log(log_level::ERROR,
                           "UDP client receive error: " + error.message());
            }

            asyncReceive();
        }

        void runIoContext()
        {
            ioContext_.run();
            node_->log(log_level::INFO, "UDP client ioContext terminated.");
        }

        void runWatchdog()
        {
            while (running_)
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(1000));

                if (running_ && ioContext_.stopped())
                {
                    node_->log(log_level::ERROR,
                               "UDP client connection lost. Trying to reconnect.");
                    ioContext_.restart();
                    ioThread_.join();
                    connect();
                }
            }
        }

    private:
        size_t findNmeaEnd(size_t idx, size_t bytes_recvd)
        {
            size_t idx_end = idx + 2;

            while (idx_end < bytes_recvd)
            {
                if ((buffer_[idx_end] == LF) && (buffer_[idx_end - 1] == CR))
                    break;

                ++idx_end;
            }
            return idx_end;
        }
        //! Pointer to the node
        ROSaicNodeBase* node_;
        std::atomic<bool> running_;
        int16_t port_;
        boost::asio::io_context ioContext_;
        std::thread ioThread_;
        std::thread watchdogThread_;
        boost::asio::ip::udp::endpoint eP_;
        std::unique_ptr<boost::asio::ip::udp::socket> socket_;
        std::array<uint8_t, MAX_UDP_PACKET_SIZE> buffer_;
        TelegramQueue* telegramQueue_;
    };

    class TcpIo
    {
    public:
        TcpIo(ROSaicNodeBase* node,
              std::shared_ptr<boost::asio::io_context> ioContext) :
            node_(node), ioContext_(ioContext), deadline_(*ioContext_)
        {
            port_ = node_->settings()->device_tcp_port;

            deadline_.expires_at(boost::posix_time::pos_infin);
            checkDeadline();
        }

        ~TcpIo() { stream_->close(); }

        void close()
        {
            deadline_.cancel();
            stream_->close();
        }

        void setPort(const std::string& port) { port_ = port; }

        [[nodiscard]] bool connect()
        {
            boost::asio::ip::tcp::resolver::results_type endpoints;

            try
            {
                boost::asio::ip::tcp::resolver resolver(*ioContext_);
                endpoints =
                    resolver.resolve(node_->settings()->device_tcp_ip, port_);
            } catch (const std::runtime_error& e)
            {
                node_->log(log_level::ERROR,
                           "Could not resolve " + node_->settings()->device_tcp_ip +
                               " on port " + port_ + ": " + e.what());
                return false;
            }

            stream_ = std::make_unique<boost::asio::ip::tcp::socket>(*ioContext_);

            node_->log(log_level::INFO, "Connecting to tcp://" +
                                            node_->settings()->device_tcp_ip + ":" +
                                            port_ + "...");

            try
            {
                boost::system::error_code ec = connectInternal(endpoints);
                while (node_->ok() && ec)
                {
                    node_->log(log_level::ERROR,
                               "TCP connection to " +
                                   endpoints.begin()
                                       ->endpoint()
                                       .address()
                                       .to_string() +
                                   " on port " +
                                   std::to_string(
                                       endpoints.begin()->endpoint().port()) +
                                   " failed: " + ec.message() + ". Retrying ...");
                    using namespace std::chrono_literals;
                    std::this_thread::sleep_for(1s);
                    ec = connectInternal(endpoints);
                }
                if (ec)
                    return false;

            } catch (const std::runtime_error& e)
            {
                node_->log(log_level::ERROR,
                           "Could not connect to " + endpoints.begin()->host_name() +
                               ": " + endpoints.begin()->service_name() + ": " +
                               e.what());
                return false;
            }

            deadline_.expires_at(boost::posix_time::pos_infin);
            stream_->set_option(boost::asio::ip::tcp::no_delay(true));
            node_->log(log_level::INFO, "Connected to " +
                                            endpoints.begin()->host_name() + ":" +
                                            endpoints.begin()->service_name() + ".");
            return true;
        }

    private:
        boost::system::error_code connectInternal(
            const boost::asio::ip::tcp::resolver::results_type& endpoints)
        {
            boost::system::error_code ec;
            deadline_.expires_from_now(boost::posix_time::seconds(10));
            ec = boost::asio::error::would_block;
            boost::asio::async_connect(*stream_, endpoints,
                                       boost::lambda::var(ec) = boost::lambda::_1);
            do
                ioContext_->run_one();
            while (node_->ok() && (ec == boost::asio::error::would_block));
            return ec;
        }

        void checkDeadline()
        {
            if (deadline_.expires_at() <=
                boost::asio::deadline_timer::traits_type::now())
            {
                boost::system::error_code ignored_ec;
                stream_->close(ignored_ec);

                deadline_.expires_at(boost::posix_time::pos_infin);
            }
            deadline_.async_wait(boost::lambda::bind(&TcpIo::checkDeadline, this));
        }

        ROSaicNodeBase* node_;
        std::shared_ptr<boost::asio::io_context> ioContext_;
        boost::asio::deadline_timer deadline_;

        std::string port_;

    public:
        std::unique_ptr<boost::asio::ip::tcp::socket> stream_;
    };

    class SerialIo
    {
    public:
        SerialIo(ROSaicNodeBase* node,
                 std::shared_ptr<boost::asio::io_context> ioContext) :
            node_(node), ioContext_(ioContext),
            flowcontrol_(node->settings()->hw_flow_control),
            baudrate_(node->settings()->baudrate)
        {
            stream_ = std::make_unique<boost::asio::serial_port>(*ioContext_);
        }

        ~SerialIo() { stream_->close(); }

        void close() { stream_->close(); }

        [[nodiscard]] bool connect()
        {
            if (stream_->is_open())
            {
                stream_->close();
            }

            bool opened = false;

            while (!opened && node_->ok())
            {
                try
                {
                    node_->log(log_level::INFO,
                               "Connecting serially to device " +
                                   node_->settings()->device +
                                   ", targeted baudrate: " +
                                   std::to_string(node_->settings()->baudrate));
                    stream_->open(node_->settings()->device);
                    opened = true;
                } catch (const boost::system::system_error& err)
                {
                    node_->log(log_level::ERROR, "Could not open serial port " +
                                                     node_->settings()->device +
                                                     ". Error: " + err.what() +
                                                     ". Will retry every second.");

                    using namespace std::chrono_literals;
                    std::this_thread::sleep_for(1s);
                }
            }
            if (!opened)
                return false;

            // No Parity, 8bits data, 1 stop Bit
            stream_->set_option(boost::asio::serial_port_base::baud_rate(baudrate_));
            stream_->set_option(boost::asio::serial_port_base::parity(
                boost::asio::serial_port_base::parity::none));
            stream_->set_option(boost::asio::serial_port_base::character_size(8));
            stream_->set_option(boost::asio::serial_port_base::stop_bits(
                boost::asio::serial_port_base::stop_bits::one));

            // Hardware flow control settings
            if (flowcontrol_ == "RTS|CTS")
            {
                stream_->set_option(boost::asio::serial_port_base::flow_control(
                    boost::asio::serial_port_base::flow_control::hardware));
            } else
            {
                stream_->set_option(boost::asio::serial_port_base::flow_control(
                    boost::asio::serial_port_base::flow_control::none));
            }

            // Set low latency
            int fd = stream_->native_handle();
            struct serial_struct serialInfo;
            ioctl(fd, TIOCGSERIAL, &serialInfo);
            serialInfo.flags |= ASYNC_LOW_LATENCY;
            ioctl(fd, TIOCSSERIAL, &serialInfo);

            return setBaudrate();
        }

        [[nodiscard]] bool setBaudrate()
        {
            // Setting the baudrate, incrementally..
            node_->log(log_level::DEBUG,
                       "Gradually increasing the baudrate to the desired value...");
            boost::asio::serial_port_base::baud_rate current_baudrate;
            node_->log(log_level::DEBUG, "Initiated current_baudrate object...");
            try
            {
                stream_->get_option(current_baudrate); // Note that this sets
                                                       // current_baudrate.value()
                                                       // often to 115200, since by
                                                       // default, all Rx COM ports,
                // at least for mosaic Rxs, are set to a baudrate of 115200 baud,
                // using 8 data-bits, no parity and 1 stop-bit.
            } catch (boost::system::system_error& e)
            {

                node_->log(log_level::ERROR, "get_option failed due to " +
                                                 static_cast<std::string>(e.what()));
                node_->log(log_level::INFO, "Additional info about error is " +
                                                static_cast<std::string>(e.what()));
                /*
                boost::system::error_code e_loop;
                do // Caution: Might cause infinite loop..
                {
                    stream_->get_option(current_baudrate, e_loop);
                } while(e_loop);
                */
                return false;
            }
            // Gradually increase the baudrate to the desired value
            // The desired baudrate can be lower or larger than the
            // current baudrate; the for loop takes care of both scenarios.
            node_->log(log_level::DEBUG,
                       "Current baudrate is " +
                           std::to_string(current_baudrate.value()));
            for (uint8_t i = 0; i < baudrates.size(); i++)
            {
                if (current_baudrate.value() == baudrate_)
                {
                    break; // Break if the desired baudrate has been reached.
                }
                if (current_baudrate.value() >= baudrates[i] &&
                    baudrate_ > baudrates[i])
                {
                    continue;
                }
                // Increment until Baudrate[i] matches current_baudrate.
                try
                {
                    stream_->set_option(
                        boost::asio::serial_port_base::baud_rate(baudrates[i]));
                } catch (boost::system::system_error& e)
                {

                    node_->log(log_level::ERROR,
                               "set_option failed due to " +
                                   static_cast<std::string>(e.what()));
                    node_->log(log_level::INFO,
                               "Additional info about error is " +
                                   static_cast<std::string>(e.what()));
                    return false;
                }
                using namespace std::chrono_literals;
                std::this_thread::sleep_for(500ms);

                try
                {
                    stream_->get_option(current_baudrate);
                } catch (boost::system::system_error& e)
                {

                    node_->log(log_level::ERROR,
                               "get_option failed due to " +
                                   static_cast<std::string>(e.what()));
                    node_->log(log_level::INFO,
                               "Additional info about error is " +
                                   static_cast<std::string>(e.what()));
                    /*
                    boost::system::error_code e_loop;
                    do // Caution: Might cause infinite loop..
                    {
                        stream_->get_option(current_baudrate, e_loop);
                    } while(e_loop);
                    */
                    return false;
                }
                node_->log(log_level::DEBUG,
                           "Set ASIO baudrate to " +
                               std::to_string(current_baudrate.value()));
            }
            node_->log(log_level::INFO,
                       "Set ASIO baudrate to " +
                           std::to_string(current_baudrate.value()) +
                           ", leaving InitializeSerial() method");

            // clear io
            ::tcflush(stream_->native_handle(), TCIOFLUSH);

            return true;
        }

    private:
        ROSaicNodeBase* node_;
        std::shared_ptr<boost::asio::io_context> ioContext_;
        std::string flowcontrol_;
        uint32_t baudrate_;

    public:
        std::unique_ptr<boost::asio::serial_port> stream_;
    };

    class SbfFileIo
    {
    public:
        SbfFileIo(ROSaicNodeBase* node,
                  std::shared_ptr<boost::asio::io_context> ioContext) :
            node_(node), ioContext_(ioContext)
        {
        }

        ~SbfFileIo() { stream_->close(); }

        void close() { stream_->close(); }

        [[nodiscard]] bool connect()
        {
            node_->log(log_level::INFO, "Opening SBF file stream" +
                                            node_->settings()->device + "...");

            int fd = open(node_->settings()->device.c_str(), O_RDONLY);
            if (fd == -1)
            {
                node_->log(log_level::ERROR, "open SBF file failed.");
                return false;
            }

            try
            {
                stream_ = std::make_unique<boost::asio::posix::stream_descriptor>(
                    *ioContext_);
                stream_->assign(fd);

            } catch (std::runtime_error& e)
            {
                node_->log(log_level::ERROR, "assigning SBF file failed due to " +
                                                 static_cast<std::string>(e.what()));
                return false;
            }
            return true;
        }

    private:
        ROSaicNodeBase* node_;
        std::shared_ptr<boost::asio::io_context> ioContext_;

    public:
        std::unique_ptr<boost::asio::posix::stream_descriptor> stream_;
    };

    class PcapFileIo
    {
    public:
        PcapFileIo(ROSaicNodeBase* node,
                   std::shared_ptr<boost::asio::io_context> ioContext) :
            node_(node), ioContext_(ioContext)
        {
        }

        ~PcapFileIo()
        {
            pcap_close(pcap_);
            stream_->close();
        }

        void close()
        {
            pcap_close(pcap_);
            stream_->close();
        }

        [[nodiscard]] bool connect()
        {
            try
            {
                node_->log(log_level::INFO, "Opening pcap file stream" +
                                                node_->settings()->device + "...");

                stream_ = std::make_unique<boost::asio::posix::stream_descriptor>(
                    *ioContext_);

                pcap_ = pcap_open_offline(node_->settings()->device.c_str(),
                                          errBuff_.data());
                stream_->assign(pcap_get_selectable_fd(pcap_));

            } catch (std::runtime_error& e)
            {
                node_->log(log_level::ERROR, "assigning PCAP file failed due to " +
                                                 static_cast<std::string>(e.what()));
                return false;
            }
            return true;
        }

    private:
        ROSaicNodeBase* node_;
        std::shared_ptr<boost::asio::io_context> ioContext_;
        std::array<char, 100> errBuff_;
        pcap_t* pcap_;

    public:
        std::unique_ptr<boost::asio::posix::stream_descriptor> stream_;
    };
} // namespace io
