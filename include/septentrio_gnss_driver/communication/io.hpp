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
#include <netinet/tcp.h>
#include <sys/socket.h>

// Boost
#include <boost/asio.hpp>
#include <boost/asio/steady_timer.hpp>
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
#include <septentrio_gnss_driver/communication/telegram_parser.hpp>

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
                // idx and bytes_recvd are size_t, so all comparisons are written
                // additively: (bytes_recvd - idx) would underflow into a huge value
                // if idx ever ran past bytes_recvd. Every path through the loop must
                // also either advance idx or break, otherwise the io thread spins
                // forever.
                while (idx + 2 < bytes_recvd)
                {
                    if (buffer_[idx] != SYNC_BYTE_1)
                    {
                        node_->log(log_level::DEBUG, "UDP msg resync.");
                        ++idx;
                        continue;
                    }

                    const telegram_type::TelegramType candidate =
                        telegram_parser::classifySync2(buffer_[idx + 1]);
                    if (candidate == telegram_type::SBF)
                    {
                        if (idx + SBF_HEADER_SIZE > bytes_recvd)
                        {
                            // Header truncated at the end of the datagram, so no
                            // further telegram can be framed out of this packet.
                            break;
                        }

                        uint16_t length =
                            telegram_parser::getSbfLength(&buffer_[idx]);

                        // The length field is attacker/noise controlled. Reject
                        // anything that is not a well-formed SBF length or that
                        // would read past what was actually received, and rescan
                        // from the next byte instead of trusting it.
                        if (!telegram_parser::isValidSbfLength(length) ||
                            idx + length > bytes_recvd)
                        {
                            node_->log(log_level::DEBUG,
                                       "UDP client invalid SBF block length: " +
                                           std::to_string(length));
                            ++idx;
                            continue;
                        }

                        auto telegram = std::make_shared<Telegram>();
                        telegram->stamp = stamp;
                        telegram->message.assign(&buffer_[idx],
                                                 &buffer_[idx] + length);
                        if (crc::isValid(telegram->message))
                        {
                            telegram->type = telegram_type::SBF;
                            telegramQueue_->push(telegram);
                        } else
                            node_->log(log_level::DEBUG,
                                       "UDP client crc failed for SBF  " +
                                           std::to_string(parsing_utilities::getId(
                                               telegram->message)) +
                                           ".");

                        idx += length;
                    } else if (((candidate == telegram_type::NMEA) &&
                                telegram_parser::isNmeaSync3(buffer_[idx + 2])) ||
                               ((candidate == telegram_type::NMEA_INS) &&
                                telegram_parser::isNmeaInsSync3(buffer_[idx + 2])))
                    {
                        bool isIns = (candidate == telegram_type::NMEA_INS);
                        size_t idx_end = findNmeaEnd(idx, bytes_recvd);
                        if (idx_end >= bytes_recvd)
                        {
                            // No terminating CRLF within the datagram: the sentence
                            // is truncated, so drop the remainder rather than
                            // reading past the received bytes.
                            break;
                        }

                        auto telegram = std::make_shared<Telegram>();
                        telegram->stamp = stamp;
                        telegram->message.assign(&buffer_[idx],
                                                 &buffer_[idx_end] + 1);
                        telegram->type =
                            isIns ? telegram_type::NMEA_INS : telegram_type::NMEA;
                        telegramQueue_->push(telegram);
                        idx = idx_end + 1;
                    } else
                    {
                        node_->log(log_level::DEBUG,
                                   "UDP client unknown telegram header: " +
                                       std::string(reinterpret_cast<const char*>(
                                                       &buffer_[idx]),
                                                   3));
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
        //! Returns the index of the LF terminating the sentence starting at idx, or
        //! bytes_recvd if no CRLF was found before the end of the received data.
        size_t findNmeaEnd(size_t idx, size_t bytes_recvd)
        {
            size_t idx_end = idx + 2;

            while (idx_end < bytes_recvd)
            {
                if (telegram_parser::isNmeaEnd(buffer_[idx_end - 1],
                                               buffer_[idx_end]))
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

            deadline_.expires_at(boost::asio::steady_timer::time_point::max());
            checkDeadline();
        }

        ~TcpIo()
        {
            if (stream_)
                stream_->close();
        }

        void close()
        {
            deadline_.cancel();
            if (stream_)
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
                    node_->log(
                        log_level::ERROR,
                        "TCP connection to " +
                            endpoints.begin()->endpoint().address().to_string() +
                            " on port " +
                            std::to_string(endpoints.begin()->endpoint().port()) +
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

            deadline_.expires_at(boost::asio::steady_timer::time_point::max());
            stream_->set_option(boost::asio::ip::tcp::no_delay(true));

            // Kernel TCP keepalive to detect silently broken links, detection
            // time is ca. idle + intvl * cnt seconds
            stream_->set_option(boost::asio::socket_base::keep_alive(true));
            int keepaliveIdle = 3;
            int keepaliveIntvl = 1;
            int keepaliveCnt = 3;
            setsockopt(stream_->native_handle(), IPPROTO_TCP, TCP_KEEPIDLE,
                       &keepaliveIdle, sizeof(keepaliveIdle));
            setsockopt(stream_->native_handle(), IPPROTO_TCP, TCP_KEEPINTVL,
                       &keepaliveIntvl, sizeof(keepaliveIntvl));
            setsockopt(stream_->native_handle(), IPPROTO_TCP, TCP_KEEPCNT,
                       &keepaliveCnt, sizeof(keepaliveCnt));

            node_->log(log_level::INFO, "Connected to " +
                                            endpoints.begin()->host_name() + ":" +
                                            endpoints.begin()->service_name() + ".");
            return true;
        }

    private:
        boost::system::error_code connectInternal(
            const boost::asio::ip::tcp::resolver::results_type& endpoints)
        {
            if (ioContext_->stopped())
                ioContext_->restart();

            boost::system::error_code ec;
            deadline_.expires_after(std::chrono::seconds(10));
            ec = boost::asio::error::would_block;
            boost::asio::async_connect(*stream_, endpoints,
                                       boost::lambda::var(ec) = boost::lambda::_1);
            while (node_->ok() && (ec == boost::asio::error::would_block))
            {
                if (ioContext_->run_one() == 0)
                    break;
            }
            return ec;
        }

        void checkDeadline()
        {
            if (deadline_.expiry() <= std::chrono::steady_clock::now())
            {
                boost::system::error_code ignored_ec;
                stream_->close(ignored_ec);

                deadline_.expires_at(boost::asio::steady_timer::time_point::max());
            }
            deadline_.async_wait(boost::lambda::bind(&TcpIo::checkDeadline, this));
        }

        ROSaicNodeBase* node_;
        std::shared_ptr<boost::asio::io_context> ioContext_;
        boost::asio::steady_timer deadline_;

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

        // stream_ stays null if connect() was never called or failed to open the
        // file.
        ~SbfFileIo()
        {
            if (stream_)
                stream_->close();
        }

        void close()
        {
            if (stream_)
                stream_->close();
        }

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

        ~PcapFileIo() { close(); }

        // Called both explicitly by AsyncManager and again from the destructor, so
        // it has to be idempotent: pcap_close() on an already-closed handle is a
        // double free, and both members stay unset if connect() never ran or failed.
        void close()
        {
            if (pcap_ != nullptr)
            {
                pcap_close(pcap_);
                pcap_ = nullptr;
            }
            if (stream_)
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

                // pcap_open_offline reports failure by returning NULL rather than
                // throwing, so without this check pcap_get_selectable_fd() would be
                // handed a null handle.
                pcap_ = pcap_open_offline(node_->settings()->device.c_str(),
                                          errBuff_.data());
                if (pcap_ == nullptr)
                {
                    errBuff_.back() = '\0';
                    node_->log(log_level::ERROR, "opening PCAP file failed: " +
                                                     std::string(errBuff_.data()));
                    return false;
                }
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
        //! pcap requires the error buffer to hold at least PCAP_ERRBUF_SIZE bytes;
        //! anything smaller can be overrun by libpcap itself.
        std::array<char, PCAP_ERRBUF_SIZE> errBuff_;
        pcap_t* pcap_ = nullptr;

    public:
        std::unique_ptr<boost::asio::posix::stream_descriptor> stream_;
    };
} // namespace io
