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

#include <chrono>
#include <linux/serial.h>

// Boost includes
#include <boost/regex.hpp>
#include <septentrio_gnss_driver/communication/communication_core.hpp>

static const int16_t ANGLE_MAX = 180;
static const int16_t ANGLE_MIN = -180;
static const int8_t THETA_Y_MAX = 90;
static const int8_t THETA_Y_MIN = -90;
static const int8_t LEVER_ARM_MAX = 100;
static const int8_t LEVER_ARM_MIN = -100;
static const int16_t HEADING_MAX = 360;
static const int16_t HEADING_MIN = -360;
static const int8_t PITCH_MAX = 90;
static const int8_t PITCH_MIN = -90;
static const int8_t ATTSTD_DEV_MIN = 0;
static const int8_t ATTSTD_DEV_MAX = 5;
static const int8_t POSSTD_DEV_MIN = 0;
static const int8_t POSSTD_DEV_MAX = 100;

/**
 * @file communication_core.cpp
 * @date 22/08/20
 * @brief Highest-Level view on communication services
 */

namespace io {

    CommunicationCore::CommunicationCore(ROSaicNodeBase* node) :
        node_(node), settings_(node->settings()), telegramHandler_(node),
        running_(true)
    {
        running_ = true;

        processingThread_ =
            std::thread(std::bind(&CommunicationCore::processTelegrams, this));
    }

    CommunicationCore::~CommunicationCore()
    {
        telegramHandler_.clearSemaphores();

        resetSettings();

        running_ = false;
        std::shared_ptr<Telegram> telegram(new Telegram);
        telegramQueue_.push(telegram);
        processingThread_.join();
    }

    void CommunicationCore::resetSettings()
    {
        if (settings_->configure_rx && !settings_->read_from_sbf_log &&
            !settings_->read_from_pcap)
        {
            resetMainConnection();
            send("sdio, " + mainConnectionPort_ + ", auto, none\x0D");
            // Turning off all current SBF/NMEA output
            send("sso, all, none, none, off \x0D");
            send("sno, all, none, none, off \x0D");
            if ((settings_->udp_port != 0) && (!settings_->udp_ip_server.empty()))
            {
                send("siss, " + settings_->udp_ip_server + ",  0\x0D");
            }
            for (auto ntrip : settings_->rtk.ntrip)
            {
                if (!ntrip.id.empty() && !ntrip.keep_open)
                {
                    send("snts, " + ntrip.id + ", off \x0D");
                }
            }
            for (auto ip_server : settings_->rtk.ip_server)
            {
                if (!ip_server.id.empty() && !ip_server.keep_open)
                {
                    send("sdio, " + ip_server.id + ",  auto, none\x0D");
                    send("siss, " + ip_server.id + ",  0\x0D");
                }
            }
            for (auto serial : settings_->rtk.serial)
            {
                if (!serial.port.empty() && !serial.keep_open)
                {
                    send("sdio, " + serial.port + ",  auto, none\x0D");
                    if (serial.port.rfind("COM", 0) == 0)
                        send("scs, " + serial.port +
                             ", baud115200, bits8, No, bit1, none\x0D");
                }
            }
            if (!settings_->ins_vsm_ip_server_id.empty())
            {
                if (!settings_->ins_vsm_ip_server_keep_open)
                {
                    send("sdio, " + settings_->ins_vsm_ip_server_id +
                         ",  auto, none\x0D");
                    send("siss, " + settings_->ins_vsm_ip_server_id + ",  0\x0D");
                }
            }
            if (!settings_->ins_vsm_serial_port.empty())
            {
                if (!settings_->ins_vsm_serial_keep_open)
                {
                    if (settings_->ins_vsm_serial_port.rfind("COM", 0) == 0)
                        send("scs, " + settings_->ins_vsm_serial_port +
                             ", baud115200, bits8, No, bit1, none\x0D");
                    send("sdio, " + settings_->ins_vsm_serial_port +
                         ",  auto, none\x0D");
                }
            }
            if (!settings_->osnma.keep_open && (settings_->osnma.mode == "loose" ||
                                                settings_->osnma.mode == "strict"))
            {
                std::stringstream ss;
                ss << "sou, off \x0D";
                send(ss.str());

                if (!settings_->osnma.ntp_server.empty())
                {
                    std::stringstream ss;
                    ss << "snc, off \x0D";
                    send(ss.str());
                }
            }

            if (!settings_->login_user.empty() && !settings_->login_password.empty())
                send("logout \x0D");
        }
    }

    void CommunicationCore::connect()
    {
        node_->log(log_level::DEBUG, "Called connect() method");
        node_->log(
            log_level::DEBUG,
            "Started timer for calling connect() method until connection succeeds");

        boost::asio::io_service io;
        boost::posix_time::millisec wait_ms(
            static_cast<uint32_t>(settings_->reconnect_delay_s * 1000));
        if (initializeIo())
        {
            while (running_)
            {
                boost::asio::deadline_timer t(io, wait_ms);

                if (manager_->connect())
                {
                    initializedIo_ = true;
                    break;
                }

                t.wait();
            }
        }

        // Sends commands to the Rx regarding which SBF/NMEA messages it should
        // output
        // and sets all its necessary corrections-related parameters
        if (!settings_->read_from_sbf_log && !settings_->read_from_pcap)
        {
            node_->log(log_level::DEBUG, "Configure Rx.");
            if (settings_->configure_rx)
                configureRx();
        }

        node_->log(log_level::INFO, "Setup complete.");

        node_->log(log_level::DEBUG,
                   "Successully connected. Leaving connect() method");
    }

    [[nodiscard]] bool CommunicationCore::initializeIo()
    {
        bool client = false;
        node_->log(log_level::DEBUG, "Called initializeIo() method");
        if ((settings_->tcp_port != 0) && (!settings_->tcp_ip_server.empty()))
        {
            tcpClient_.reset(new AsyncManager<TcpIo>(node_, &telegramQueue_));
            tcpClient_->setPort(std::to_string(settings_->tcp_port));
            if (!settings_->configure_rx)
                tcpClient_->connect();
            client = true;
        }
        if ((settings_->udp_port != 0) && (!settings_->udp_ip_server.empty()))
        {
            udpClient_.reset(
                new UdpClient(node_, settings_->udp_port, &telegramQueue_));
            client = true;
        }

        switch (settings_->device_type)
        {
        case device_type::TCP:
        {
            manager_.reset(new AsyncManager<TcpIo>(node_, &telegramQueue_));
            break;
        }
        case device_type::SERIAL:
        {
            manager_.reset(new AsyncManager<SerialIo>(node_, &telegramQueue_));
            break;
        }
        case device_type::SBF_FILE:
        {
            manager_.reset(new AsyncManager<SbfFileIo>(node_, &telegramQueue_));
            break;
        }
        case device_type::PCAP_FILE:
        {
            manager_.reset(new AsyncManager<PcapFileIo>(node_, &telegramQueue_));
            break;
        }
        default:
        {
            if (!client || settings_->configure_rx ||
                (settings_->ins_vsm_ros_source == "odometry") ||
                (settings_->ins_vsm_ros_source == "twist"))
            {
                node_->log(log_level::DEBUG, "Unsupported device.");
                return false;
            }
            break;
        }
        }
        return true;
    }

    //! The send() method of AsyncManager class is paramount for this purpose.
    //! Note that std::to_string() is from C++11 onwards only.
    //! Since ROSaic can be launched before booting the Rx, we have to watch out for
    //! escape characters that are sent by the Rx to indicate that it is in upgrade
    //! mode. Those characters would then be mingled with the first command we send
    //! to it in this method and could result in an invalid command. Hence we first
    //! enter command mode via "SSSSSSSSSS".
    void CommunicationCore::configureRx()
    {
        node_->log(log_level::DEBUG, "Called configureRx() method");

        if (!initializedIo_)
        {
            node_->log(log_level::DEBUG,
                       "Called configureRx() method but IO is not initialized.");
            return;
        }

        uint8_t stream = 1;
        // Determining communication mode: TCP vs USB/Serial
        boost::smatch match;
        boost::regex_match(settings_->device, match,
                           boost::regex("(tcp)://(.+):(\\d+)"));
        std::string proto(match[1]);
        mainConnectionPort_ = resetMainConnection();
        node_->log(log_level::INFO,
                   "The connection descriptor is " + mainConnectionPort_);
        streamPort_ = mainConnectionPort_;
        if ((settings_->tcp_port != 0) && (!settings_->tcp_ip_server.empty()))
        {
            streamPort_ = settings_->tcp_ip_server;
            send("siss, " + streamPort_ + ", " +
                 std::to_string(settings_->tcp_port) + ", TCP, " + "\x0D");
            tcpClient_->connect();
        } else if ((settings_->udp_port != 0) && (!settings_->udp_ip_server.empty()))
        {
            streamPort_ = settings_->udp_ip_server;
            std::string destination;
            if (!settings_->udp_unicast_ip.empty())
                destination = settings_->udp_unicast_ip;
            else
                destination = "255.255.255.255";
            send("siss, " + streamPort_ + ", " +
                 std::to_string(settings_->udp_port) + ", UDP, " + destination +
                 "\x0D");
        }

        node_->log(log_level::INFO, "Setting up Rx.");

        std::string pvt_interval = parsing_utilities::convertUserPeriodToRxCommand(
            settings_->polling_period_pvt);

        std::string rest_interval = parsing_utilities::convertUserPeriodToRxCommand(
            settings_->polling_period_rest);

        // Credentials for login
        if (!settings_->login_user.empty() && !settings_->login_password.empty())
        {
            if (string_utilities::containsSpace(settings_->login_password))
                send("login, " + settings_->login_user + ", \"" +
                     settings_->login_password + "\" \x0D");
            else
                send("login, " + settings_->login_user + ", " +
                     settings_->login_password + " \x0D");
        }

        // Turning off all current SBF/NMEA output
        send("sso, all, none, none, off \x0D");
        send("sno, all, none, none, off \x0D");

        // Get Rx capabilities
        send("grc \x0D");
        telegramHandler_.waitForCapabilities();

        // Activate NTP server
        if (settings_->use_gnss_time)
            send("sntp, on \x0D");

        // Setting the datum to be used by the Rx (not the NMEA output though, which
        // only provides MSL and undulation (by default with respect to WGS84), but
        // not ellipsoidal height)
        {
            std::stringstream ss;
            ss << "sgd, " << settings_->datum << "\x0D";
            send(ss.str());
        }

        if ((settings_->septentrio_receiver_type == "ins") || node_->isIns())
        {
            {
                std::stringstream ss;
                ss << "sat, Main, \"" << settings_->ant_type << "\""
                   << "\x0D";
                send(ss.str());
            }

            // Configure Aux1 antenna
            if (settings_->multi_antenna)
            {
                std::stringstream ss;
                ss << "sat, Aux1, \"" << settings_->ant_type << "\""
                   << "\x0D";
                send(ss.str());
            }
        } else if (settings_->septentrio_receiver_type == "gnss")
        {
            // Setting the marker-to-ARP offsets. This comes after the "sso, ...,
            // ReceiverSetup, ..." command, since the latter is only generated when a
            // user-command is entered to change one or more values in the block.
            {
                std::stringstream ss;
                ss << "sao, Main, "
                   << string_utilities::trimDecimalPlaces(settings_->delta_e) << ", "
                   << string_utilities::trimDecimalPlaces(settings_->delta_n) << ", "
                   << string_utilities::trimDecimalPlaces(settings_->delta_u)
                   << ", \"" << settings_->ant_type << "\", "
                   << settings_->ant_serial_nr << "\x0D";
                send(ss.str());
            }

            // Configure Aux1 antenna
            if (settings_->multi_antenna)
            {
                std::stringstream ss;
                ss << "sao, Aux1, " << string_utilities::trimDecimalPlaces(0.0)
                   << ", " << string_utilities::trimDecimalPlaces(0.0) << ", "
                   << string_utilities::trimDecimalPlaces(0.0) << ", \""
                   << settings_->ant_aux1_type << "\", "
                   << settings_->ant_aux1_serial_nr << "\x0D";
                send(ss.str());
            }
        }

        // Configuring the corrections connection
        for (auto ntrip : settings_->rtk.ntrip)
        {
            if (!ntrip.id.empty())
            {
                // First disable any existing NTRIP connection on NTR1
                send("snts, " + ntrip.id + ", off \x0D");
                {
                    std::stringstream ss;
                    ss << "snts, " << ntrip.id << ", Client, " << ntrip.caster
                       << ", " << std::to_string(ntrip.caster_port) << ", "
                       << ntrip.username << ", " << ntrip.password << ", "
                       << ntrip.mountpoint << ", " << ntrip.version << ", "
                       << ntrip.send_gga << " \x0D";
                    send(ss.str());
                }
                if (ntrip.tls)
                {
                    std::stringstream ss;
                    ss << "sntt, " << ntrip.id << ", on, \"" << ntrip.fingerprint
                       << "\" \x0D";
                    send(ss.str());
                } else
                {
                    std::stringstream ss;
                    ss << "sntt, " << ntrip.id << ", off \x0D";
                    send(ss.str());
                }
            }
        }

        for (auto ip_server : settings_->rtk.ip_server)
        {
            if (!ip_server.id.empty())
            // Since the Rx does not have internet (and you will not
            // be able to share it via USB), we need to forward the
            // corrections ourselves, though not on the same port.
            {
                {
                    std::stringstream ss;
                    // In case this IP server was used before, old
                    // configuration is lost of course.
                    ss << "siss, " << ip_server.id << ", "
                       << std::to_string(ip_server.port) << ", TCP2Way \x0D";
                    send(ss.str());
                }
                {
                    std::stringstream ss;
                    ss << "sdio, " << ip_server.id << ", " << ip_server.rtk_standard
                       << ", +SBF+NMEA \x0D";
                    send(ss.str());
                }
                if (ip_server.send_gga != "off")
                {
                    std::string rate = ip_server.send_gga;
                    if (ip_server.send_gga == "auto")
                        rate = "sec1";
                    std::stringstream ss;
                    ss << "sno, Stream" << std::to_string(stream) << ", "
                       << ip_server.id << ", GGA, " << rate << " \x0D";
                    ++stream;
                    send(ss.str());
                }
            }
        }

        for (auto serial : settings_->rtk.serial)
        {
            if (!serial.port.empty())
            {
                if (serial.port.rfind("COM", 0) == 0)
                    send("scs, " + serial.port + ", baud" +
                         std::to_string(serial.baud_rate) +
                         ", bits8, No, bit1, none\x0D");

                std::stringstream ss;
                ss << "sdio, " << serial.port << ", " << serial.rtk_standard
                   << ", +SBF+NMEA \x0D";
                send(ss.str());
                if (serial.send_gga != "off")
                {
                    std::string rate = serial.send_gga;
                    if (serial.send_gga == "auto")
                        rate = "sec1";
                    std::stringstream ss;
                    ss << "sno, Stream" << std::to_string(stream) << ", "
                       << serial.port << ", GGA, " << rate << " \x0D";
                    ++stream;
                    send(ss.str());
                }
            }
        }

        // Setting multi antenna
        if (settings_->multi_antenna)
        {
            if (node_->hasHeading())
                send("sga, MultiAntenna \x0D");
            else
                node_->log(log_level::WARN,
                           "Multi antenna requested but Rx does not support it.");
        } else
        {
            send("sga, none \x0D");
        }

        // Setting the Attitude Determination
        {
            if (settings_->heading_offset >= HEADING_MIN &&
                settings_->heading_offset <= HEADING_MAX &&
                settings_->pitch_offset >= PITCH_MIN &&
                settings_->pitch_offset <= PITCH_MAX)
            {
                std::stringstream ss;
                ss << "sto, "
                   << string_utilities::trimDecimalPlaces(settings_->heading_offset)
                   << ", "
                   << string_utilities::trimDecimalPlaces(settings_->pitch_offset)
                   << " \x0D";
                send(ss.str());
            } else
            {
                node_->log(log_level::ERROR,
                           "Please specify a valid parameter for heading and pitch");
            }
        }

        // Setting the INS-related commands
        if (settings_->septentrio_receiver_type == "ins")
        {
            // IMU orientation
            {
                std::stringstream ss;
                if (settings_->theta_x >= ANGLE_MIN &&
                    settings_->theta_x <= ANGLE_MAX &&
                    settings_->theta_y >= THETA_Y_MIN &&
                    settings_->theta_y <= THETA_Y_MAX &&
                    settings_->theta_z >= ANGLE_MIN &&
                    settings_->theta_z <= ANGLE_MAX)
                {
                    ss << " sio, "
                       << "manual"
                       << ", "
                       << string_utilities::trimDecimalPlaces(settings_->theta_x)
                       << ", "
                       << string_utilities::trimDecimalPlaces(settings_->theta_y)
                       << ", "
                       << string_utilities::trimDecimalPlaces(settings_->theta_z)
                       << " \x0D";
                    send(ss.str());
                } else
                {
                    node_->log(
                        log_level::ERROR,
                        "Please specify a correct value for IMU orientation angles");
                }
            }

            // Setting the INS antenna lever arm offset
            {
                if (settings_->ant_lever_x >= LEVER_ARM_MIN &&
                    settings_->ant_lever_x <= LEVER_ARM_MAX &&
                    settings_->ant_lever_y >= LEVER_ARM_MIN &&
                    settings_->ant_lever_y <= LEVER_ARM_MAX &&
                    settings_->ant_lever_z >= LEVER_ARM_MIN &&
                    settings_->ant_lever_z <= LEVER_ARM_MAX)
                {
                    std::stringstream ss;
                    ss << "sial, "
                       << string_utilities::trimDecimalPlaces(settings_->ant_lever_x)
                       << ", "
                       << string_utilities::trimDecimalPlaces(settings_->ant_lever_y)
                       << ", "
                       << string_utilities::trimDecimalPlaces(settings_->ant_lever_z)
                       << " \x0D";
                    send(ss.str());
                } else
                {
                    node_->log(
                        log_level::ERROR,
                        "Please specify a correct value for x, y and z in the config file under ant_lever_arm");
                }
            }

            // Setting the user defined point offset
            {
                if (settings_->poi_x >= LEVER_ARM_MIN &&
                    settings_->poi_x <= LEVER_ARM_MAX &&
                    settings_->poi_y >= LEVER_ARM_MIN &&
                    settings_->poi_y <= LEVER_ARM_MAX &&
                    settings_->poi_z >= LEVER_ARM_MIN &&
                    settings_->poi_z <= LEVER_ARM_MAX)
                {
                    std::stringstream ss;
                    ss << "sipl, POI1, "
                       << string_utilities::trimDecimalPlaces(settings_->poi_x)
                       << ", "
                       << string_utilities::trimDecimalPlaces(settings_->poi_y)
                       << ", "
                       << string_utilities::trimDecimalPlaces(settings_->poi_z)
                       << " \x0D";
                    send(ss.str());
                } else
                {
                    node_->log(
                        log_level::ERROR,
                        "Please specify a correct value for poi_x, poi_y and poi_z in the config file under poi_lever_arm");
                }
            }

            // Setting the Velocity sensor lever arm offset
            {
                if (settings_->vsm_x >= LEVER_ARM_MIN &&
                    settings_->vsm_x <= LEVER_ARM_MAX &&
                    settings_->vsm_y >= LEVER_ARM_MIN &&
                    settings_->vsm_y <= LEVER_ARM_MAX &&
                    settings_->vsm_z >= LEVER_ARM_MIN &&
                    settings_->vsm_z <= LEVER_ARM_MAX)
                {
                    std::stringstream ss;
                    ss << "sivl, VSM1, "
                       << string_utilities::trimDecimalPlaces(settings_->vsm_x)
                       << ", "
                       << string_utilities::trimDecimalPlaces(settings_->vsm_y)
                       << ", "
                       << string_utilities::trimDecimalPlaces(settings_->vsm_z)
                       << " \x0D";
                    send(ss.str());
                } else
                {
                    node_->log(
                        log_level::ERROR,
                        "Please specify a correct value for vsm_x, vsm_y and vsm_z in the config file under vsm_lever_arm");
                }
            }

            // Setting the INS Solution Reference Point: MainAnt or POI1
            // First disable any existing INS sub-block connection
            {
                std::stringstream ss;
                ss << "sinc, off, all, MainAnt \x0D";
                send(ss.str());
            }

            // INS solution reference point
            {
                std::stringstream ss;
                if (settings_->ins_use_poi)
                {
                    ss << "sinc, on, all, "
                       << "POI1"
                       << " \x0D";
                    send(ss.str());
                } else
                {
                    ss << "sinc, on, all, "
                       << "MainAnt"
                       << " \x0D";
                    send(ss.str());
                }
            }

            // Setting the INS heading
            {
                std::stringstream ss;
                if (settings_->ins_initial_heading == "auto")
                {
                    ss << "siih, " << settings_->ins_initial_heading << " \x0D";
                    send(ss.str());
                } else if (settings_->ins_initial_heading == "stored")
                {
                    ss << "siih, " << settings_->ins_initial_heading << " \x0D";
                    send(ss.str());
                } else
                {
                    node_->log(log_level::ERROR,
                               "Invalid mode specified for ins_initial_heading.");
                }
            }

            // Setting the INS navigation filter
            {
                if (settings_->att_std_dev >= ATTSTD_DEV_MIN &&
                    settings_->att_std_dev <= ATTSTD_DEV_MAX &&
                    settings_->pos_std_dev >= POSSTD_DEV_MIN &&
                    settings_->pos_std_dev <= POSSTD_DEV_MAX)
                {
                    std::stringstream ss;
                    ss << "sism, "
                       << string_utilities::trimDecimalPlaces(settings_->att_std_dev)
                       << ", "
                       << string_utilities::trimDecimalPlaces(settings_->pos_std_dev)
                       << " \x0D";
                    send(ss.str());
                } else
                {
                    node_->log(log_level::ERROR,
                               "Please specify a valid AttStsDev and PosStdDev");
                }
            }
        }

        // OSNMA
        if (settings_->osnma.mode == "loose" || settings_->osnma.mode == "strict")
        {
            std::stringstream ss;
            ss << "sou, " << settings_->osnma.mode << " \x0D";
            send(ss.str());

            if (!settings_->osnma.ntp_server.empty())
            {
                std::stringstream ss;
                ss << "snc, on, " << settings_->osnma.ntp_server << " \x0D";
                send(ss.str());
            } else
            {
                if (settings_->osnma.mode == "strict")
                    node_->log(
                        log_level::ERROR,
                        "OSNMA mode set to strict but no NTP server provided. In Strict mode an NTP server is mandatory!");
            }
        }

        //  Setting up SBF blocks with rx_period_rest
        {
            std::stringstream blocks;
            if (settings_->septentrio_receiver_type == "ins")
            {
                if (settings_->publish_imusetup)
                {
                    blocks << " +IMUSetup";
                }
                if (settings_->publish_velsensorsetup)
                {
                    blocks << " +VelSensorSetup";
                }
            }
            if (settings_->publish_diagnostics)
            {
                blocks << " +ReceiverStatus +QualityInd";
            }
            if (settings_->publish_aimplusstatus)
            {
                blocks << " +RFStatus";
            }
            if (settings_->publish_galauthstatus ||
                (settings_->osnma.mode == "loose") ||
                (settings_->osnma.mode == "strict"))
            {
                blocks << " +GALAuthStatus";
            }

            blocks << " +ReceiverSetup";

            std::stringstream ss;
            ss << "sso, Stream" << std::to_string(stream) << ", " << streamPort_
               << "," << blocks.str() << ", " << rest_interval << "\x0D";
            send(ss.str());
            ++stream;
        }

        // send command to trigger emission of receiver setup
        send("sop, \"\", \"\" \x0D");

        // Setting up NMEA streams
        {
            if (settings_->septentrio_receiver_type == "ins")
                send("snti, auto\x0D");
            else
                send("snti, GP\x0D");

            std::stringstream blocks;
            if (settings_->publish_gpgga)
            {
                blocks << " +GGA";
            }
            if (settings_->publish_gprmc)
            {
                blocks << " +RMC";
            }
            if (settings_->publish_gpgsa)
            {
                blocks << " +GSA";
            }
            if (settings_->publish_gpgsv)
            {
                blocks << " +GSV";
            }

            std::stringstream ss;
            ss << "sno, Stream" << std::to_string(stream) << ", " << streamPort_
               << "," << blocks.str() << ", " << pvt_interval << "\x0D";
            send(ss.str());
            ++stream;
        }

        // Setting up SBF blocks with rx_period_pvt
        {
            std::stringstream blocks;
            if (settings_->use_gnss_time || settings_->publish_gpst)
            {
                blocks << " +ReceiverTime";
            }
            if (settings_->publish_pvtcartesian)
            {
                blocks << " +PVTCartesian";
            }
            if (settings_->publish_pvtgeodetic || settings_->publish_twist ||
                (settings_->publish_gpst &&
                 (settings_->septentrio_receiver_type == "gnss")) ||
                (settings_->publish_navsatfix &&
                 (settings_->septentrio_receiver_type == "gnss")) ||
                (settings_->publish_gpsfix &&
                 (settings_->septentrio_receiver_type == "gnss")) ||
                (settings_->publish_pose &&
                 (settings_->septentrio_receiver_type == "gnss")) ||
                settings_->latency_compensation)
            {
                blocks << " +PVTGeodetic";
            }
            if (settings_->publish_basevectorcart)
            {
                blocks << " +BaseVectorCart";
            }
            if (settings_->publish_basevectorgeod)
            {
                blocks << " +BaseVectorGeod";
            }
            if (settings_->publish_poscovcartesian)
            {
                blocks << " +PosCovCartesian";
            }
            if (settings_->publish_poscovgeodetic ||
                (settings_->publish_navsatfix &&
                 (settings_->septentrio_receiver_type == "gnss")) ||
                (settings_->publish_gpsfix &&
                 (settings_->septentrio_receiver_type == "gnss")) ||
                (settings_->publish_pose &&
                 (settings_->septentrio_receiver_type == "gnss")))
            {
                blocks << " +PosCovGeodetic";
            }
            if (settings_->publish_velcovcartesian)
            {
                blocks << " +VelCovCartesian";
            }
            if (settings_->publish_velcovgeodetic || settings_->publish_twist ||
                (settings_->publish_gpsfix &&
                 (settings_->septentrio_receiver_type == "gnss")))
            {
                blocks << " +VelCovGeodetic";
            }
            if (settings_->publish_atteuler ||
                (settings_->publish_gpsfix &&
                 (settings_->septentrio_receiver_type == "gnss")) ||
                (settings_->publish_pose &&
                 (settings_->septentrio_receiver_type == "gnss")))
            {
                blocks << " +AttEuler";
            }
            if (settings_->publish_attcoveuler ||
                (settings_->publish_gpsfix &&
                 (settings_->septentrio_receiver_type == "gnss")) ||
                (settings_->publish_pose &&
                 (settings_->septentrio_receiver_type == "gnss")))
            {
                blocks << " +AttCovEuler";
            }
            if (settings_->publish_measepoch || settings_->publish_gpsfix)
            {
                blocks << " +MeasEpoch";
            }
            if (settings_->publish_gpsfix)
            {
                blocks << " +ChannelStatus +DOP";
            }
            // Setting SBF output of Rx depending on the receiver type
            // If INS then...
            if (settings_->septentrio_receiver_type == "ins")
            {
                if (settings_->publish_insnavcart ||
                    settings_->publish_localization_ecef ||
                    settings_->publish_tf_ecef)
                {
                    blocks << " +INSNavCart";
                }
                if (settings_->publish_insnavgeod || settings_->publish_navsatfix ||
                    settings_->publish_gpsfix || settings_->publish_pose ||
                    settings_->publish_imu || settings_->publish_localization ||
                    settings_->publish_tf || settings_->publish_twist ||
                    settings_->publish_localization_ecef ||
                    settings_->publish_tf_ecef || settings_->publish_gpst)
                {
                    blocks << " +INSNavGeod";
                }
                if (settings_->publish_exteventinsnavgeod)
                {
                    blocks << " +ExtEventINSNavGeod";
                }
                if (settings_->publish_exteventinsnavcart)
                {
                    blocks << " +ExtEventINSNavCart";
                }
                if (settings_->publish_extsensormeas || settings_->publish_imu)
                {
                    blocks << " +ExtSensorMeas";
                }
            }
            std::stringstream ss;
            ss << "sso, Stream" << std::to_string(stream) << ", " << streamPort_
               << "," << blocks.str() << ", " << pvt_interval << "\x0D";
            send(ss.str());
            ++stream;
        }

        if (settings_->septentrio_receiver_type == "ins")
        {
            if (!settings_->ins_vsm_ip_server_id.empty())
            {
                send("siss, " + settings_->ins_vsm_ip_server_id + ", " +
                     std::to_string(settings_->ins_vsm_ip_server_port) +
                     ", TCP2Way \x0D");
                send("sdio, IPS2, NMEA, none\x0D");
            }
            if (!settings_->ins_vsm_serial_port.empty())
            {
                if (settings_->ins_vsm_serial_port.rfind("COM", 0) == 0)
                    send("scs, " + settings_->ins_vsm_serial_port + ", baud" +
                         std::to_string(settings_->ins_vsm_serial_baud_rate) +
                         ", bits8, No, bit1, none\x0D");
                send("sdio, " + settings_->ins_vsm_serial_port + ", NMEA\x0D");
            }
            if ((settings_->ins_vsm_ros_source == "odometry") ||
                (settings_->ins_vsm_ros_source == "twist"))
            {
                std::string s;
                s = "sdio, " + mainConnectionPort_ + ", NMEA, +NMEA +SBF\x0D";
                send(s);
                nmeaActivated_ = true;
            }
        }

        node_->log(log_level::DEBUG, "Leaving configureRx() method");
    }

    void CommunicationCore::sendVelocity(const std::string& velNmea)
    {
        if (nmeaActivated_)
            manager_.get()->send(velNmea);
    }

    std::string CommunicationCore::resetMainConnection()
    {
        // Escape sequence (escape from correction mode), ensuring that we
        // can send our real commands afterwards... has to be sent multiple times.
        std::string cmd("\x0DSSSSSSSSSS\x0D\x0D");
        telegramHandler_.resetWaitforMainCd();
        manager_.get()->send(cmd);
        std::ignore = telegramHandler_.getMainCd();
        telegramHandler_.resetWaitforMainCd();
        manager_.get()->send(cmd);
        std::ignore = telegramHandler_.getMainCd();
        telegramHandler_.resetWaitforMainCd();
        manager_.get()->send(cmd);
        return telegramHandler_.getMainCd();
    }

    void CommunicationCore::processTelegrams()
    {
        while (running_)
        {
            std::shared_ptr<Telegram> telegram;
            telegramQueue_.pop(telegram);

            if (telegram->type != telegram_type::EMPTY)
                telegramHandler_.handleTelegram(telegram);
        }
    }

    void CommunicationCore::send(const std::string& cmd)
    {
        manager_.get()->send(cmd);
        telegramHandler_.waitForResponse();
    }

} // namespace io