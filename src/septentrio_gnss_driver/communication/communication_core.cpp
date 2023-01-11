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
#include <septentrio_gnss_driver/communication/pcap_reader.hpp>

#ifndef ANGLE_MAX
#define ANGLE_MAX 180
#endif

#ifndef ANGLE_MIN
#define ANGLE_MIN -180
#endif

#ifndef THETA_Y_MAX
#define THETA_Y_MAX 90
#endif

#ifndef THETA_Y_MIN
#define THETA_Y_MIN -90
#endif

#ifndef LEVER_ARM_MAX
#define LEVER_ARM_MAX 100
#endif

#ifndef LEVER_ARM_MIN
#define LEVER_ARM_MIN -100
#endif

#ifndef HEADING_MAX
#define HEADING_MAX 360
#endif

#ifndef HEADING_MIN
#define HEADING_MIN -360
#endif

#ifndef PITCH_MAX
#define PITCH_MAX 90
#endif

#ifndef PITCH_MIN
#define PITCH_MIN -90
#endif

#ifndef ATTSTD_DEV_MIN
#define ATTSTD_DEV_MIN 0
#endif

#ifndef ATTSTD_DEV_MAX
#define ATTSTD_DEV_MAX 5
#endif

#ifndef POSSTD_DEV_MIN
#define POSSTD_DEV_MIN 0
#endif

#ifndef POSSTD_DEV_MAX
#define POSSTD_DEV_MAX 100
#endif

/**
 * @file communication_core.cpp
 * @date 22/08/20
 * @brief Highest-Level view on communication services
 */

//! Mutex to control changes of global variable "g_response_received"
boost::mutex g_response_mutex;
//! Determines whether a command reply was received from the Rx
bool g_response_received;
//! Condition variable complementing "g_response_mutex"
boost::condition_variable g_response_condition;
//! Mutex to control changes of global variable "g_cd_received"
boost::mutex g_cd_mutex;
//! Determines whether the connection descriptor was received from the Rx
bool g_cd_received;
//! Condition variable complementing "g_cd_mutex"
boost::condition_variable g_cd_condition;
//! Whether or not we still want to read the connection descriptor, which we only
//! want in the very beginning to know whether it is IP10, IP11 etc.
bool g_read_cd;
//! Rx TCP port, e.g. IP10 or IP11, to which ROSaic is connected to
std::string g_rx_tcp_port;
//! Since after SSSSSSSSSSS we need to wait for second connection descriptor, we have
//! to count the connection descriptors
uint32_t g_cd_count;

io::CommIo::CommIo(ROSaicNodeBase* node, Settings* settings) :
    node_(node), handlers_(node, settings), settings_(settings), stopping_(false)
{
    g_response_received = false;
    g_cd_received = false;
    g_read_cd = true;
    g_cd_count = 0;
}

io::CommIo::~CommIo()
{
    if (!settings_->read_from_sbf_log && !settings_->read_from_pcap)
    {
        std::string cmd("\x0DSSSSSSSSSSSSSSSSSSS\x0D\x0D");
        manager_.get()->send(cmd);
        send("sdio, " + mainPort_ + ", auto, none\x0D");
        for (auto ntrip : settings_->rtk_settings.ntrip)
        {
            if (!ntrip.id.empty() && !ntrip.keep_open)
            {
                send("snts, " + ntrip.id + ", off \x0D");
            }
        }
        for (auto ip_server : settings_->rtk_settings.ip_server)
        {
            if (!ip_server.id.empty() && !ip_server.keep_open)
            {
                send("sdio, " + ip_server.id + ",  auto, none\x0D");
                send("siss, " + ip_server.id + ",  0\x0D");
            }
        }
        for (auto serial : settings_->rtk_settings.serial)
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

        send("logout \x0D");
    }

    stopping_ = true;
    connectionThread_.join();
}

void io::CommIo::resetMainPort()
{
    // It is imperative to hold a lock on the mutex  "g_cd_mutex" while
    // modifying the variable and "g_cd_received".
    boost::mutex::scoped_lock lock_cd(g_cd_mutex);
    // Escape sequence (escape from correction mode), ensuring that we can send
    // our real commands afterwards...
    std::string cmd("\x0DSSSSSSSSSSSSSSSSSSS\x0D\x0D");
    manager_.get()->send(cmd);
    // We wait for the connection descriptor before we send another command,
    // otherwise the latter would not be processed.
    g_cd_condition.wait(lock_cd, []() { return g_cd_received; });
    g_cd_received = false;
}

void io::CommIo::initializeIo()
{
    node_->log(LogLevel::DEBUG, "Called initializeIo() method");
    boost::smatch match;
    // In fact: smatch is a typedef of match_results<string::const_iterator>
    if (boost::regex_match(settings_->device, match,
                           boost::regex("(tcp)://(.+):(\\d+)")))
    // C++ needs \\d instead of \d: Both mean decimal.
    // Note that regex_match can be used with a smatch object to store results, or
    // without. In any case, true is returned if and only if it matches the
    // !complete! string.
    {
        // The first sub_match (index 0) contained in a match_result always
        // represents the full match within a target sequence made by a regex, and
        // subsequent sub_matches represent sub-expression matches corresponding in
        // sequence to the left parenthesis delimiting the sub-expression in the
        // regex, i.e. $n Perl is equivalent to m[n] in boost regex.
        tcp_host_ = match[2];
        tcp_port_ = match[3];

        serial_ = false;
        connectionThread_ = boost::thread(boost::bind(&CommIo::connect, this));
    } else if (boost::regex_match(settings_->device, match,
                                  boost::regex("(file_name):(/|(?:/[\\w-]+)+.sbf)")))
    {
        serial_ = false;
        settings_->read_from_sbf_log = true;
        settings_->use_gnss_time = true;
        connectionThread_ = boost::thread(
            boost::bind(&CommIo::prepareSBFFileReading, this, match[2]));

    } else if (boost::regex_match(
                   settings_->device, match,
                   boost::regex("(file_name):(/|(?:/[\\w-]+)+.pcap)")))
    {
        serial_ = false;
        settings_->read_from_pcap = true;
        settings_->use_gnss_time = true;
        connectionThread_ = boost::thread(
            boost::bind(&CommIo::preparePCAPFileReading, this, match[2]));

    } else if (boost::regex_match(settings_->device, match,
                                  boost::regex("(serial):(.+)")))
    {
        serial_ = true;
        std::string proto(match[2]);
        std::stringstream ss;
        ss << "Searching for serial port" << proto;
        settings_->device = proto;
        node_->log(LogLevel::DEBUG, ss.str());
        connectionThread_ = boost::thread(boost::bind(&CommIo::connect, this));
    } else
    {
        std::stringstream ss;
        ss << "Device is unsupported. Perhaps you meant 'tcp://host:port' or 'file_name:xxx.sbf' or 'serial:/path/to/device'?";
        node_->log(LogLevel::ERROR, ss.str());
    }
    node_->log(LogLevel::DEBUG, "Leaving initializeIo() method");
}

void io::CommIo::prepareSBFFileReading(std::string file_name)
{
    try
    {
        std::stringstream ss;
        ss << "Setting up everything needed to read from" << file_name;
        node_->log(LogLevel::DEBUG, ss.str());
        initializeSBFFileReading(file_name);
    } catch (std::runtime_error& e)
    {
        std::stringstream ss;
        ss << "CommIo::initializeSBFFileReading() failed for SBF File" << file_name
           << " due to: " << e.what();
        node_->log(LogLevel::ERROR, ss.str());
    }
}

void io::CommIo::preparePCAPFileReading(std::string file_name)
{
    try
    {
        std::stringstream ss;
        ss << "Setting up everything needed to read from " << file_name;
        node_->log(LogLevel::DEBUG, ss.str());
        initializePCAPFileReading(file_name);
    } catch (std::runtime_error& e)
    {
        std::stringstream ss;
        ss << "CommIO::initializePCAPFileReading() failed for SBF File " << file_name
           << " due to: " << e.what();
        node_->log(LogLevel::ERROR, ss.str());
    }
}

void io::CommIo::connect()
{
    node_->log(LogLevel::DEBUG, "Called connect() method");
    node_->log(
        LogLevel::DEBUG,
        "Started timer for calling reconnect() method until connection succeeds");

    boost::asio::io_service io;
    boost::posix_time::millisec wait_ms(
        static_cast<uint32_t>(settings_->reconnect_delay_s * 1000));
    while (!connected_ && !stopping_)
    {
        boost::asio::deadline_timer t(io, wait_ms);
        reconnect();
        t.wait();
    }
    node_->log(LogLevel::DEBUG, "Successully connected. Leaving connect() method");
}

//! The send() method of AsyncManager class is paramount for this purpose.
//! Note that std::to_string() is from C++11 onwards only.
//! Since ROSaic can be launched before booting the Rx, we have to watch out for
//! escape characters that are sent by the Rx to indicate that it is in upgrade mode.
//! Those characters would then be mingled with the first command we send to it in
//! this method and could result in an invalid command. Hence we first enter command
//! mode via "SSSSSSSSSS".
void io::CommIo::configureRx()
{
    node_->log(LogLevel::DEBUG, "Called configureRx() method");
    {
        // wait for connection
        boost::mutex::scoped_lock lock(connection_mutex_);
        connection_condition_.wait(lock, [this]() { return connected_; });
    }

    // Determining communication mode: TCP vs USB/Serial
    unsigned stream = 1;
    boost::smatch match;
    boost::regex_match(settings_->device, match,
                       boost::regex("(tcp)://(.+):(\\d+)"));
    std::string proto(match[1]);
    resetMainPort();
    if (proto == "tcp")
    {
        mainPort_ = g_rx_tcp_port;
    } else
    {
        mainPort_ = settings_->rx_serial_port;
        // After booting, the Rx sends the characters "x?" to all ports, which could
        // potentially mingle with our first command. Hence send a safeguard command
        // "lif", whose potentially false processing is harmless.
        send("lif, Identification \x0D");
    }

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

    // Activate NTP server
    if (settings_->use_gnss_time)
        send("sntp, on \x0D");

    // Setting the datum to be used by the Rx (not the NMEA output though, which only
    // provides MSL and undulation (by default with respect to WGS84), but not
    // ellipsoidal height)
    {
        std::stringstream ss;
        // WGS84 is equivalent to Default and kept for backwards compatibility
        if (settings_->datum == "Default")
            settings_->datum = "WGS84";
        ss << "sgd, " << settings_->datum << "\x0D";
        send(ss.str());
    }

    // Setting up SBF blocks with rx_period_pvt
    {
        std::stringstream blocks;
        if (settings_->use_gnss_time)
        {
            blocks << " +ReceiverTime";
        }
        if (settings_->publish_pvtcartesian)
        {
            blocks << " +PVTCartesian";
        }
        if (settings_->publish_pvtgeodetic || settings_->publish_twist ||
            (settings_->publish_navsatfix &&
             (settings_->septentrio_receiver_type == "gnss")) ||
            (settings_->publish_gpsfix &&
             (settings_->septentrio_receiver_type == "gnss")) ||
            (settings_->publish_pose &&
             (settings_->septentrio_receiver_type == "gnss")))
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
                settings_->publish_localization_ecef || settings_->publish_tf_ecef)
            {
                blocks << " +INSNavCart";
            }
            if (settings_->publish_insnavgeod || settings_->publish_navsatfix ||
                settings_->publish_gpsfix || settings_->publish_pose ||
                settings_->publish_imu || settings_->publish_localization ||
                settings_->publish_tf || settings_->publish_twist ||
                settings_->publish_localization_ecef || settings_->publish_tf_ecef)
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
        ss << "sso, Stream" << std::to_string(stream) << ", " << mainPort_ << ","
           << blocks.str() << ", " << pvt_interval << "\x0D";
        send(ss.str());
        ++stream;
    }
    // Setting up SBF blocks with rx_period_rest
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

        blocks << " +ReceiverSetup";

        std::stringstream ss;
        ss << "sso, Stream" << std::to_string(stream) << ", " << mainPort_ << ","
           << blocks.str() << ", " << rest_interval << "\x0D";
        send(ss.str());
        ++stream;
    }

    // Setting up NMEA streams
    {
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
        ss << "sno, Stream" << std::to_string(stream) << ", " << mainPort_ << ","
           << blocks.str() << ", " << pvt_interval << "\x0D";
        send(ss.str());
        ++stream;
    }

    if ((settings_->septentrio_receiver_type == "ins") ||
        settings_->ins_in_gnss_mode)
    {
        {
            std::stringstream ss;
            ss << "sat, Main, \"" << settings_->ant_type << "\""
               << "\x0D";
            send(ss.str());
        }

        // Configure Aux1 antenna
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
               << string_utilities::trimDecimalPlaces(settings_->delta_u) << ", \""
               << settings_->ant_type << "\", " << settings_->ant_serial_nr
               << "\x0D";
            send(ss.str());
        }

        // Configure Aux1 antenna
        {
            std::stringstream ss;
            ss << "sao, Aux1, " << string_utilities::trimDecimalPlaces(0.0) << ", "
               << string_utilities::trimDecimalPlaces(0.0) << ", "
               << string_utilities::trimDecimalPlaces(0.0) << ", \""
               << settings_->ant_aux1_type << "\", " << settings_->ant_aux1_serial_nr
               << "\x0D";
            send(ss.str());
        }
    }

    // Configuring the corrections connection
    for (auto ntrip : settings_->rtk_settings.ntrip)
    {
        if (!ntrip.id.empty())
        {
            // First disable any existing NTRIP connection on NTR1
            send("snts, " + ntrip.id + ", off \x0D");
            {
                std::stringstream ss;
                ss << "snts, " << ntrip.id << ", Client, " << ntrip.caster << ", "
                   << std::to_string(ntrip.caster_port) << ", " << ntrip.username
                   << ", " << ntrip.password << ", " << ntrip.mountpoint << ", "
                   << ntrip.version << ", " << ntrip.send_gga << " \x0D";
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

    for (auto ip_server : settings_->rtk_settings.ip_server)
    {
        if (!ip_server.id.empty())
        // Since the Rx does not have internet (and you will not
        // be able to share it via USB), we need to forward the
        // corrections ourselves, though not on the same port.
        {
            {
                std::stringstream ss;
                // In case this IP server was used before, old configuration is lost
                // of course.
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
                ss << "sno, Stream" << std::to_string(stream) << ", " << ip_server.id
                   << ", GGA, " << rate << " \x0D";
                ++stream;
                send(ss.str());
            }
        }
    }

    for (auto serial : settings_->rtk_settings.serial)
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
                ss << "sno, Stream" << std::to_string(stream) << ", " << serial.port
                   << ", GGA, " << rate << " \x0D";
                ++stream;
                send(ss.str());
            }
        }
    }

    // Setting multi antenna
    if (settings_->multi_antenna)
    {
        send("sga, MultiAntenna \x0D");
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
            node_->log(LogLevel::ERROR,
                       "Please specify a valid parameter for heading and pitch");
        }
    }

    // Setting the INS-related commands
    if (settings_->septentrio_receiver_type == "ins")
    {
        // IMU orientation
        {
            std::stringstream ss;
            if (settings_->theta_x >= ANGLE_MIN && settings_->theta_x <= ANGLE_MAX &&
                settings_->theta_y >= THETA_Y_MIN &&
                settings_->theta_y <= THETA_Y_MAX &&
                settings_->theta_z >= ANGLE_MIN && settings_->theta_z <= ANGLE_MAX)
            {
                ss << " sio, "
                   << "manual"
                   << ", " << string_utilities::trimDecimalPlaces(settings_->theta_x)
                   << ", " << string_utilities::trimDecimalPlaces(settings_->theta_y)
                   << ", " << string_utilities::trimDecimalPlaces(settings_->theta_z)
                   << " \x0D";
                send(ss.str());
            } else
            {
                node_->log(
                    LogLevel::ERROR,
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
                    LogLevel::ERROR,
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
                   << string_utilities::trimDecimalPlaces(settings_->poi_x) << ", "
                   << string_utilities::trimDecimalPlaces(settings_->poi_y) << ", "
                   << string_utilities::trimDecimalPlaces(settings_->poi_z)
                   << " \x0D";
                send(ss.str());
            } else
            {
                node_->log(
                    LogLevel::ERROR,
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
                   << string_utilities::trimDecimalPlaces(settings_->vsm_x) << ", "
                   << string_utilities::trimDecimalPlaces(settings_->vsm_y) << ", "
                   << string_utilities::trimDecimalPlaces(settings_->vsm_z)
                   << " \x0D";
                send(ss.str());
            } else
            {
                node_->log(
                    LogLevel::ERROR,
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
                node_->log(LogLevel::ERROR,
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
                node_->log(LogLevel::ERROR,
                           "Please specify a valid AttStsDev and PosStdDev");
            }
        }
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
            s = "sdio, " + mainPort_ + ", NMEA, +NMEA +SBF\x0D";
            send(s);
            nmeaActivated_ = true;
        }
    }
    node_->log(LogLevel::DEBUG, "Leaving configureRx() method");
}

void io::CommIo::send(const std::string& cmd)
{
    // It is imperative to hold a lock on the mutex "g_response_mutex" while
    // modifying the variable "g_response_received".
    boost::mutex::scoped_lock lock(g_response_mutex);
    // Determine byte size of cmd and hand over to send() method of manager_
    manager_.get()->send(cmd);
    g_response_condition.wait(lock, []() { return g_response_received; });
    g_response_received = false;
}

void io::CommIo::sendVelocity(const std::string& velNmea)
{
    if (nmeaActivated_)
        manager_.get()->send(velNmea);
}

void io::CommIo::initializeSBFFileReading(std::string file_name)
{
    node_->log(LogLevel::DEBUG, "Calling initializeSBFFileReading() method..");
    std::size_t buffer_size = 8192;
    uint8_t* to_be_parsed;
    to_be_parsed = new uint8_t[buffer_size];
    std::ifstream bin_file(file_name, std::ios::binary);
    std::vector<uint8_t> vec_buf;
    if (bin_file.good())
    {
        /* Reads binary data using streambuffer iterators.
        Copies all SBF file content into bin_data. */
        std::vector<uint8_t> v_buf((std::istreambuf_iterator<char>(bin_file)),
                                   (std::istreambuf_iterator<char>()));
        vec_buf = v_buf;
        bin_file.close();
    } else
    {
        throw std::runtime_error("I could not find your file. Or it is corrupted.");
    }
    // The spec now guarantees that vectors store their elements contiguously.
    to_be_parsed = vec_buf.data();
    std::stringstream ss;
    ss << "Opened and copied over from " << file_name;
    node_->log(LogLevel::DEBUG, ss.str());

    while (!stopping_) // Loop will stop if we are done reading the SBF file
    {
        try
        {
            node_->log(
                LogLevel::DEBUG,
                "Calling read_callback_() method, with number of bytes to be parsed being " +
                    buffer_size);
            handlers_.readCallback(node_->getTime(), to_be_parsed, buffer_size);
        } catch (std::size_t& parsing_failed_here)
        {
            if (to_be_parsed - vec_buf.data() >= vec_buf.size() * sizeof(uint8_t))
            {
                break;
            }
            to_be_parsed = to_be_parsed + parsing_failed_here;
            node_->log(LogLevel::DEBUG,
                       "Parsing_failed_here is " + parsing_failed_here);
            continue;
        }
        if (to_be_parsed - vec_buf.data() >= vec_buf.size() * sizeof(uint8_t))
        {
            break;
        }
        to_be_parsed = to_be_parsed + buffer_size;
    }
    node_->log(LogLevel::DEBUG, "Leaving initializeSBFFileReading() method..");
}

void io::CommIo::initializePCAPFileReading(std::string file_name)
{
    node_->log(LogLevel::DEBUG, "Calling initializePCAPFileReading() method..");
    pcapReader::buffer_t vec_buf;
    pcapReader::PcapDevice device(node_, vec_buf);

    if (!device.connect(file_name.c_str()))
    {
        node_->log(LogLevel::ERROR, "Unable to find file or either it is corrupted");
        return;
    }

    node_->log(LogLevel::INFO, "Reading ...");
    while (device.isConnected() && device.read() == pcapReader::READ_SUCCESS)
        ;
    device.disconnect();

    std::size_t buffer_size = pcapReader::PcapDevice::BUFFSIZE;
    uint8_t* to_be_parsed = new uint8_t[buffer_size];
    to_be_parsed = vec_buf.data();

    while (!stopping_) // Loop will stop if we are done reading the SBF file
    {
        try
        {
            node_->log(
                LogLevel::DEBUG,
                "Calling read_callback_() method, with number of bytes to be parsed being " +
                    buffer_size);
            handlers_.readCallback(node_->getTime(), to_be_parsed, buffer_size);
        } catch (std::size_t& parsing_failed_here)
        {
            if (to_be_parsed - vec_buf.data() >= vec_buf.size() * sizeof(uint8_t))
            {
                break;
            }
            if (!parsing_failed_here)
                parsing_failed_here = 1;

            to_be_parsed = to_be_parsed + parsing_failed_here;
            node_->log(LogLevel::DEBUG,
                       "Parsing_failed_here is " + parsing_failed_here);
            continue;
        }
        if (to_be_parsed - vec_buf.data() >= vec_buf.size() * sizeof(uint8_t))
        {
            break;
        }
        to_be_parsed = to_be_parsed + buffer_size;
    }
    node_->log(LogLevel::DEBUG, "Leaving initializePCAPFileReading() method..");
}