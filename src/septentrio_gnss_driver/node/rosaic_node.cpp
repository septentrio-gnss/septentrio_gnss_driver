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
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, ORs
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// ****************************************************************************

#include <septentrio_gnss_driver/node/rosaic_node.hpp>
/**
 * @file rosaic_node.cpp
 * @date 22/08/20
 * @brief The heart of the ROSaic driver: The ROS node that represents it
 */

rosaic_node::ROSaicNode::ROSaicNode()
{
    ROS_DEBUG("Called ROSaicNode() constructor..");

    // Parameters must be set before initializing IO
    connected_ = false;
    getROSParams();

    // Initializes Connection
    initializeIO();

    // Subscribes to all requested Rx messages by adding entries to the C++ multimap
    // storing the callback handlers and publishes ROS messages
    defineMessages();

    // Sends commands to the Rx regarding which SBF/NMEA messages it should output
    // and sets all its necessary corrections-related parameters
    if (!g_read_from_sbf_log && !g_read_from_pcap)
    {
        boost::mutex::scoped_lock lock(connection_mutex_);
        connection_condition_.wait(lock, [this]() { return connected_; });
        configureRx();
    }

    // Since we already have a ros::Spin() elsewhere, we use waitForShutdown() here
    ros::waitForShutdown();
    ROS_DEBUG("Leaving ROSaicNode() constructor..");
}

//! The send() method of AsyncManager class is paramount for this purpose.
//! Note that std::to_string() is from C++11 onwards only.
//! Since ROSaic can be launched before booting the Rx, we have to watch out for
//! escape characters that are sent by the Rx to indicate that it is in upgrade mode.
//! Those characters would then be mingled with the first command we send to it in
//! this method and could result in an invalid command. Hence we first enter command
//! mode via "SSSSSSSSSS".
void rosaic_node::ROSaicNode::configureRx()
{
    ROS_DEBUG("Called configureRx() method");

    // It is imperative to hold a lock on the mutex "g_response_mutex" while
    // modifying the variable "g_response_received". Same for "g_cd_mutex" and
    // "g_cd_received".
    boost::mutex::scoped_lock lock(g_response_mutex);
    boost::mutex::scoped_lock lock_cd(g_cd_mutex);

    // Determining communication mode: TCP vs USB/Serial
    unsigned stream = 1;
    boost::smatch match;
    boost::regex_match(device_, match, boost::regex("(tcp)://(.+):(\\d+)"));
    std::string proto(match[1]);
    std::string rx_port;
    if (proto == "tcp")
    {
        // Escape sequence (escape from correction mode), ensuring that we can send
        // our real commands afterwards...
        IO.send("\x0DSSSSSSSSSSSSSSSSSSS\x0D\x0D");
        // We wait for the connection descriptor before we send another command,
        // otherwise the latter would not be processed.
        g_cd_condition.wait(lock_cd, []() { return g_cd_received; });
        g_cd_received = false;
        rx_port = g_rx_tcp_port;
    } else
    {
        rx_port = rx_serial_port_;
        // After booting, the Rx sends the characters "x?" to all ports, which could
        // potentially mingle with our first command. Hence send a safeguard command
        // "lif", whose potentially false processing is harmless.
        IO.send("lif, Identification \x0D");
        g_response_condition.wait(lock, []() { return g_response_received; });
        g_response_received = false;
    }
    uint32_t rx_period_pvt =
        parsing_utilities::convertUserPeriodToRxCommand(polling_period_pvt_);
    uint32_t rx_period_rest =
        parsing_utilities::convertUserPeriodToRxCommand(polling_period_rest_);
    std::string pvt_sec_or_msec;
    std::string rest_sec_or_msec;
    if (polling_period_pvt_ == 1000 || polling_period_pvt_ == 2000 ||
        polling_period_pvt_ == 5000 || polling_period_pvt_ == 10000)
        pvt_sec_or_msec = "sec";
    else
        pvt_sec_or_msec = "msec";
    if (polling_period_rest_ == 1000 || polling_period_rest_ == 2000 ||
        polling_period_rest_ == 5000 || polling_period_rest_ == 10000)
        rest_sec_or_msec = "sec";
    else
        rest_sec_or_msec = "msec";

    // Turning off all current SBF/NMEA output
    // Authentication, leaving anonymous mode
    IO.send("login, Tibor, Tibor \x0D");
    g_response_condition.wait(lock, []() { return g_response_received; });
    g_response_received = false;
    IO.send("sso, all, none, none, off \x0D");
    g_response_condition.wait(lock, []() { return g_response_received; });
    g_response_received = false;
    IO.send("sno, all, none, none, off \x0D");
    g_response_condition.wait(lock, []() { return g_response_received; });
    g_response_received = false;

    // Setting the datum to be used by the Rx (not the NMEA output though, which only
    // provides MSL and undulation (by default with respect to WGS84), but not
    // ellipsoidal height)
    {
        std::stringstream ss;
        ss << "sgd, " << datum_ << "\x0D";
        IO.send(ss.str());
    }
    g_response_condition.wait(lock, []() { return g_response_received; });
    g_response_received = false;

    // Setting SBF/NMEA output of Rx depending on the receiver type
    // if GNSS then.....
    if (septentrio_receiver_type_ == "GNSS")
    {
        if (publish_pvtcartesian_ == true)
        {
            std::stringstream ss;
            ss << "sso, Stream" << std::to_string(stream) << ", " << rx_port
            << ", PVTCartesian, " << pvt_sec_or_msec << std::to_string(rx_period_pvt)
            << "\x0D";
            IO.send(ss.str());
            ++stream;
            g_response_condition.wait(lock, []() { return g_response_received; });
            g_response_received = false;
        }
        if (publish_pvtgeodetic_ == true)
        {
            std::stringstream ss;
            ss << "sso, Stream" << std::to_string(stream) << ", " << rx_port
            << ", PVTGeodetic, " << pvt_sec_or_msec << std::to_string(rx_period_pvt)
            << "\x0D";
            IO.send(ss.str());
            ++stream;
            g_response_condition.wait(lock, []() { return g_response_received; });
            g_response_received = false;
        }
        if (publish_poscovcartesian_ == true)
        {
            std::stringstream ss;
            ss << "sso, Stream" << std::to_string(stream) << ", " << rx_port
            << ", PosCovCartesian, " << pvt_sec_or_msec
            << std::to_string(rx_period_pvt) << "\x0D";
            IO.send(ss.str());
            ++stream;
            g_response_condition.wait(lock, []() { return g_response_received; });
            g_response_received = false;
        }
        if (publish_poscovgeodetic_ == true)
        {
            std::stringstream ss;
            ss << "sso, Stream" << std::to_string(stream) << ", " << rx_port
            << ", PosCovGeodetic, " << pvt_sec_or_msec
            << std::to_string(rx_period_pvt) << "\x0D";
            IO.send(ss.str());
            ++stream;
            g_response_condition.wait(lock, []() { return g_response_received; });
            g_response_received = false;
        }
        if (publish_atteuler_ == true)
        {
            std::stringstream ss;
            ss << "sso, Stream" << std::to_string(stream) << ", " << rx_port
            << ", AttEuler, " << rest_sec_or_msec << std::to_string(rx_period_rest)
            << "\x0D";
            IO.send(ss.str());
            ++stream;
            g_response_condition.wait(lock, []() { return g_response_received; });
            g_response_received = false;
        }
        if (publish_attcoveuler_ == true)
        {
            std::stringstream ss;
            ss << "sso, Stream" << std::to_string(stream) << ", " << rx_port
            << ", AttCovEuler, " << rest_sec_or_msec << std::to_string(rx_period_rest)
            << "\x0D";
            IO.send(ss.str());
            ++stream;
            g_response_condition.wait(lock, []() { return g_response_received; });
            g_response_received = false;
        }
    }
    // Setting SBF/NMEA output of Rx depending on the receiver type
    // if INS then.....
    if (septentrio_receiver_type_ == "INS")
    {
        if (publish_insnavcart_ == true)
        {
            std::stringstream ss;
            ss <<"sso, Stream" <<std::to_string(stream) << " , " << rx_port
            << ", INSNavCart, " << pvt_sec_or_msec << std::to_string(rx_period_rest)
            << "\x0D";
            IO.send(ss.str());
            ++stream;
            g_response_condition.wait(lock, []() { return g_response_received; });
            g_response_received = false;
        }
        if (publish_insnavgeod_ == true)
        {
            std::stringstream ss;
            ss <<"sso, Stream" <<std::to_string(stream) << " , " << rx_port
            << ", INSNavGeod, " << pvt_sec_or_msec << std::to_string(rx_period_rest)
            << "\x0D";
            IO.send(ss.str());
            ++stream;
            g_response_condition.wait(lock, []() { return g_response_received; });
            g_response_received = false;
        }
        if (publish_imusetup_ == true)
        {
            std::stringstream ss;
            ss <<"sso, Stream" <<std::to_string(stream) << " , " << rx_port
            << ", IMUSetup, " << pvt_sec_or_msec << std::to_string(rx_period_rest)
            << "\x0D";
            IO.send(ss.str());
            ++stream;
            g_response_condition.wait(lock, []() { return g_response_received; });
            g_response_received = false;
        }
        if (publish_velsensorsetup_ == true)
        {
            std::stringstream ss;
            ss <<"sso, Stream" <<std::to_string(stream) << " , " << rx_port
            << ", VelSensorSetup, " << pvt_sec_or_msec << std::to_string(rx_period_rest)
            << "\x0D";
            IO.send(ss.str());
            ++stream;
            g_response_condition.wait(lock, []() { return g_response_received; });
            g_response_received = false;
        }
        if (publish_exteventinsnavgeod_ == true)
        {
            std::stringstream ss;
            ss <<"sso, Stream" <<std::to_string(stream) << " , " << rx_port
            << ", ExtEventINSNavGeod, " << pvt_sec_or_msec << std::to_string(rx_period_rest)
            << "\x0D";
            IO.send(ss.str());
            ++stream;
            g_response_condition.wait(lock, []() { return g_response_received; });
            g_response_received = false;
        }
        if (publish_exteventinsnavcart_ == true)
        {
            std::stringstream ss;
            ss <<"sso, Stream" <<std::to_string(stream) << " , " << rx_port
            << ", ExtEventINSNavCart, " << pvt_sec_or_msec << std::to_string(rx_period_rest)
            << "\x0D";
            IO.send(ss.str());
            ++stream;
            g_response_condition.wait(lock, []() { return g_response_received; });
            g_response_received = false;
        }
    }
    if ((septentrio_receiver_type_ == "GNSS") || (septentrio_receiver_type_ == "INS"))  
    {
        if (publish_gpgga_ == true)
        {
            std::stringstream ss;

            ss << "sno, Stream" << std::to_string(stream) << ", " << rx_port << ", GGA, "
            << pvt_sec_or_msec << std::to_string(rx_period_pvt) << "\x0D";
            IO.send(ss.str());
            ++stream;
            g_response_condition.wait(lock, []() { return g_response_received; });
            g_response_received = false;
        }
        if (publish_gprmc_ == true)
        {
            std::stringstream ss;

            ss << "sno, Stream" << std::to_string(stream) << ", " << rx_port << ", RMC, "
            << pvt_sec_or_msec << std::to_string(rx_period_pvt) << "\x0D";
            IO.send(ss.str());
            ++stream;
            g_response_condition.wait(lock, []() { return g_response_received; });
            g_response_received = false;
        }
        if (publish_gpgsa_ == true)
        {
            std::stringstream ss;

            ss << "sno, Stream" << std::to_string(stream) << ", " << rx_port << ", GSA, "
            << pvt_sec_or_msec << std::to_string(rx_period_pvt) << "\x0D";
            IO.send(ss.str());
            ++stream;
            g_response_condition.wait(lock, []() { return g_response_received; });
            g_response_received = false;
        }
        if (publish_gpgsv_ == true)
        {
            std::stringstream ss;

            ss << "sno, Stream" << std::to_string(stream) << ", " << rx_port << ", GSV, "
            << rest_sec_or_msec << std::to_string(rx_period_rest) << "\x0D";
            IO.send(ss.str());
            ++stream;
            g_response_condition.wait(lock, []() { return g_response_received; });
            g_response_received = false;
        }
        if (g_publish_gpsfix == true)
        {
            std::stringstream ss;
            ss << "sso, Stream" << std::to_string(stream) << ", " << rx_port
            << ", ChannelStatus, " << pvt_sec_or_msec << std::to_string(rx_period_pvt)
            << "\x0D";
            IO.send(ss.str());
            ++stream;
            g_response_condition.wait(lock, []() { return g_response_received; });
            g_response_received = false;
            ss.str(std::string()); // avoids invoking the std::string constructor
            ss << "sso, Stream" << std::to_string(stream) << ", " << rx_port
            << ", MeasEpoch, " << pvt_sec_or_msec << std::to_string(rx_period_pvt)
            << "\x0D";
            IO.send(ss.str());
            ++stream;
            g_response_condition.wait(lock, []() { return g_response_received; });
            g_response_received = false;
            ss.str(std::string());
            ss << "sso, Stream" << std::to_string(stream) << ", " << rx_port << ", DOP, "
            << pvt_sec_or_msec << std::to_string(rx_period_pvt) << "\x0D";
            IO.send(ss.str());
            ++stream;
            g_response_condition.wait(lock, []() { return g_response_received; });
            g_response_received = false;
            ss.str(std::string());
            ss << "sso, Stream" << std::to_string(stream) << ", " << rx_port
            << ", VelCovGeodetic, " << pvt_sec_or_msec
            << std::to_string(rx_period_pvt) << "\x0D";
            IO.send(ss.str());
            ++stream;
            g_response_condition.wait(lock, []() { return g_response_received; });
            g_response_received = false;
        }  
        if (g_publish_diagnostics == true)
        {
            std::stringstream ss;
            ss << "sso, Stream" << std::to_string(stream) << ", " << rx_port
            << ", ReceiverStatus, " << rest_sec_or_msec
            << std::to_string(rx_period_rest) << "\x0D";
            IO.send(ss.str());
            ++stream;
            g_response_condition.wait(lock, []() { return g_response_received; });
            g_response_received = false;
            ss.str(std::string());
            ss << "sso, Stream" << std::to_string(stream) << ", " << rx_port
            << ", QualityInd, " << rest_sec_or_msec << std::to_string(rx_period_rest)
            << "\x0D";
            IO.send(ss.str());
            ++stream;
            g_response_condition.wait(lock, []() { return g_response_received; });
            g_response_received = false;
            ss.str(std::string());
            ss << "sso, Stream" << std::to_string(stream) << ", " << rx_port
            << ", ReceiverSetup, " << rest_sec_or_msec
            << std::to_string(rx_period_rest) << "\x0D";
            IO.send(ss.str());
            ++stream;
            g_response_condition.wait(lock, []() { return g_response_received; });
            g_response_received = false;
        }
    }

    // Setting the marker-to-ARP offsets. This comes after the "sso, ...,
    // ReceiverSetup, ..." command, since the latter is only generated when a
    // user-command is entered to change one or more values in the block.
    {
        std::stringstream ss;
        ss << "sao, Main, " << string_utilities::trimString(std::to_string(delta_e_))
           << ", " << string_utilities::trimString(std::to_string(delta_n_)) << ", "
           << string_utilities::trimString(std::to_string(delta_u_)) << ", \""
           << ant_type_ << "\", \"" << ant_serial_nr_ << "\", 0 \x0D";
        IO.send(ss.str());
    }
    g_response_condition.wait(lock, []() { return g_response_received; });
    g_response_received = false;

    // Configuring the NTRIP connection
    // First disable any existing NTRIP connection on NTR1
    {
        std::stringstream ss;
        ss << "snts, NTR1, off \x0D";
        IO.send(ss.str());
    }
    g_response_condition.wait(lock, []() { return g_response_received; });
    g_response_received = false;
    if (rx_has_internet_)
    {
        if (mode_ == "off")
        {
        } else if (mode_ == "Client")
        {
            {
                std::stringstream ss;
                ss << "snts, NTR1, " << mode_ << ", " << caster_ << ", "
                   << std::to_string(caster_port_) << ", " << username_ << ", "
                   << password_ << ", " << mountpoint_ << ", " << ntrip_version_
                   << ", " << send_gga_ << " \x0D";
                IO.send(ss.str());
            }
            g_response_condition.wait(lock, []() { return g_response_received; });
            g_response_received = false;
        } else if (mode_ == "Client-Sapcorda")
        {
            {
                std::stringstream ss;
                ss << "snts, NTR1, Client-Sapcorda, , , , , , , , \x0D";
                IO.send(ss.str());
            }
            g_response_condition.wait(lock, []() { return g_response_received; });
            g_response_received = false;
        } else
        {
            ROS_ERROR("Invalid mode specified for NTRIP settings.");
        }
    } else // Since the Rx does not have internet (and you will not be able to share
           // it via USB),
           // we need to forward the corrections ourselves, though not on the same
           // port.
    {
        if (proto == "tcp")
        {
            {
                std::stringstream ss;
                // In case IPS1 was used before, old configuration is lost of course.
                ss << "siss, IPS1, " << std::to_string(rx_input_corrections_tcp_)
                   << ", TCP2Way \x0D";
                IO.send(ss.str());
            }
            g_response_condition.wait(lock, []() { return g_response_received; });
            g_response_received = false;
            {
                std::stringstream ss;
                ss << "sno, Stream" << std::to_string(stream) << ", IPS1, GGA, "
                   << pvt_sec_or_msec << std::to_string(rx_period_pvt) << " \x0D";
                ++stream;
                IO.send(ss.str());
            }
            g_response_condition.wait(lock, []() { return g_response_received; });
            g_response_received = false;
        }
        {
            std::stringstream ss;
            if (proto == "tcp")
            {
                ss << "sdio, IPS1, " << rtcm_version_ << ", +SBF+NMEA \x0D";
            } else
            {
                ss << "sdio, " << rx_input_corrections_serial_ << ", "
                   << rtcm_version_ << ", +SBF+NMEA \x0D";
            }
            IO.send(ss.str());
        }
    }
    ROS_DEBUG("Leaving configureRx() method");

    // Setting the INS receiver related command 
    if (septentrio_receiver_type_ == "INS")
    {
        //IMU orientation 
        //To change angle of oreintation of the receiver
        {
            std::stringstream ss;
            if(sensor_default_ || manual_ == true)
            {
                if(sensor_default_)
                {
                    orientation_mode = "SensorDefault";
                }
                else
                {
                    orientation_mode = "manual";
                }
                if(thetaX_ >= ANGLE_MIN && thetaX_<= ANGLE_MAX && thetaY_ >= THETA_Y_MIN && thetaY_ <= THETA_Y_MAX && 
                    thetaZ_ >= ANGLE_MIN && thetaZ_ <= ANGLE_MAX)
                {
                    ss <<" sio, " <<orientation_mode << ", " << string_utilities::trimString(std::to_string(thetaX_))<< ", " 
                    << string_utilities::trimString(std::to_string(thetaY_))<< ", " 
                    << string_utilities::trimString(std::to_string(thetaZ_))<< " \x0D";
                    IO.send(ss.str());
                }
                else
                {
                    ROS_ERROR("Please specify a correct value for IMU_orientation angles");
                }
            }
        
        }
        g_response_condition.wait(lock, []() { return g_response_received; });
        g_response_received = false;

        // setting the INS antenna lever arm offset
        {
            if(x_>=LEVER_ARM_MIN && x_<=LEVER_ARM_MAX && y_>=LEVER_ARM_MIN && y_<=LEVER_ARM_MAX 
                && z_>=LEVER_ARM_MIN && z_<=LEVER_ARM_MAX)
            {
                std::stringstream ss;
                ss << "sial, " << string_utilities::trimString(std::to_string(x_))
                << ", " << string_utilities::trimString(std::to_string(y_)) << ", "
                << string_utilities::trimString(std::to_string(z_)) << " \x0D";
                IO.send(ss.str());
            }
            else
            {
                ROS_ERROR("Please specify a correct value for X, Y and Z in config file under INS_antenna_lever_arm");
            }
        }
        g_response_condition.wait(lock, []() { return g_response_received; });
        g_response_received = false;

        // setting the user defined point offset
        {
            if(poi_x_>=LEVER_ARM_MIN && poi_x_<=LEVER_ARM_MAX && poi_y_>=LEVER_ARM_MIN && poi_y_<=LEVER_ARM_MAX 
                && poi_z_>=LEVER_ARM_MIN && poi_z_<=LEVER_ARM_MAX)
            {
                std::stringstream ss;
                ss << "sipl, POI1, " << string_utilities::trimString(std::to_string(poi_x_))
                << ", " << string_utilities::trimString(std::to_string(poi_y_)) << ", "
                << string_utilities::trimString(std::to_string(poi_z_)) << " \x0D";
                IO.send(ss.str());
            }
            else
            {
                ROS_ERROR("Please specify a correct value for POI_X, POI_Y and POI_Z in config file under INS_point_of_interest");
            }
        }
        g_response_condition.wait(lock, []() { return g_response_received; });
        g_response_received = false;

        // setting the Velocity sensor lever arm offset
        {
            if(vsm_x_>=LEVER_ARM_MIN && vsm_x_<=LEVER_ARM_MAX && vsm_y_>=LEVER_ARM_MIN && vsm_y_<=LEVER_ARM_MAX 
                && vsm_z_>=LEVER_ARM_MIN && vsm_z_<=LEVER_ARM_MAX)
            {
                std::stringstream ss;
                ss << "sivl, VSM1, " << string_utilities::trimString(std::to_string(vsm_x_))
                << ", " << string_utilities::trimString(std::to_string(vsm_y_)) << ", "
                << string_utilities::trimString(std::to_string(vsm_z_)) << " \x0D";
                IO.send(ss.str());
            }
            else
            {
                ROS_ERROR("Please specify a correct value for VSM_X, VSM_Y and VSM_Z in config file under INS_vel_sensor_lever_arm");
            }
            
        }
        g_response_condition.wait(lock, []() { return g_response_received; });
        g_response_received = false;

        // Setting the Attitude Determination
        //heading
        //pitch
        {
            if(heading_ >= HEADING_MIN && heading_<= HEADING_MAX && pitch_ >= PITCH_MIN && pitch_ <= PITCH_MAX)
            {
                std::stringstream ss;
                ss << "sto, " << string_utilities::trimString(std::to_string(heading_))
                << ", " << string_utilities::trimString(std::to_string(pitch_)) << " \x0D";
                IO.send(ss.str());
            }
            else
            {
                ROS_ERROR("Please specify a valid parameter for heading_offset and pitch_offset");
            }
        }
        g_response_condition.wait(lock, []() { return g_response_received; });
        g_response_received = false;
        
        //Setting the INS Output type
        // INS Solution configuration
        // First disable any existing INS sub-block connection
        {
            std::stringstream ss;
            ss << "sinc, off, all, MainAnt \x0D";
            IO.send(ss.str());
        }
        g_response_condition.wait(lock, []() { return g_response_received; });
        g_response_received = false;

        if(PosStdDev_ || Att_ || AttStdDev_ || Vel_ || VelStdDev_ == true)
        {
            std::stringstream ss;
            {
                if(PosStdDev_)
                {
                    insnavconfig_ = " PosStdDev ";
                }
                if(Att_)
                {
                    insnavconfig_ += "+Att";
                }
                if(AttStdDev_)
                {
                    insnavconfig_ += "+AttStdDev";
                }
                if(Vel_)
                {
                    insnavconfig_ += "+Vel";
                }
                if(VelStdDev_)
                {
                    insnavconfig_ += "+VelStdDev";
                }
                if(output_location_ == "POI1")
                {
                    ss << "sinc, on, " << string_utilities::trimString(insnavconfig_) <<", " <<output_location_ << " \x0D";
                    IO.send(ss.str());
                }
                else if(output_location_ == "MainAnt")
                {
                    ss << "sinc, on, " << string_utilities::trimString(insnavconfig_) <<", " <<output_location_ << " \x0D";
                    IO.send(ss.str());
                }
                else
                {
                    ROS_ERROR("Invalid output location selected !");
                }
            }
            g_response_condition.wait(lock, []() { return g_response_received; });
            g_response_received = false;
        }
        else
        {
            ROS_ERROR("Please specify a valid parameter for ins_output_type");
        }

        //Setting the INS heading
        //auto
        //stored
        {
            std::stringstream ss;
            if(ins_initial_heading_ == "auto")
            {
                ss << "siih, " << ins_initial_heading_ << " \x0D";
                IO.send(ss.str());
            }
            else if (ins_initial_heading_ == "stored")
            {
                ss << "siih, " << ins_initial_heading_ << " \x0D";
                IO.send(ss.str());
            }
            else
            {
                ROS_ERROR("Invalid mode specified for INS_initial_heading.");
            }
            g_response_condition.wait(lock, []() { return g_response_received; });
            g_response_received = false;
        }

        //Setting the INS navigation filter
        {
            if(att_std_dev_ >=ATTSTD_DEV_MIN && att_std_dev_ <= ATTSTD_DEV_MAX && 
            pos_std_dev_ >= POSSTD_DEV_MIN && pos_std_dev_ <= POSSTD_DEV_MAX)
            {
                std::stringstream ss;
                ss << "sism, " << string_utilities::trimString(std::to_string(att_std_dev_)) << ", " << string_utilities::trimString(std::to_string(pos_std_dev_))
                << " \x0D";
                IO.send(ss.str());
            }
            else
            {
                ROS_ERROR("Please specify a valid AttStsDev and PosStdDev");
            }
        }
        g_response_condition.wait(lock, []() { return g_response_received; });
        g_response_received = false;
    }

    // Velocity sensor Measurement

    // {
    //     if (proto == "tcp")
    //     {
    //         {
    //             std::stringstream ss;
    //             ss << "sdio, COM1, NMEA \x0D";
    //             IO.send(ss.str());
    //         }
    //         g_response_condition.wait(lock, []() { return g_response_received; });
    //         g_response_received = false;
    //         // {
    //         //     std::stringstream ss;
    //         //     pssn_vsm_ = "PSSN,VSM";
    //         //     hh_ = "*4D";
    //         //     ss << pssn_vsm_ << ", " << string_utilities::trimString(std::to_string(gps_time_)) << ", " << string_utilities::trimString(std::to_string(x_x_)) << ", "
    //         //         << string_utilities::trimString(std::to_string(y_y_)) << ", " << string_utilities::trimString(std::to_string(std_x_x_)) << ", "
    //         //         << string_utilities::trimString(std::to_string(std_y_y_)) << hh_ << " \x0D";
    //         //     IO.send(ss.str());
    //         //     ROS_INFO("VSM_1 it's working");
    //         // }
    //         // g_response_condition.wait(lock, []() { return g_response_received; });
    //         // g_response_received = false;
    //     }
        
    // }
}
void rosaic_node::ROSaicNode::getROSParams()
{
    // Communication parameters
    g_nh->param("device", device_, std::string("/dev/ttyACM0"));
    getROSInt("serial/baudrate", baudrate_, static_cast<uint32_t>(115200));
    g_nh->param("serial/hw_flow_control", hw_flow_control_, std::string("off"));
    g_nh->param("serial/rx_serial_port", rx_serial_port_, std::string("USB1"));

    g_nh->param("reconnect_delay_s", reconnect_delay_s_, 4.0f);

    // Polling period parameters
    getROSInt("polling_period/pvt", polling_period_pvt_,
              static_cast<uint32_t>(1000));
    if (polling_period_pvt_ != 10 && polling_period_pvt_ != 20 &&
        polling_period_pvt_ != 50 && polling_period_pvt_ != 100 &&
        polling_period_pvt_ != 200 && polling_period_pvt_ != 250 &&
        polling_period_pvt_ != 500 && polling_period_pvt_ != 1000 &&
        polling_period_pvt_ != 2000 && polling_period_pvt_ != 5000 &&
        polling_period_pvt_ != 10000)
    {
        ROS_ERROR(
            "Please specify a valid polling period for PVT-related SBF blocks and NMEA messages.");
    }
    getROSInt("polling_period/rest", polling_period_rest_,
              static_cast<uint32_t>(1000));
    if (polling_period_rest_ != 10 && polling_period_rest_ != 20 &&
        polling_period_rest_ != 50 && polling_period_rest_ != 100 &&
        polling_period_rest_ != 200 && polling_period_rest_ != 250 &&
        polling_period_rest_ != 500 && polling_period_rest_ != 1000 &&
        polling_period_rest_ != 2000 && polling_period_rest_ != 5000 &&
        polling_period_rest_ != 10000)
    {
        ROS_ERROR(
            "Please specify a valid polling period for PVT-unrelated SBF blocks and NMEA messages.");
    }

    // Datum and marker-to-ARP offset
    g_nh->param("datum", datum_, std::string("ETRS89"));
    g_nh->param("ant_type", ant_type_, std::string("Unknown"));
    g_nh->param("ant_serial_nr", ant_serial_nr_, std::string("Unknown"));
    g_nh->param("marker_to_arp/delta_e", delta_e_, 0.0f);
    g_nh->param("marker_to_arp/delta_n", delta_n_, 0.0f);
    g_nh->param("marker_to_arp/delta_u", delta_u_, 0.0f);

    //IMU orientation parameter
    g_nh->param("IMU_orientation/sensor_default", sensor_default_,false);
    g_nh->param("IMU_orientation/manual", manual_,false);
    g_nh->param("IMU_orientation/angles/thetaX", thetaX_,0.0f);
    g_nh->param("IMU_orientation/angles/thetaY", thetaY_,0.0f);
    g_nh->param("IMU_orientation/angles/thetaZ", thetaZ_,0.0f);

    //INS antenna lever arm offset parameter
    g_nh->param("INS_antenna_lever_arm/X", x_, 0.0f);
    g_nh->param("INS_antenna_lever_arm/Y", y_, 0.0f);
    g_nh->param("INS_antenna_lever_arm/Z", z_, 0.0f);

    g_nh->param("receiver_type", septentrio_receiver_type_, std::string("INS"));

    //INS POI ofset paramter
    g_nh->param("INS_point_of_interest/POI_X", poi_x_, 0.0f);
    g_nh->param("INS_point_of_interest/POI_Y", poi_y_, 0.0f);
    g_nh->param("INS_point_of_interest/POI_Z", poi_z_, 0.0f);

    //INS velocity sensor lever arm offset parameter
    g_nh->param("INS_vel_sensor_lever_arm/VSM_X", vsm_x_, 0.0f);
    g_nh->param("INS_vel_sensor_lever_arm/VSM_Y", vsm_y_, 0.0f);
    g_nh->param("INS_vel_sensor_lever_arm/VSM_Z", vsm_z_, 0.0f);
    
    // Attitude Determination parameter
    g_nh->param("Attitude_offset/heading", heading_, 0.0f);
    g_nh->param("Attitude_offset/pitch", pitch_, 0.0f);

    //INS_initial_heading param
    g_nh->param("INS_initial_heading", ins_initial_heading_, std::string("auto"));

    //INS_std_dev_mask
    g_nh->param("INS_std_dev_mask/Att_Std_Dev", att_std_dev_, 0.0f);
    g_nh->param("INS_std_dev_mask/Pos_Std_Dev", pos_std_dev_, 0.0f);

    //INS output  parameters
    g_nh->param("INS_output_type/PosStdDev", PosStdDev_, false);
    g_nh->param("INS_output_type/Att", Att_, false);
    g_nh->param("INS_output_type/AttStdDev", AttStdDev_, false);
    g_nh->param("INS_output_type/Vel", Vel_, false);
    g_nh->param("INS_output_type/VelStdDev", VelStdDev_, false);
    g_nh->param("INS_output_type/output_location", output_location_, std::string("POI1"));

    // // Velocity Sensor Measurement
    // g_nh->param("Velocity_sensor_measurement/time_stamp", gps_time_, 0.0f);
    // g_nh->param("Velocity_sensor_measurement/x_direction", x_x_, 0.0f);
    // g_nh->param("Velocity_sensor_measurement/y_direction", y_y_, 0.0f);
    // g_nh->param("Velocity_sensor_measurement/std_x_direction", std_x_x_, 0.0f);
    // g_nh->param("Velocity_sensor_measurement/std_y_direction", std_y_y_, 0.0f);

    // Correction service parameters
    g_nh->param("ntrip_settings/mode", mode_, std::string("off"));
    g_nh->param("ntrip_settings/caster", caster_, std::string());
    getROSInt("ntrip_settings/caster_port", caster_port_, static_cast<uint32_t>(0));
    g_nh->param("ntrip_settings/username", username_, std::string());
    g_nh->param("ntrip_settings/password", password_, std::string());
    g_nh->param("ntrip_settings/mountpoint", mountpoint_, std::string());
    g_nh->param("ntrip_settings/ntrip_version", ntrip_version_, std::string("v2"));
    g_nh->param("ntrip_settings/send_gga", send_gga_, std::string("auto"));
    g_nh->param("ntrip_settings/rx_has_internet", rx_has_internet_, false);
    g_nh->param("ntrip_settings/rtcm_version", rtcm_version_, std::string("RTCMv3"));
    getROSInt("ntrip_settings/rx_input_corrections_tcp", rx_input_corrections_tcp_,
              static_cast<uint32_t>(28785));
    g_nh->param("ntrip_settings/rx_input_corrections_serial",
                rx_input_corrections_serial_, std::string("USB2"));

    // Publishing parameters, the others remained global since they need to be
    // accessed in the callbackhandlers.hpp file
    g_nh->param("publish/gpgga", publish_gpgga_, true);
    g_nh->param("publish/gprmc", publish_gprmc_, true);
    g_nh->param("publish/gpgsa", publish_gpgsa_, true);
    g_nh->param("publish/gpgsv", publish_gpgsv_, true);
    g_nh->param("publish/pvtcartesian", publish_pvtcartesian_, true);
    g_nh->param("publish/pvtgeodetic", publish_pvtgeodetic_, true);
    g_nh->param("publish/poscovcartesian", publish_poscovcartesian_, true);
    g_nh->param("publish/poscovgeodetic", publish_poscovgeodetic_, true);
    g_nh->param("publish/atteuler", publish_atteuler_, true);
    g_nh->param("publish/attcoveuler", publish_attcoveuler_, true);
    g_nh->param("publish/insnavcart", publish_insnavcart_, true);
    g_nh->param("publish/insnavgeod", publish_insnavgeod_, true);
    g_nh->param("publish/imusetup", publish_imusetup_, true);
    g_nh->param("publish/velsensorsetup", publish_velsensorsetup_, true);
    g_nh->param("publish/exteventinsnavgeod", publish_exteventinsnavgeod_, true);
    g_nh->param("publish/exteventinsnavcart", publish_exteventinsnavcart_, true);

    // To be implemented: RTCM, setting datum, raw data settings, PPP, SBAS, fix
    // mode...
    ROS_DEBUG("Finished getROSParams() method");
}

void rosaic_node::ROSaicNode::initializeIO()
{
    ROS_DEBUG("Called initializeIO() method");
    boost::smatch match;
    // In fact: smatch is a typedef of match_results<string::const_iterator>
    if (boost::regex_match(device_, match, boost::regex("(tcp)://(.+):(\\d+)")))
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
        g_read_from_sbf_log = false;
        g_read_from_pcap = false;
        boost::thread temporary_thread(boost::bind(&ROSaicNode::connect, this));
        temporary_thread.detach();
    } else if (boost::regex_match(device_, match,
                                  boost::regex("(file_name):(/|(?:/[\\w-]+)+.sbf)")))
    {
        serial_ = false;
        g_read_from_sbf_log = true;
        g_read_from_pcap = false;
        g_unix_time.sec = 0;
        g_unix_time.nsec = 0;
        boost::thread temporary_thread(
            boost::bind(&ROSaicNode::prepareSBFFileReading, this, match[2]));
        temporary_thread.detach();

    } else if (boost::regex_match(
                   device_, match,
                   boost::regex("(file_name):(/|(?:/[\\w-]+)+.pcap)")))
    {
        serial_ = false;
        g_read_from_sbf_log = false;
        g_read_from_pcap = true;
        g_unix_time.sec = 0;
        g_unix_time.nsec = 0;
        boost::thread temporary_thread(
            boost::bind(&ROSaicNode::preparePCAPFileReading, this, match[2]));
        temporary_thread.detach();

    } else if (boost::regex_match(device_, match, boost::regex("(serial):(.+)")))
    {
        serial_ = true;
        g_read_from_sbf_log = false;
        g_read_from_pcap = false;
        std::string proto(match[2]);
        std::stringstream ss;
        ss << "Searching for serial port" << proto;
        ROS_DEBUG("%s", ss.str().c_str());
        boost::thread temporary_thread(boost::bind(&ROSaicNode::connect, this));
        temporary_thread.detach();
    } else
    {
        std::stringstream ss;
        ss << "Device is unsupported. Perhaps you meant 'tcp://host:port' or 'file_name:xxx.sbf' or 'serial:/path/to/device'?";
        ROS_ERROR("%s", ss.str().c_str());
    }
    ROS_DEBUG("Leaving initializeIO() method");
}

void rosaic_node::ROSaicNode::prepareSBFFileReading(std::string file_name)
{
    try
    {
        std::stringstream ss;
        ss << "Setting up everything needed to read from" << file_name;
        ROS_DEBUG("%s", ss.str().c_str());
        IO.initializeSBFFileReading(file_name);
    } catch (std::runtime_error& e)
    {
        std::stringstream ss;
        ss << "Comm_IO::initializeSBFFileReading() failed for SBF File" << file_name
           << " due to: " << e.what();
        ROS_ERROR("%s", ss.str().c_str());
    }
}

void rosaic_node::ROSaicNode::preparePCAPFileReading(std::string file_name)
{
    try
    {
        std::stringstream ss;
        ss << "Setting up everything needed to read from " << file_name;
        ROS_DEBUG("%s", ss.str().c_str());
        IO.initializePCAPFileReading(file_name);
    } catch (std::runtime_error& e)
    {
        std::stringstream ss;
        ss << "CommIO::initializePCAPFileReading() failed for SBF File " << file_name
           << " due to: " << e.what();
        ROS_ERROR("%s", ss.str().c_str());
    }
}

void rosaic_node::ROSaicNode::connect()
{
    ROS_DEBUG("Called connect() method");
    ROS_DEBUG(
        "Setting ROS timer for calling reconnect() method until connection succeds");
    reconnect_timer_ = g_nh->createTimer(ros::Duration(reconnect_delay_s_),
                                         &ROSaicNode::reconnect, this);
    reconnect_timer_.start();
    ROS_DEBUG(
        "Started ROS timer for calling reconnect() method until connection succeds");
    ros::spin();
    ROS_DEBUG("Leaving connect() method"); // This will never be output since
                                           // ros::spin() is on the line above.
}

//! In serial mode (not USB, since the Rx port is then called USB1 or USB2), please
//! ensure that you are connected to the Rx's COM1, COM2 or COM3 port, !if! you
//! employ UART hardware flow control.
void rosaic_node::ROSaicNode::reconnect(const ros::TimerEvent& event)
{
    ROS_DEBUG("Called reconnect() method");
    if (connected_ == true)
    {
        reconnect_timer_.stop();
        ROS_DEBUG("Stopped ROS timer since successully connected.");
    } else
    {
        if (serial_)
        {
            bool initialize_serial_return = false;
            try
            {
                ROS_INFO("Connecting serially to device %s, targeted baudrate: %u",
                         device_.c_str(), baudrate_);
                initialize_serial_return =
                    IO.initializeSerial(device_, baudrate_, hw_flow_control_);
            } catch (std::runtime_error& e)
            {
                {
                    std::stringstream ss;
                    ss << "IO.initializeSerial() failed for device " << device_
                       << " due to: " << e.what();
                    ROS_ERROR("%s", ss.str().c_str());
                }
            }
            if (initialize_serial_return)
            {
                boost::mutex::scoped_lock lock(connection_mutex_);
                connected_ = true;
                lock.unlock();
                connection_condition_.notify_one();
            }
        } else
        {
            bool initialize_tcp_return = false;
            try
            {
                ROS_INFO("Connecting to tcp://%s:%s ...", tcp_host_.c_str(),
                         tcp_port_.c_str());
                initialize_tcp_return = IO.initializeTCP(tcp_host_, tcp_port_);
            } catch (std::runtime_error& e)
            {
                {
                    std::stringstream ss;
                    ss << "IO.initializeTCP() failed for host " << tcp_host_
                       << " on port " << tcp_port_ << " due to: " << e.what();
                    ROS_ERROR("%s", ss.str().c_str());
                }
            }
            if (initialize_tcp_return)
            {
                boost::mutex::scoped_lock lock(connection_mutex_);
                connected_ = true;
                lock.unlock();
                connection_condition_.notify_one();
            }
        }
    }
    ROS_DEBUG("Leaving reconnect() method");
}

//! initializeSerial is not self-contained: The for loop in Callbackhandlers' handle
//! method would never open a specific handler unless the handler is added
//! (=inserted) to the C++ map via this function. This way, the specific handler can
//! be called, in which in turn RxMessage's read() method is called, which publishes
//! the ROS message depending on the type of receiver
//! GNSS or INS.
void rosaic_node::ROSaicNode::defineMessages()
{
    ROS_DEBUG("Called defineMessages() method");
    if ((septentrio_receiver_type_ == "GNSS") || (septentrio_receiver_type_ == "INS"))  
    {
        if (publish_gpgga_ == true)
        {
            IO.handlers_.callbackmap_ =
                IO.getHandlers().insert<septentrio_gnss_driver::Gpgga>("$GPGGA");
        }
        if (publish_gprmc_ == true)
        {
            IO.handlers_.callbackmap_ =
                IO.getHandlers().insert<septentrio_gnss_driver::Gprmc>("$GPRMC");
        }
        if (publish_gpgsa_ == true)
        {
            IO.handlers_.callbackmap_ =
                IO.getHandlers().insert<septentrio_gnss_driver::Gpgsa>("$GPGSA");
        }
        if (publish_gpgsv_ == true)
        {
            IO.handlers_.callbackmap_ =
                IO.getHandlers().insert<septentrio_gnss_driver::Gpgsv>("$GPGSV");
            IO.handlers_.callbackmap_ =
                IO.getHandlers().insert<septentrio_gnss_driver::Gpgsv>("$GLGSV");
            IO.handlers_.callbackmap_ =
                IO.getHandlers().insert<septentrio_gnss_driver::Gpgsv>("$GAGSV");
            IO.handlers_.callbackmap_ =
                IO.getHandlers().insert<septentrio_gnss_driver::Gpgsv>("$GBGSV");
        }
    }
    if (publish_pvtcartesian_ == true)
    {
        IO.handlers_.callbackmap_ =
            IO.getHandlers().insert<septentrio_gnss_driver::PVTCartesian>("4006");
    }
    if (publish_pvtgeodetic_ == true)
    {
        IO.handlers_.callbackmap_ =
            IO.getHandlers().insert<septentrio_gnss_driver::PVTGeodetic>("4007");
    }
    if (publish_poscovcartesian_ == true)
    {
        IO.handlers_.callbackmap_ =
            IO.getHandlers().insert<septentrio_gnss_driver::PosCovCartesian>("5905");
    }
    if (publish_poscovgeodetic_ == true)
    {
        IO.handlers_.callbackmap_ =
            IO.getHandlers().insert<septentrio_gnss_driver::PosCovGeodetic>("5906");
    }
    if (publish_atteuler_ == true)
    {
        IO.handlers_.callbackmap_ =
            IO.getHandlers().insert<septentrio_gnss_driver::AttEuler>("5938");
    }
    if (publish_attcoveuler_ == true)
    {
        IO.handlers_.callbackmap_ =
            IO.getHandlers().insert<septentrio_gnss_driver::AttCovEuler>("5939");
    }
    //INS related SBF blocks
    if (publish_insnavcart_ == true)
    {
        IO.handlers_.callbackmap_ = 
            IO.getHandlers().insert<septentrio_gnss_driver::INSNavCart>("4225");
    }
    if (publish_insnavgeod_ == true)
    {
        IO.handlers_.callbackmap_ = 
            IO.getHandlers().insert<septentrio_gnss_driver::INSNavGeod>("4226");
    }
    if (publish_imusetup_ == true)
    {
        IO.handlers_.callbackmap_ = 
            IO.getHandlers().insert<septentrio_gnss_driver::IMUSetup>("4224");
    }
    if (publish_velsensorsetup_ == true)
    {
        IO.handlers_.callbackmap_ = 
            IO.getHandlers().insert<septentrio_gnss_driver::VelSensorSetup>("4244");
    }
    if (publish_exteventinsnavgeod_ == true)
    {
        IO.handlers_.callbackmap_ = 
            IO.getHandlers().insert<septentrio_gnss_driver::ExtEventINSNavGeod>("4230");
    }
    if (publish_exteventinsnavcart_ == true)
    {
        IO.handlers_.callbackmap_ = 
            IO.getHandlers().insert<septentrio_gnss_driver::ExtEventINSNavCart>("4229");
    }
    if ((septentrio_receiver_type_ == "GNSS") || (septentrio_receiver_type_ == "INS"))
    {
        if (g_publish_gpst == true)
        {
            IO.handlers_.callbackmap_ = IO.getHandlers().insert<int32_t>("GPST");
        }
    }
    if(septentrio_receiver_type_ == "GNSS")
    {
        if (g_publish_navsatfix == true)
        {
            if (publish_pvtgeodetic_ == false || publish_poscovgeodetic_ == false)
            {
                ROS_ERROR(
                    "For a proper NavSatFix message, please set the publish/pvtgeodetic and the publish/poscovgeodetic ROSaic parameters both to true.");
            }
            IO.handlers_.callbackmap_ =
                IO.getHandlers().insert<sensor_msgs::NavSatFix>("NavSatFix");
        }
    }
    if(septentrio_receiver_type_ == "INS")
    {
        if (g_publish_navsatfix == true)
        {
            if (publish_insnavgeod_ == false)
            {
                ROS_ERROR(
                    "For a proper NavSatFix message, please set the publish/insnavgeod to true.");
            }
            IO.handlers_.callbackmap_ =
                IO.getHandlers().insert<sensor_msgs::NavSatFix>("INSNavSatFix");
        }
    }
    if(septentrio_receiver_type_ == "GNSS")
    {
        if (g_publish_gpsfix == true)
        {
            if (publish_pvtgeodetic_ == false || publish_poscovgeodetic_ == false)
            {
                ROS_ERROR(
                    "For a proper GPSFix message, please set the publish/pvtgeodetic and the publish/poscovgeodetic ROSaic parameters both to true.");
            }
            IO.handlers_.callbackmap_ =
                IO.getHandlers().insert<gps_common::GPSFix>("GPSFix");
            // The following blocks are never published, yet are needed for the
            // construction of the GPSFix message, hence we have empty callbacks.
            IO.handlers_.callbackmap_ =
                IO.getHandlers().insert<int32_t>("4013"); // ChannelStatus block
            IO.handlers_.callbackmap_ =
                IO.getHandlers().insert<int32_t>("4027"); // MeasEpoch block
            IO.handlers_.callbackmap_ =
                IO.getHandlers().insert<int32_t>("4001"); // DOP block
            IO.handlers_.callbackmap_ =
                IO.getHandlers().insert<int32_t>("5908"); // VelCovGeodetic block
        }
    }
    if(septentrio_receiver_type_ == "INS")
    {
        if (g_publish_gpsfix == true)
        {
            if (publish_insnavgeod_ == false)
            {
                ROS_ERROR(
                        "For a proper NavSatFix message, please set the publish/insnavgeod to true.");
            }
            IO.handlers_.callbackmap_ =
                IO.getHandlers().insert<gps_common::GPSFix>("INSGPSFix");
            // The following blocks are never published, yet are needed for the
            // construction of the GPSFix message, hence we have empty callbacks.
            IO.handlers_.callbackmap_ =
                IO.getHandlers().insert<int32_t>("4013"); // ChannelStatus block
            IO.handlers_.callbackmap_ =
                IO.getHandlers().insert<int32_t>("4027"); // MeasEpoch block
            IO.handlers_.callbackmap_ =
                IO.getHandlers().insert<int32_t>("4001"); // DOP block
        }
    }
    if(septentrio_receiver_type_ == "GNSS")
    {
        if (g_publish_pose == true)
        {
            if (publish_pvtgeodetic_ == false || publish_poscovgeodetic_ == false ||
                publish_atteuler_ == false || publish_attcoveuler_ == false)
            {
                ROS_ERROR(
                    "For a proper PoseWithCovarianceStamped message, please set the publish/pvtgeodetic, publish/poscovgeodetic, publish_atteuler and publish_attcoveuler ROSaic parameters all to true.");
            }
            IO.handlers_.callbackmap_ =
                IO.getHandlers().insert<geometry_msgs::PoseWithCovarianceStamped>(
                    "PoseWithCovarianceStamped");
        }
    }
    if(septentrio_receiver_type_ == "INS")
    {
        if (g_publish_pose == true)
        {
            if (publish_insnavgeod_ == false)
            {
                ROS_ERROR(
                    "For a proper PoseWithCovarianceStamped message, please set the publish/insnavgeod to true.");
            }
            IO.handlers_.callbackmap_ =
                IO.getHandlers().insert<geometry_msgs::PoseWithCovarianceStamped>(
                    "INSPoseWithCovarianceStamped");
        }
    }
    if ((septentrio_receiver_type_ == "GNSS") || (septentrio_receiver_type_ == "INS"))  
    {
        if (g_publish_diagnostics == true)
        {
            IO.handlers_.callbackmap_ =
                IO.getHandlers().insert<diagnostic_msgs::DiagnosticArray>(
                    "DiagnosticArray");
            IO.handlers_.callbackmap_ =
                IO.getHandlers().insert<int32_t>("4014"); // ReceiverStatus block
            IO.handlers_.callbackmap_ =
                IO.getHandlers().insert<int32_t>("4082"); // QualityInd block
            IO.handlers_.callbackmap_ =
                IO.getHandlers().insert<int32_t>("5902"); // ReceiverSetup block
        }
    }
    
    // so on and so forth...
    ROS_DEBUG("Leaving defineMessages() method");
}

//! If true, the ROS message headers' unix time field is constructed from the TOW (in
//! the SBF case) and UTC (in the NMEA case) data. If false, times are constructed
//! within the driver via time(NULL) of the \<ctime\> library.
bool g_use_gnss_time;
//! Whether or not to publish the sensor_msgs::TimeReference message with GPST
bool g_publish_gpst;
//! Whether or not to publish the sensor_msgs::NavSatFix message
bool g_publish_navsatfix;
//! Whether or not to publish the gps_common::GPSFix message
bool g_publish_gpsfix;
//! Whether or not to publish the geometry_msgs::PoseWithCovarianceStamped message
bool g_publish_pose;
//! Whether or not to publish the diagnostic_msgs::DiagnosticArray message
bool g_publish_diagnostics;
//! The frame ID used in the header of every published ROS message
std::string g_frame_id;
//! The frame ID used in the header of every published ROS INS message
std::string g_frame_id_ins;
//! The number of leap seconds that have been inserted into the UTC time
uint32_t g_leap_seconds;
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
//! For GPSFix: Whether the ChannelStatus block of the current epoch has arrived or
//! not
bool g_channelstatus_has_arrived_gpsfix;
//! For GPSFix: Whether the MeasEpoch block of the current epoch has arrived or not
bool g_measepoch_has_arrived_gpsfix;
//! For GPSFix: Whether the DOP block of the current epoch has arrived or not
bool g_dop_has_arrived_gpsfix;
//! For GPSFix: Whether the PVTGeodetic block of the current epoch has arrived or not
bool g_pvtgeodetic_has_arrived_gpsfix;
//! For GPSFix: Whether the PosCovGeodetic block of the current epoch has arrived or
//! not
bool g_poscovgeodetic_has_arrived_gpsfix;
//! For GPSFix: Whether the VelCovGeodetic block of the current epoch has arrived or
//! not
bool g_velcovgeodetic_has_arrived_gpsfix;
//! For GPSFix: Whether the AttEuler block of the current epoch has arrived or not
bool g_atteuler_has_arrived_gpsfix;
//! For GPSFix: Whether the AttCovEuler block of the current epoch has arrived or not
bool g_attcoveuler_has_arrived_gpsfix;
//! For GPSFix: Whether the INSNavGeod block of the current epoch has arrived or not
bool g_insnavgeod_has_arrived_gpsfix;
//! For NavSatFix: Whether the PVTGeodetic block of the current epoch has arrived or
//! not
bool g_pvtgeodetic_has_arrived_navsatfix;
//! For NavSatFix: Whether the PosCovGeodetic block of the current epoch has arrived
//! or not
bool g_poscovgeodetic_has_arrived_navsatfix;
//! For NavSatFix: Whether the INSNavGeod block of the current epoch has arrived
//! or not
bool g_insnavgeod_has_arrived_navsatfix;
//! For PoseWithCovarianceStamped: Whether the PVTGeodetic block of the current epoch
//! has arrived or not
bool g_pvtgeodetic_has_arrived_pose;
//! For PoseWithCovarianceStamped: Whether the PosCovGeodetic block of the current
//! epoch has arrived or not
bool g_poscovgeodetic_has_arrived_pose;
//! For PoseWithCovarianceStamped: Whether the AttEuler block of the current epoch
//! has arrived or not
bool g_atteuler_has_arrived_pose;
//! For PoseWithCovarianceStamped: Whether the AttCovEuler block of the current epoch
//! has arrived or not
bool g_attcoveuler_has_arrived_pose;
//! For PoseWithCovarianceStamped: Whether the INSNavGeod block of the current epoch
//! has arrived or not
bool g_insnavgeod_has_arrived_pose;
//! For DiagnosticArray: Whether the ReceiverStatus block of the current epoch has
//! arrived or not
bool g_receiverstatus_has_arrived_diagnostics;
//! For DiagnosticArray: Whether the QualityInd block of the current epoch has
//! arrived or not
bool g_qualityind_has_arrived_diagnostics;
//! When reading from an SBF file, the ROS publishing frequency is governed by the
//! time stamps found in the SBF blocks therein.
ros::Time g_unix_time;
//! Whether or not we are reading from an SBF file
bool g_read_from_sbf_log;
//! Whether or not we are reading from a PCAP file
bool g_read_from_pcap;
//! A C++ map for keeping track of the SBF blocks necessary to construct the GPSFix
//! ROS message
std::map<std::string, uint32_t> g_GPSFixMap;
//! A C++ map for keeping track of the SBF blocks necessary to construct the
//! NavSatFix ROS message
std::map<std::string, uint32_t> g_NavSatFixMap;
//! A C++ map for keeping track of SBF blocks necessary to construct the
//! PoseWithCovarianceStamped ROS message
std::map<std::string, uint32_t> g_PoseWithCovarianceStampedMap;
//! A C++ map for keeping track of SBF blocks necessary to construct the
//! DiagnosticArray ROS message
std::map<std::string, uint32_t> g_DiagnosticArrayMap;
//! Node Handle for the ROSaic node
//! You must initialize the NodeHandle in the "main" function (or in any method
//! called indirectly or directly by the main function). One can declare a pointer to
//! the NodeHandle to be a global variable and then initialize it afterwards only...
boost::shared_ptr<ros::NodeHandle> g_nh;
//! Queue size for ROS publishers
const uint32_t g_ROS_QUEUE_SIZE = 1;
std::string septentrio_receiver_type_;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "septentrio_gnss");
    g_nh.reset(new ros::NodeHandle("~"));
    g_nh->param("use_gnss_time", g_use_gnss_time, true);
    g_nh->param("frame_id", g_frame_id, (std::string) "gnss");
    g_nh->param("ins_frame_id", g_frame_id_ins, (std::string) "INS");
    g_nh->param("publish/gpst", g_publish_gpst, true);
    g_nh->param("publish/navsatfix", g_publish_navsatfix, true);
    g_nh->param("publish/gpsfix", g_publish_gpsfix, true);
    g_nh->param("publish/pose", g_publish_pose, true);
    g_nh->param("publish/diagnostics", g_publish_diagnostics, true);
    rosaic_node::getROSInt("leap_seconds", g_leap_seconds,
                           static_cast<uint32_t>(18));

    g_response_received = false;
    g_cd_received = false;
    g_read_cd = true;
    g_cd_count = 0;
    g_channelstatus_has_arrived_gpsfix = false;
    g_measepoch_has_arrived_gpsfix = false;
    g_dop_has_arrived_gpsfix = false;
    g_pvtgeodetic_has_arrived_gpsfix = false;
    g_pvtgeodetic_has_arrived_navsatfix = false;
    g_pvtgeodetic_has_arrived_pose = false;
    g_poscovgeodetic_has_arrived_gpsfix = false;
    g_poscovgeodetic_has_arrived_navsatfix = false;
    g_poscovgeodetic_has_arrived_pose = false;
    g_velcovgeodetic_has_arrived_gpsfix = false;
    g_atteuler_has_arrived_gpsfix = false;
    g_atteuler_has_arrived_pose = false;
    g_attcoveuler_has_arrived_gpsfix = false;
    g_attcoveuler_has_arrived_pose = false;
    g_receiverstatus_has_arrived_diagnostics = false;
    g_qualityind_has_arrived_diagnostics = false;
    g_insnavgeod_has_arrived_gpsfix = false;
    g_insnavgeod_has_arrived_navsatfix = false;
    g_insnavgeod_has_arrived_pose = false;

    // The info logging level seems to be default, hence we modify log level
    // momentarily.. The following is the C++ version of
    // rospy.init_node('my_ros_node', log_level=rospy.DEBUG)
    if (ros::console::set_logger_level(
            ROSCONSOLE_DEFAULT_NAME,
            ros::console::levels::Debug)) // debug is lowest level, shows everything
        ros::console::notifyLoggerLevelsChanged();

    rosaic_node::ROSaicNode
        rx_node; // This launches everything we need, in theory :)
    return 0;
}
