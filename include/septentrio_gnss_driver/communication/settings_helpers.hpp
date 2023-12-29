// *****************************************************************************
//
// © Copyright 2020, Septentrio NV/SA.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//    1. Redistributions of source code must retain the above copyright
//       notice, node list of conditions and the following disclaimer.
//    2. Redistributions in binary form must reproduce the above copyright
//       notice, node list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//    3. Neither the name of the copyright holder nor the names of its
//       contributors may be used to endorse or promote products derived
//       from node software without specific prior written permission.
//
// node SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF node SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// *****************************************************************************

#pragma once

#include "settings.hpp"
#ifdef ROS1
#include <septentrio_gnss_driver/abstraction/typedefs_ros1.hpp>
#endif
#ifdef ROS2
#include <septentrio_gnss_driver/abstraction/typedefs.hpp>
#endif

namespace settings {

    // Check uniqueness of IPS ids
    void checkUniquenssOfIps(ROSaicNodeBase* node, const Settings& settings)
    {
        if (!settings.tcp_ip_server.empty())
        {
            if (settings.tcp_ip_server == settings.udp_ip_server)
                node->log(
                    log_level::ERROR,
                    "stream_device.tcp.ip_server and stream_device.udp.ip_server cannot use the same IP server");
            for (size_t i = 0; i < settings.rtk.ip_server.size(); ++i)
            {
                if (settings.tcp_ip_server == settings.rtk.ip_server[i].id)
                    node->log(
                        log_level::ERROR,
                        "stream_device.tcp.ip_server and rtk_settings.ip_server_" +
                            std::to_string(i + 1) +
                            ".id cannot use the same IP server");
            }
        }
        if (!settings.udp_ip_server.empty())
        {
            for (size_t i = 0; i < settings.rtk.ip_server.size(); ++i)
            {
                if (settings.udp_ip_server == settings.rtk.ip_server[i].id)
                    node->log(
                        log_level::ERROR,
                        "stream_device.udp.ip_server and rtk_settings.ip_server_" +
                            std::to_string(i + 1) +
                            ".id cannot use the same IP server");
            }
        }
        if (settings.rtk.ip_server.size() == 2)
        {
            if (!settings.rtk.ip_server[0].id.empty() &&
                (settings.rtk.ip_server[0].id == settings.rtk.ip_server[1].id))
                node->log(
                    log_level::ERROR,
                    "rtk_settings.ip_server_1.id and rtk_settings.ip_server_2.id cannot use the same IP server");
        }
    }

    // Check uniqueness of IPS ports
    void checkUniquenssOfIpsPorts(ROSaicNodeBase* node, const Settings& settings)
    {
        if (settings.tcp_port != 0)
        {
            if (std::to_string(settings.tcp_port) == settings.device_tcp_port)
                node->log(
                    log_level::ERROR,
                    "stream_device.tcp.port and device port cannot be the same");
            if (settings.tcp_port == settings.udp_port)
                node->log(
                    log_level::ERROR,
                    "stream_device.tcp.port and stream_device.udp.port cannot be the same");
            for (size_t i = 0; i < settings.rtk.ip_server.size(); ++i)
            {
                if (settings.tcp_port == settings.rtk.ip_server[i].port)
                    node->log(log_level::ERROR,
                              "stream_device.tcp.port and rtk_settings.ip_server_" +
                                  std::to_string(i + 1) +
                                  ".port cannot be the same!");
            }
        }
        if (settings.udp_port != 0)
        {
            if (std::to_string(settings.udp_port) == settings.device_tcp_port)
                node->log(
                    log_level::ERROR,
                    "stream_device.udp.port and device port cannot be the same");
            for (size_t i = 0; i < settings.rtk.ip_server.size(); ++i)
            {
                if (settings.udp_port == settings.rtk.ip_server[i].port)
                    node->log(log_level::ERROR,
                              "stream_device.udp.port and rtk_settings.ip_server_" +
                                  std::to_string(i + 1) +
                                  ".port cannot be the same");
            }
        }
        if (settings.rtk.ip_server.size() == 2)
        {
            if ((settings.rtk.ip_server[0].port != 0) &&
                (settings.rtk.ip_server[0].port == settings.rtk.ip_server[1].port))
                node->log(
                    log_level::ERROR,
                    "rtk_settings.ip_server_1.port and rtk_settings.ip_server_2.port cannot be the same");
        }
    }

    // Check uniqueness of IPS id for VSM
    void checkUniquenssOfIpsVsm(ROSaicNodeBase* node, const Settings& settings)
    {
        if (!settings.ins_vsm.ip_server.empty())
        {
            if (!settings.tcp_ip_server.empty() &&
                (settings.tcp_ip_server == settings.ins_vsm.ip_server))
                node->log(
                    log_level::ERROR,
                    "stream_device.tcp.ip_server and ins_vsm.ip_server.id cannot use the same IP server");
            if (!settings.udp_ip_server.empty() &&
                (settings.udp_ip_server == settings.ins_vsm.ip_server))
                node->log(
                    log_level::ERROR,
                    "stream_device.udp.ip_server and ins_vsm.ip_server.id cannot use the same IP server");
            for (size_t i = 0; i < settings.rtk.ip_server.size(); ++i)
            {
                if (settings.ins_vsm.ip_server == settings.rtk.ip_server[i].id)
                    node->log(log_level::ERROR,
                              "ins_vsm.ip_server.id and rtk_settings.ip_server_" +
                                  std::to_string(i + 1) +
                                  ".id cannot use the same IP server");
            }
        }
    }

    // Check uniqueness of IPS port for VSM
    void checkUniquenssOfIpsPortsVsm(ROSaicNodeBase* node, const Settings& settings)
    {
        if (settings.ins_vsm.ip_server_port != 0)
        {
            if (std::to_string(settings.ins_vsm.ip_server_port) ==
                settings.device_tcp_port)
                node->log(
                    log_level::ERROR,
                    "device port  and ins_vsm.ip_server.port cannot be the same");
            if ((settings.tcp_port != 0) &&
                (settings.tcp_port == settings.ins_vsm.ip_server_port))
                node->log(
                    log_level::ERROR,
                    "stream_device.tcp.port and ins_vsm.ip_server.port cannot be the same");
            if ((settings.udp_port != 0) &&
                (settings.udp_port == settings.ins_vsm.ip_server_port))
                node->log(
                    log_level::ERROR,
                    "stream_device.udp.port and ins_vsm.ip_server.port cannot be the same");
            for (size_t i = 0; i < settings.rtk.ip_server.size(); ++i)
            {
                if (settings.ins_vsm.ip_server_port ==
                    settings.rtk.ip_server[i].port)
                    node->log(log_level::ERROR,
                              "ins_vsm.ip_server.port and rtk_settings.ip_server_" +
                                  std::to_string(i + 1) +
                                  ".port cannot use be same");
            }
        }
    }

} // namespace settings