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

#include "septentrio_gnss_driver/communication/pcap_reader.hpp"

#include <chrono>
#include <net/ethernet.h>
#include <netinet/ip.h>
#include <netinet/tcp.h>
#include <septentrio_gnss_driver/abstraction/typedefs.hpp>
#include <thread>

/**
 * @file pcap_reader.cpp
 * @date 07/05/2021
 * @author Ashwin A Nayar
 *
 * @brief Implements auxiliary reader object for handling pcap files.
 *
 * Functions include connecting to the file, reading the contents to the
 * specified data buffer and graceful exit.
 */

namespace pcapReader {

    PcapDevice::PcapDevice(ROSaicNodeBase* node, buffer_t& buffer) :
        node_(node), m_dataBuff{buffer}
    {
    }

    PcapDevice::~PcapDevice() { disconnect(); }

    bool PcapDevice::connect(const char* device)
    {
        if (isConnected())
            return true;
        // Try to open pcap file
        if ((m_device = pcap_open_offline(device, m_errBuff)) == nullptr)
            return false;

        m_deviceName = (char*)device;
        // Try to compile filter program
        if (pcap_compile(m_device, &m_pktFilter, "tcp dst port 3001", 1,
                         PCAP_NETMASK_UNKNOWN) != 0)
            return false;

        node_->log(LogLevel::INFO, "Connected to" + std::string(m_deviceName));
        return true;
    }

    void PcapDevice::disconnect()
    {
        if (!isConnected())
            return;

        pcap_close(m_device);
        m_device = nullptr;
        node_->log(LogLevel::INFO, "Disconnected from " + std::string(m_deviceName));
    }

    bool PcapDevice::isConnected() const { return m_device; }

    ReadResult PcapDevice::read()
    {
        if (!isConnected())
            return READ_ERROR;

        struct pcap_pkthdr* header;
        const u_char* pktData;
        int result;

        result = pcap_next_ex(m_device, &header, &pktData);

        if (result >= 0)
        {
            auto ipHdr =
                reinterpret_cast<const iphdr*>(pktData + sizeof(struct ethhdr));
            uint32_t ipHdrLen = ipHdr->ihl * 4u;

            switch (ipHdr->protocol)
            {
            case 6:
            {
                if (header->len == 54)
                {
                    return READ_SUCCESS;
                }
                bool storePkt = true;

                if (!m_lastPkt.empty())
                {
                    auto tcpHdr = reinterpret_cast<const tcphdr*>(
                        pktData + ipHdrLen + sizeof(struct ethhdr));

                    auto lastIpHdr = reinterpret_cast<const iphdr*>(&(m_lastPkt[0]));
                    uint32_t lastIpHdrLen = lastIpHdr->ihl * 4u;
                    auto lastTcpHdr = reinterpret_cast<const tcphdr*>(
                        &(m_lastPkt[0]) + lastIpHdrLen);
                    uint16_t lastLen =
                        ntohs(static_cast<uint16_t>(lastIpHdr->tot_len));
                    uint16_t newLen = ntohs(static_cast<uint16_t>(ipHdr->tot_len));
                    uint32_t lastSeq = ntohl(lastTcpHdr->seq);
                    uint32_t newSeq = ntohl(tcpHdr->seq);

                    if (newSeq != lastSeq)
                    {
                        uint32_t dataOffset = lastTcpHdr->doff * 4;
                        m_dataBuff.insert(m_dataBuff.end(),
                                          m_lastPkt.begin() + lastIpHdrLen +
                                              dataOffset,
                                          m_lastPkt.end());
                    } else if (newLen <= lastLen)
                    {
                        storePkt = false;
                    }
                }

                if (storePkt)
                {
                    m_lastPkt.clear();
                    m_lastPkt.insert(m_lastPkt.end(),
                                     pktData + sizeof(struct ethhdr),
                                     pktData + header->len);
                }
                break;
            }

            default:
            {
                node_->log(LogLevel::ERROR,
                           "Skipping protocol: " + std::to_string(result));
                return READ_ERROR;
            }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1));

            return READ_SUCCESS;
        } else if (result == -2)
        {
            node_->log(LogLevel::INFO,
                       "Done reading from " + std::string(m_deviceName));
            if (!m_lastPkt.empty())
            {
                auto lastIpHdr = reinterpret_cast<const iphdr*>(&(m_lastPkt[0]));
                uint32_t ipHdrLength = lastIpHdr->ihl * 4u;

                auto lastTcpHdr =
                    reinterpret_cast<const tcphdr*>(&(m_lastPkt[0]) + ipHdrLength);
                uint32_t dataOffset = lastTcpHdr->doff * 4u;

                m_dataBuff.insert(m_dataBuff.end(),
                                  m_lastPkt.begin() + ipHdrLength + dataOffset,
                                  m_lastPkt.end());

                m_lastPkt.clear();
            }
            disconnect();
            return READ_SUCCESS;
        } else
        {
            node_->log(LogLevel::ERROR, "Error reading data");
            return READ_ERROR;
        }
    }
} // namespace pcapReader