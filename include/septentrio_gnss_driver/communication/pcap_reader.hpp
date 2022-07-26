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

#ifndef PCAP_READER_H
#define PCAP_READER_H

#include <cstdint>
#include <pcap/pcap.h>
#include <septentrio_gnss_driver/abstraction/typedefs.hpp>
#include <vector>

/**
 * @file pcap_reader.hpp
 * @date 07/05/2021
 * @author Ashwin A Nayar
 *
 * @brief Declares a class for handling pcap files.
 */

namespace pcapReader {

    using buffer_t = std::vector<uint8_t>;

    //! Read operation status
    enum ReadResult
    {
        /// Data read successfully
        READ_SUCCESS = 0,
        READ_INSUFFICIENT_DATA = 1,
        READ_TIMEOUT = 2,
        READ_INTERRUPTED = 3,
        READ_ERROR = -1,
        /// Unable to parse data, parsing error
        READ_PARSE_FAILED = -2

    };

    /**
     * @class PcapDevice
     * @brief Class for handling a pcap file
     */
    class PcapDevice
    {
    public:
        static const size_t BUFFSIZE = 100;

        /**
         * @brief Constructor for PcapDevice
         * @param[out] buffer Buffer to write read raw data to
         */
        explicit PcapDevice(ROSaicNodeBase* node, buffer_t& buffer);

        /**
         * @brief Try to open a pcap file
         * @param[in] device Path to pcap file
         * @return True if success, false otherwise
         */
        bool connect(const char* device);

        /**
         * @brief Close connected file
         */
        void disconnect();

        /**
         * @brief Check if file is open and healthy
         * @return True if file is open, false otherwise
         */
        bool isConnected() const;

        /**
         * @brief Attempt to read a packet and store data to buffer
         * @return Result of read operation
         */
        ReadResult read();

        //! Destructor for PcapDevice
        ~PcapDevice();

    private:
        //! Pointer to the node
        ROSaicNodeBase* node_;
        //! Reference to raw data buffer to write to
        buffer_t& m_dataBuff;
        //! File handle to pcap file
        pcap_t* m_device{nullptr};
        bpf_program m_pktFilter{};
        char m_errBuff[BUFFSIZE]{};
        char* m_deviceName;
        buffer_t m_lastPkt;
    };
} // namespace pcapReader

#endif // PCAP_READER_H
