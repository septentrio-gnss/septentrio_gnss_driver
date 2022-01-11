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

#ifndef SBFStructs_HPP
#define SBFStructs_HPP

//! Using maximum value of NR_OF_LOGICALCHANNELS for SBF definitions
#ifndef NR_OF_LOGICALCHANNELS
#define NR_OF_LOGICALCHANNELS 80
#endif
//! Inmarsat is a British satellite telecommunications company.
#ifndef MAX_NB_INMARSATCHANNELS
#define MAX_NB_INMARSATCHANNELS 1
#endif
//! Using maximum value of MAX_NR_OF_SIGNALS_PER_SATELLITE for SBF definitions
#ifndef MAX_NR_OF_SIGNALS_PER_SATELLITE
#define MAX_NR_OF_SIGNALS_PER_SATELLITE 7
#endif
//! Using maximum value of NR_OF_ANTENNAS for SBF definitions
#ifndef NR_OF_ANTENNAS
#define NR_OF_ANTENNAS 3
#endif
//! Maximum number of antennas that mosaic etc. can handle
#ifndef MAXSB_NBRANTENNA
#define MAXSB_NBRANTENNA 4
#endif
//! Max number of bytes that ChannelSatInfo sub-block can consist of
#ifndef MAXSB_CHANNELSATINFO
#define MAXSB_CHANNELSATINFO (NR_OF_LOGICALCHANNELS + MAX_NB_INMARSATCHANNELS)
#endif
//! Max number of bytes that ChannelStateInfo sub-block can consist of
#ifndef MAXSB_CHANNELSTATEINFO
#define MAXSB_CHANNELSTATEINFO (MAXSB_CHANNELSATINFO * MAXSB_NBRANTENNA)
#endif
//! Max number of bytes that MeasEpochChannelType1 sub-block can consist of
#ifndef MAXSB_MEASEPOCH_T1
#define MAXSB_MEASEPOCH_T1 (NR_OF_LOGICALCHANNELS + MAX_NB_INMARSATCHANNELS)
#endif
//! Max number of bytes that MeasEpochChannelType2 sub-block can consist of
#ifndef MAXSB_MEASEPOCH_T2
#define MAXSB_MEASEPOCH_T2 ((MAXSB_MEASEPOCH_T1) * (((MAX_NR_OF_SIGNALS_PER_SATELLITE) * (NR_OF_ANTENNAS)) - 1))
#endif
//! Max number of bytes that INSNavGeod sub-block can consist of
#ifndef SBF_INSNAVGEOD_LENGTH_1 
#define SBF_INSNAVGEOD_LENGTH_1 16 
#endif
//! Max number of bytes that INSNavCart sub-block can consist of
#ifndef SBF_INSNAVCART_LENGTH_1 
#define SBF_INSNAVCART_LENGTH_1 16 
#endif
//! Max number of bytes that INSNavGeod sub-block can consist of
#ifndef SBF_EXTEVENTINSNAVGEOD_LENGTH_1 
#define SBF_EXTEVENTINSNAVGEOD_LENGTH_1 16 
#endif
//! Max number of bytes that INSNavCart sub-block can consist of
#ifndef SBF_EXTEVENTINSNAVCART_LENGTH_1 
#define SBF_EXTEVENTINSNAVCART_LENGTH_1 16 
#endif
//! Max number of bytes that the Data part of the ChannelStatus struct can consist of
#ifndef SBF_CHANNELSTATUS_DATA_LENGTH
#define SBF_CHANNELSTATUS_DATA_LENGTH                                               \
    MAXSB_CHANNELSATINFO * sizeof(ChannelSatInfo) +                                 \
        MAXSB_CHANNELSTATEINFO * sizeof(ChannelStateInfo)
#endif
//! Max number of bytes that the data part of the MeasEpoch struct can consist of
#ifndef MEASEPOCH_DATA_LENGTH
#define MEASEPOCH_DATA_LENGTH                                                       \
    (MAXSB_MEASEPOCH_T1 * sizeof(MeasEpochChannelType1) +                           \
     MAXSB_MEASEPOCH_T2 * sizeof(MeasEpochChannelType2))
#endif
//! Max size for ExtSensorMeasSet_1 sub-block
#ifndef SBF_EXTSENSORMEAS_1_0_EXTSENSORMEAS_LENGTH 
#define SBF_EXTSENSORMEAS_1_0_EXTSENSORMEAS_LENGTH 4 
#endif

// Couple of redefinitions
#define SBF_INSNAVCART_LENGTH SBF_INSNAVCART_LENGTH_1
#define SBF_EXTSENSORMEAS_1_EXTSENSORMEAS_LENGTH SBF_EXTSENSORMEAS_1_0_EXTSENSORMEAS_LENGTH
#define SBF_EXTEVENTINSNAVCART_LENGTH SBF_EXTEVENTINSNAVCART_LENGTH_1
#define SBF_EXTEVENTINSNAVGEOD_LENGTH SBF_EXTEVENTINSNAVGEOD_LENGTH_1
#define SBF_INSNAVGEOD_LENGTH SBF_INSNAVGEOD_LENGTH_1

//! -2e10 shall be the do-not-use value. When an INS solution is not available, 
//! INS-related SBF sub-blocks are output with fields set to this DO_NOT_USE_VALUE.
#ifndef DO_NOT_USE_VALUE
#define DO_NOT_USE_VALUE -2e10f
#endif

//! 0x24 is ASCII for $ - 1st byte in each message
#ifndef SBF_SYNC_BYTE_1
#define SBF_SYNC_BYTE_1 0x24
#endif
//! 0x40 is ASCII for @ - 2nd byte to indicate SBF block
#ifndef SBF_SYNC_BYTE_2
#define SBF_SYNC_BYTE_2 0x40
#endif

// C++
#include <algorithm>
// Boost
//#define BOOST_SPIRIT_DEBUG 1
#define BOOST_SPIRIT_USE_PHOENIX_V3
#include <boost/spirit/include/qi.hpp>
#include <boost/spirit/include/phoenix.hpp>
#include <boost/fusion/include/adapt_struct.hpp>
#include <boost/spirit/repository/include/qi_advance.hpp>

#include <septentrio_gnss_driver/abstraction/typedefs.hpp>
#include <septentrio_gnss_driver/parsers/parsing_utilities.hpp>

/**
 * @file sbf_structs.hpp
 * @brief Declares and defines structs into which SBF blocks are unpacked then
 * shipped to handler functions
 * @date 17/08/20
 */

/**
 * @brief Struct for the SBF block's header message
 */
struct BlockHeader
{
    uint8_t  sync_1;   //!< first sync byte is $ or 0x24
    uint8_t  sync_2;   //!< 2nd sync byte is @ or 0x40
    uint16_t crc;      //!< The check sum
    uint16_t id;       //!< This is the block ID
    uint8_t  revision; //!< This is the block revision
    uint16_t length;   //!< Length of the entire message including the header. A
                       //!< multiple of 4 between 8 and 4096
    uint32_t tow;      //!< This is the time of week in ms
    uint16_t wnc;      //!< This is the GPS week counter
} ;

/**
 * @class ChannelStateInfo
 * @brief Struct for the SBF sub-block "ChannelStateInfo"
 */
struct ChannelStateInfo
{
    uint8_t antenna;
    uint16_t tracking_status;
    uint16_t pvt_status;
    uint16_t pvt_info;
};

/**
 * @class ChannelSatInfo
 * @brief Struct for the SBF sub-block "ChannelSatInfo"
 */
struct ChannelSatInfo
{
    uint8_t sv_id;
    uint8_t freq_nr;
    uint16_t az_rise_set;
    uint16_t health_status;
    int8_t elev;
    uint8_t n2;
    uint8_t rx_channel;
    std::vector<ChannelStateInfo> stateInfo;
};

/**
 * @class ChannelStatus
 * @brief Struct for the SBF block "ChannelStatus"
 */
struct ChannelStatus
{
    BlockHeader block_header;

    uint8_t n;
    uint8_t sb1_length;
    uint8_t sb2_length;
    std::vector<ChannelSatInfo> satInfo;
};

/**
 * @class DOP
 * @brief Struct for the SBF block "DOP"
 */
struct DOP
{
    BlockHeader block_header;

    uint8_t nr_sv;
    uint16_t pdop;
    uint16_t tdop;
    uint16_t hdop;
    uint16_t vdop;
    float hpl;
    float vpl;
};

/**
 * @class ReceiverSetup
 * @brief Struct for the SBF block "ReceiverSetup"
 */
struct ReceiverSetup
{
    BlockHeader block_header;

    std::string marker_name;
    std::string marker_number;
    std::string observer;
    std::string agency;
    std::string rx_serial_number;
    std::string rx_name;
    std::string rx_version;
    std::string ant_serial_nbr;
    std::string ant_type;
    float delta_h; /* [m] */
    float delta_e; /* [m] */
    float delta_n; /* [m] */
    std::string marker_type;
    std::string gnss_fw_version;
    std::string product_name;
    double latitude;
    double longitude;
    float  height;
    std::string station_code;
    uint8_t monument_idx;
    uint8_t receiver_idx;
    std::string country_code;
};

/**
 * @class QualityInd
 * @brief Struct for the SBF block "QualityInd"
 */
struct QualityInd
{
    BlockHeader block_header;

    uint8_t n = 0;
    std::vector<uint16_t> indicators;
};

/**
 * @brief Struct for the SBF sub-block "AGCState"
 */
struct AgcState
{
    uint8_t frontend_id;
    int8_t gain;
    uint8_t sample_var;
    uint8_t blanking_stat;
};

/**
 * @class ReceiverStatus
 * @brief Struct for the SBF block "ReceiverStatus"
 */
struct ReceiverStatus
{
    BlockHeader block_header;

    uint8_t cpu_load;
    uint8_t ext_error;
    uint32_t up_time;
    uint32_t rx_status;
    uint32_t rx_error;
    uint8_t n;
    uint8_t sb_length;
    uint8_t cmd_count;
    uint8_t temperature;
    std::vector<AgcState> agc_state;
};

/**
 * @brief CRC look-up table for fast computation of the 16-bit CRC for SBF blocks.
 *
 * Provided by Septenrio (c) 2020 Septentrio N.V./S.A., Belgium.
 */
static const uint16_t CRC_LOOK_UP[256] = {
    0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50a5, 0x60c6, 0x70e7, 0x8108, 0x9129,
    0xa14a, 0xb16b, 0xc18c, 0xd1ad, 0xe1ce, 0xf1ef, 0x1231, 0x0210, 0x3273, 0x2252,
    0x52b5, 0x4294, 0x72f7, 0x62d6, 0x9339, 0x8318, 0xb37b, 0xa35a, 0xd3bd, 0xc39c,
    0xf3ff, 0xe3de, 0x2462, 0x3443, 0x0420, 0x1401, 0x64e6, 0x74c7, 0x44a4, 0x5485,
    0xa56a, 0xb54b, 0x8528, 0x9509, 0xe5ee, 0xf5cf, 0xc5ac, 0xd58d, 0x3653, 0x2672,
    0x1611, 0x0630, 0x76d7, 0x66f6, 0x5695, 0x46b4, 0xb75b, 0xa77a, 0x9719, 0x8738,
    0xf7df, 0xe7fe, 0xd79d, 0xc7bc, 0x48c4, 0x58e5, 0x6886, 0x78a7, 0x0840, 0x1861,
    0x2802, 0x3823, 0xc9cc, 0xd9ed, 0xe98e, 0xf9af, 0x8948, 0x9969, 0xa90a, 0xb92b,
    0x5af5, 0x4ad4, 0x7ab7, 0x6a96, 0x1a71, 0x0a50, 0x3a33, 0x2a12, 0xdbfd, 0xcbdc,
    0xfbbf, 0xeb9e, 0x9b79, 0x8b58, 0xbb3b, 0xab1a, 0x6ca6, 0x7c87, 0x4ce4, 0x5cc5,
    0x2c22, 0x3c03, 0x0c60, 0x1c41, 0xedae, 0xfd8f, 0xcdec, 0xddcd, 0xad2a, 0xbd0b,
    0x8d68, 0x9d49, 0x7e97, 0x6eb6, 0x5ed5, 0x4ef4, 0x3e13, 0x2e32, 0x1e51, 0x0e70,
    0xff9f, 0xefbe, 0xdfdd, 0xcffc, 0xbf1b, 0xaf3a, 0x9f59, 0x8f78, 0x9188, 0x81a9,
    0xb1ca, 0xa1eb, 0xd10c, 0xc12d, 0xf14e, 0xe16f, 0x1080, 0x00a1, 0x30c2, 0x20e3,
    0x5004, 0x4025, 0x7046, 0x6067, 0x83b9, 0x9398, 0xa3fb, 0xb3da, 0xc33d, 0xd31c,
    0xe37f, 0xf35e, 0x02b1, 0x1290, 0x22f3, 0x32d2, 0x4235, 0x5214, 0x6277, 0x7256,
    0xb5ea, 0xa5cb, 0x95a8, 0x8589, 0xf56e, 0xe54f, 0xd52c, 0xc50d, 0x34e2, 0x24c3,
    0x14a0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405, 0xa7db, 0xb7fa, 0x8799, 0x97b8,
    0xe75f, 0xf77e, 0xc71d, 0xd73c, 0x26d3, 0x36f2, 0x0691, 0x16b0, 0x6657, 0x7676,
    0x4615, 0x5634, 0xd94c, 0xc96d, 0xf90e, 0xe92f, 0x99c8, 0x89e9, 0xb98a, 0xa9ab,
    0x5844, 0x4865, 0x7806, 0x6827, 0x18c0, 0x08e1, 0x3882, 0x28a3, 0xcb7d, 0xdb5c,
    0xeb3f, 0xfb1e, 0x8bf9, 0x9bd8, 0xabbb, 0xbb9a, 0x4a75, 0x5a54, 0x6a37, 0x7a16,
    0x0af1, 0x1ad0, 0x2ab3, 0x3a92, 0xfd2e, 0xed0f, 0xdd6c, 0xcd4d, 0xbdaa, 0xad8b,
    0x9de8, 0x8dc9, 0x7c26, 0x6c07, 0x5c64, 0x4c45, 0x3ca2, 0x2c83, 0x1ce0, 0x0cc1,
    0xef1f, 0xff3e, 0xcf5d, 0xdf7c, 0xaf9b, 0xbfba, 0x8fd9, 0x9ff8, 0x6e17, 0x7e36,
    0x4e55, 0x5e74, 0x2e93, 0x3eb2, 0x0ed1, 0x1ef0};

BOOST_FUSION_ADAPT_STRUCT(
BlockHeader,
    (uint8_t, sync_1),
    (uint8_t, sync_2),
    (uint16_t, crc),
    (uint16_t, id),
    (uint8_t, revision),
    (uint16_t, length),
    (uint32_t, tow),
    (uint16_t, wnc)
)

BOOST_FUSION_ADAPT_STRUCT(
DOP,
    (BlockHeader, block_header),
    (uint8_t, nr_sv),
    (uint16_t, pdop),
    (uint16_t, tdop),
    (uint16_t, hdop),
    (uint16_t, vdop),
    (float, hpl),
    (float, vpl)
)

BOOST_FUSION_ADAPT_STRUCT(
QualityInd,
    (BlockHeader, block_header),
    (uint8_t, n),
    (std::vector<uint16_t>, indicators)
)

BOOST_FUSION_ADAPT_STRUCT(
AgcState,
    (uint8_t, frontend_id),
    (int8_t, gain),
    (uint8_t, sample_var),
    (uint8_t, blanking_stat)
)

BOOST_FUSION_ADAPT_STRUCT(
ReceiverStatus,
    (BlockHeader, block_header),
    (uint8_t, cpu_load),
    (uint8_t, ext_error),
    (uint32_t, up_time),
    (uint32_t, rx_status),
    (uint32_t, rx_error),
    (uint8_t, n),
    (uint8_t, sb_length),
    (uint8_t, cmd_count),
    (uint8_t, temperature),
    (std::vector<AgcState>, agc_state)
)

namespace qi  = boost::spirit::qi;
namespace rep = boost::spirit::repository;
namespace phx = boost::phoenix;

/**
 * @struct BlockHeaderGrammar
 * @brief Spirit grammar for the SBF block "BlockHeader"
 */
template<typename Iterator>
struct BlockHeaderGrammar : qi::grammar<Iterator, BlockHeader(uint16_t, uint8_t&)>
{
	BlockHeaderGrammar() : BlockHeaderGrammar::base_type(blockHeader)
	{
		using namespace qi::labels;
		
        blockHeader %= qi::byte_[_pass = qi::_1 == SBF_SYNC_BYTE_1]
		            >> qi::byte_[_pass = qi::_1 == SBF_SYNC_BYTE_2]
		            >> qi::little_word
		            >> qi::omit[qi::little_word[phx::ref(id) = (qi::_1 & 8191), _pass = (phx::ref(id) == _r1), _r2 = qi::_1 >> 13]] // revision is upper 3 bits
                    >> qi::attr(phx::ref(id))
                    >> qi::attr(_r2)
                    >> qi::little_word
                    >> qi::little_dword
                    >> qi::little_word;

        
        BOOST_SPIRIT_DEBUG_NODE(blockHeader);
	}

    uint16_t id;

	qi::rule<Iterator, BlockHeader(uint16_t, uint8_t&)> blockHeader;
};

/**
 * @struct DopGrammar
 * @brief Spirit grammar for the SBF block "DOP"
 */
template<typename Iterator>
struct DopGrammar : qi::grammar<Iterator, DOP()>
{
	DopGrammar() : DopGrammar::base_type(dop)
	{
        using namespace qi::labels;       

		dop %= header(4001, phx::ref(revision))
		    >> qi::byte_
            >> rep::qi::advance(1) // reserved
            >> qi::little_word
		    >> qi::little_word
            >> qi::little_word
		    >> qi::little_word
            >> qi::little_bin_float
            >> qi::little_bin_float
            >> qi::repeat[rep::qi::advance(1)]; // skip padding
	}

    uint8_t  revision;

    BlockHeaderGrammar<Iterator> header;
	qi::rule<Iterator, DOP()>    dop;
};

/**
 * @struct QualityIndGrammar
 * @brief Spirit grammar for the SBF block "QualityInd"
 */
template<typename Iterator>
struct QualityIndGrammar : qi::grammar<Iterator, QualityInd()>
{
	QualityIndGrammar() : QualityIndGrammar::base_type(qualityInd)
	{
        using namespace qi::labels;        

		qualityInd %= header(4082, phx::ref(revision))
		           >> qi::byte_[_pass = (qi::_1 <= 40), phx::ref(n) = qi::_1]
                   >> rep::qi::advance(1) // reserved
                   >> qi::eps[phx::reserve(phx::at_c<2>(_val), phx::ref(n))]
                   >> qi::repeat(phx::ref(n))[qi::little_word]
		           >> qi::repeat[rep::qi::advance(1)]; // skip padding
	}

    uint8_t  revision;
    uint8_t  n;

    BlockHeaderGrammar<Iterator>     header;
    qi::rule<Iterator, QualityInd()> qualityInd;
};

/**
 * @struct ReceiverStatusGrammar
 * @brief Spirit grammar for the SBF block "ReceiverStatus"
 */
template<typename Iterator>
struct ReceiverStatusGrammar : qi::grammar<Iterator, ReceiverStatus()>
{
	ReceiverStatusGrammar() : ReceiverStatusGrammar::base_type(receiverStatus)
	{
        using namespace qi::labels;        

        agcState %= qi::byte_
                 >> qi::char_
                 >> qi::byte_
                 >> qi::byte_
                 >> rep::qi::advance(phx::ref(sb_length) - 4); // skip padding: sb_length - 4 bytes

        receiverStatus %= header(4014, phx::ref(revision))
		               >> qi::byte_
                       >> qi::byte_
                       >> qi::little_dword
                       >> qi::little_dword
                       >> qi::little_dword
                       >> qi::byte_[_pass = (qi::_1 <= 18), phx::ref(n) = qi::_1] // n
                       >> qi::byte_[phx::ref(sb_length) = qi::_1] // sb_length
                       >> qi::byte_
                       >> qi::byte_
                       >> qi::eps[phx::reserve(phx::at_c<10>(_val), phx::ref(n))]
                       >> qi::repeat(phx::ref(n))[agcState]
                       >> qi::repeat[rep::qi::advance(1)]; // skip padding

        BOOST_SPIRIT_DEBUG_NODE(receiverStatus);
	}

    uint8_t  revision;
    uint8_t  n;
    uint8_t  sb_length;

    BlockHeaderGrammar<Iterator>         header;
    qi::rule<Iterator, AgcState()>       agcState;
	qi::rule<Iterator, ReceiverStatus()> receiverStatus;
};

/**
 * qiLittleEndianParser
 * @brief Qi little endian parsers for numeric values
 */
template<typename It, typename Val>
bool qiLittleEndianParser(It& it, Val& val)
{
    if (std::is_same<int8_t, Val>::value)
	{
        return qi::parse(it, it + 1, qi::char_, val);
    }
    else if (std::is_same<uint8_t, Val>::value)
	{
        return qi::parse(it, it + 1, qi::byte_, val);
    }
    else if ((std::is_same<int16_t, Val>::value) || (std::is_same<uint16_t, Val>::value))
	{
        return qi::parse(it, it + 2, qi::little_word, val);
    }
    else if ((std::is_same<int32_t, Val>::value) || (std::is_same<uint32_t, Val>::value))
	{
        return qi::parse(it, it + 4, qi::little_dword, val);
    }
    else if ((std::is_same<int64_t, Val>::value) || (std::is_same<uint64_t, Val>::value))
	{
        return qi::parse(it, it + 8, qi::little_qword, val);
    }
    else if (std::is_same<float, Val>::value)
	{
        return qi::parse(it, it + 4, qi::little_bin_float, val);
    }
    else if (std::is_same<double, Val>::value)
	{
        return qi::parse(it, it + 8, qi::little_bin_double, val);
    }
}

/**
 * qiCharsToStringParser
 * @brief Qi parser for char array to string
 */
template<typename It>
bool qiCharsToStringParser(It& it, std::string& val, std::size_t num)
{
    bool success = false;
    val.clear();
    success = qi::parse(it, it + num, qi::repeat(num)[qi::char_], val);
    val.erase(std::remove(val.begin(), val.end(), '\0'), val.end());
    return success;
}

/**
 * BlockHeaderParser
 * @brief Qi based parser for the SBF block "BlockHeader" plus receiver time stamp 
 */
template<typename It, typename Hdr>
bool BlockHeaderParser(ROSaicNodeBase* node, It& it, Hdr& block_header)
{  
    qiLittleEndianParser(it, block_header.sync_1);
    if (block_header.sync_1 != SBF_SYNC_BYTE_1)
    {
        node->log(LogLevel::ERROR, "Parse error: Wrong sync byte 1.");
        return false;
    }
    qiLittleEndianParser(it, block_header.sync_2);
    if (block_header.sync_2 != SBF_SYNC_BYTE_2)
    {
        node->log(LogLevel::ERROR, "Parse error: Wrong sync byte 2.");
        return false;
    }
    qiLittleEndianParser(it, block_header.crc);
    uint16_t ID;
    qiLittleEndianParser(it, ID);
    block_header.id       = ID & 8191;   
    block_header.revision = ID >> 13;
    qiLittleEndianParser(it, block_header.length);
    qiLittleEndianParser(it, block_header.tow);
    qiLittleEndianParser(it, block_header.wnc);
    return true;
}

/**
 * ChannelStateInfoParser
 * @brief Struct for the SBF sub-block "ChannelStateInfo"
 */
template<typename It>
void ChannelStateInfoParser(It& it, ChannelStateInfo& msg, uint8_t sb2_length)
{
    qiLittleEndianParser(it, msg.antenna);
    ++it; // reserved
    qiLittleEndianParser(it, msg.tracking_status);
    qiLittleEndianParser(it, msg.pvt_status);
    qiLittleEndianParser(it, msg.pvt_info);
    std::advance(it, sb2_length - 8); // skip padding
};

/**
 * ChannelSatInfoParser
 * @brief Struct for the SBF sub-block "ChannelSatInfo"
 */
template<typename It>
bool ChannelSatInfoParser(ROSaicNodeBase* node, It& it, ChannelSatInfo& msg, uint8_t sb1_length, uint8_t sb2_length)
{
    qiLittleEndianParser(it, msg.sv_id);
    qiLittleEndianParser(it, msg.freq_nr);
    std::advance(it, 2); // reserved
    qiLittleEndianParser(it, msg.az_rise_set);
    qiLittleEndianParser(it, msg.health_status);
    qiLittleEndianParser(it, msg.elev);
    qiLittleEndianParser(it, msg.n2);
    if (msg.n2 > MAXSB_MEASEPOCH_T2)
    {
        node->log(LogLevel::ERROR, "Parse error: Too many ChannelStateInfo " + std::to_string(msg.n2));
        return false;
    }
    qiLittleEndianParser(it, msg.rx_channel);
    ++it; // reserved
    std::advance(it, sb1_length - 12); // skip padding
    msg.stateInfo.resize(msg.n2);
    for (auto& stateInfo : msg.stateInfo)
    {
        ChannelStateInfoParser(it, stateInfo, sb2_length);
    } 
    return true;
};

/**
 * ChannelStatusParser
 * @brief Struct for the SBF block "ChannelStatus"
 */
template<typename It>
bool ChannelStatusParser(ROSaicNodeBase* node, It it, It itEnd, ChannelStatus& msg)
{
    if(!BlockHeaderParser(node, it, msg.block_header))
        return false;
    if (msg.block_header.id != 4013)
    {
        node->log(LogLevel::ERROR, "Parse error: Wrong header ID " + std::to_string(msg.block_header.id));
        return false;
    }
    qiLittleEndianParser(it, msg.n);
    if (msg.n > MAXSB_CHANNELSATINFO)
    {
        node->log(LogLevel::ERROR, "Parse error: Too many ChannelSatInfo " + std::to_string(msg.n));
        return false;
    }
    qiLittleEndianParser(it, msg.sb1_length);
    qiLittleEndianParser(it, msg.sb2_length);
    std::advance(it, 3); // reserved
    msg.satInfo.resize(msg.n);
    for (auto& satInfo : msg.satInfo)
    {
        if (!ChannelSatInfoParser(node, it, satInfo, msg.sb1_length, msg.sb2_length))
            return false;
    } 
    return true;
};

/**
 * MeasEpochChannelType2Parser
 * @brief Qi based parser for the SBF sub-block "MeasEpochChannelType2"
 */
template<typename It>
void MeasEpochChannelType2Parser(It& it, MeasEpochChannelType2Msg& msg, uint8_t sb2_length)
{
    qiLittleEndianParser(it, msg.type);
    qiLittleEndianParser(it, msg.lock_time);
    qiLittleEndianParser(it, msg.cn0);
    qiLittleEndianParser(it, msg.offsets_msb);
    qiLittleEndianParser(it, msg.carrier_msb);
    qiLittleEndianParser(it, msg.obs_info);
    qiLittleEndianParser(it, msg.code_offset_lsb);
    qiLittleEndianParser(it, msg.carrier_lsb);
    qiLittleEndianParser(it, msg.doppler_offset_lsb);
    std::advance(it, sb2_length - 12); // skip padding
};

/**
 * @class MeasEpochChannelType1Parser
 * @brief Qi based parser for the SBF sub-block "MeasEpochChannelType1"
 */
template<typename It>
bool MeasEpochChannelType1Parser(ROSaicNodeBase* node, It& it, MeasEpochChannelType1Msg& msg, uint8_t sb1_length, uint8_t sb2_length)
{
    qiLittleEndianParser(it, msg.rx_channel);
    qiLittleEndianParser(it, msg.type);
    qiLittleEndianParser(it, msg.sv_id);
    qiLittleEndianParser(it, msg.misc);
    qiLittleEndianParser(it, msg.code_lsb);
    qiLittleEndianParser(it, msg.doppler);
    qiLittleEndianParser(it, msg.carrier_lsb);
    qiLittleEndianParser(it, msg.carrier_msb);
    qiLittleEndianParser(it, msg.cn0);
    qiLittleEndianParser(it, msg.lock_time);
    qiLittleEndianParser(it, msg.obs_info);
    qiLittleEndianParser(it, msg.n2);
    std::advance(it, sb1_length - 20); // skip padding
    if (msg.n2 > MAXSB_MEASEPOCH_T2)
    {
        node->log(LogLevel::ERROR, "Parse error: Too many MeasEpochChannelType2 " + std::to_string(msg.n2));
        return false;
    }
    msg.type2.resize(msg.n2);
    for (auto& type2 : msg.type2)
    {
        MeasEpochChannelType2Parser(it, type2, sb2_length);
    } 
    return true;
};

/**
 * @class MeasEpoch
 * @brief Qi based parser for the SBF block "MeasEpoch"
 */
template<typename It>
bool MeasEpochParser(ROSaicNodeBase* node, It it, It itEnd, MeasEpochMsg& msg)
{
    if(!BlockHeaderParser(node, it, msg.block_header))
        return false;
    if (msg.block_header.id != 4027)
    {
        node->log(LogLevel::ERROR, "Parse error: Wrong header ID " + std::to_string(msg.block_header.id));
        return false;
    }
    qiLittleEndianParser(it, msg.n);
    if (msg.n > MAXSB_MEASEPOCH_T1)
    {
        node->log(LogLevel::ERROR, "Parse error: Too many MeasEpochChannelType1 " + std::to_string(msg.n));
        return false;
    }
    qiLittleEndianParser(it, msg.sb1_length);
    qiLittleEndianParser(it, msg.sb2_length);
    qiLittleEndianParser(it, msg.common_flags);
    if (msg.block_header.revision > 0)
        qiLittleEndianParser(it, msg.cum_clk_jumps);
    ++it; // reserved
    msg.type1.resize(msg.n);
    for (auto& type1 : msg.type1)
    {
        if (!MeasEpochChannelType1Parser(node, it, type1, msg.sb1_length, msg.sb2_length))
            return false;
    }
    if (it > itEnd)
    {
        node->log(LogLevel::ERROR, "Parse error: iterator past end.");
        return false;
    }
    return true;
};

/**
 * ReceiverSetupParser
 * @brief Qi based parser for the SBF block "ReceiverSetup"
 */
template<typename It>
bool ReceiverSetupParser(ROSaicNodeBase* node, It it, It itEnd, ReceiverSetup& msg)
{
    if(!BlockHeaderParser(node, it, msg.block_header))
        return false;
    if (msg.block_header.id != 5902)
    {
        node->log(LogLevel::ERROR, "Parse error: Wrong header ID " + std::to_string(msg.block_header.id));
        return false;
    }
    std::advance(it, 2); // reserved
    qiCharsToStringParser(it, msg.marker_name, 60);
    qiCharsToStringParser(it, msg.marker_number, 20);
    qiCharsToStringParser(it, msg.observer, 20);
    qiCharsToStringParser(it, msg.agency, 40);
    qiCharsToStringParser(it, msg.rx_serial_number, 20);
    qiCharsToStringParser(it, msg.rx_name, 20);
    qiCharsToStringParser(it, msg.rx_version, 20);
    qiCharsToStringParser(it, msg.ant_serial_nbr, 20);
    qiCharsToStringParser(it, msg.ant_type, 20);
    qiLittleEndianParser(it, msg.delta_h);
    qiLittleEndianParser(it, msg.delta_e);
    qiLittleEndianParser(it, msg.delta_n);
    if (msg.block_header.revision > 0)
        qiCharsToStringParser(it, msg.marker_type, 20);
    if (msg.block_header.revision > 1)
        qiCharsToStringParser(it, msg.gnss_fw_version, 40);
    if (msg.block_header.revision > 2)
        qiCharsToStringParser(it, msg.product_name, 40);
    if (msg.block_header.revision > 3)
    {    
        qiLittleEndianParser(it, msg.latitude);
        qiLittleEndianParser(it, msg.longitude);
        qiLittleEndianParser(it, msg.height);
        qiCharsToStringParser(it, msg.station_code, 10);
        qiLittleEndianParser(it, msg.monument_idx);
        qiLittleEndianParser(it, msg.receiver_idx);
        qiCharsToStringParser(it, msg.country_code, 3);
    }
    else
    {
        msg.latitude  = DO_NOT_USE_VALUE;
        msg.longitude = DO_NOT_USE_VALUE;
        msg.height    = DO_NOT_USE_VALUE;
    }
    if (it > itEnd)
    {
        node->log(LogLevel::ERROR, "Parse error: iterator past end.");
        return false;
    }
    return true;
};

/**
 * PVTCartesianParser
 * @brief Qi based parser for the SBF block "PVTCartesian"
 */
template<typename It>
bool PVTCartesianParser(ROSaicNodeBase* node, It it, It itEnd, PVTCartesianMsg& msg)
{
    if(!BlockHeaderParser(node, it, msg.block_header))
        return false;
    if (msg.block_header.id != 4006)
    {
        node->log(LogLevel::ERROR, "Parse error: Wrong header ID " + std::to_string(msg.block_header.id));
        return false;
    }
    qiLittleEndianParser(it, msg.mode);
    qiLittleEndianParser(it, msg.error);
    qiLittleEndianParser(it, msg.x);
    qiLittleEndianParser(it, msg.y);
    qiLittleEndianParser(it, msg.z);
    qiLittleEndianParser(it, msg.undulation);
    qiLittleEndianParser(it, msg.vx);
    qiLittleEndianParser(it, msg.vy);
    qiLittleEndianParser(it, msg.vz);
    qiLittleEndianParser(it, msg.cog);
    qiLittleEndianParser(it, msg.rx_clk_bias);
    qiLittleEndianParser(it, msg.rx_clk_drift);
    qiLittleEndianParser(it, msg.time_system);
    qiLittleEndianParser(it, msg.datum);
    qiLittleEndianParser(it, msg.nr_sv);
    qiLittleEndianParser(it, msg.wa_corr_info);
    qiLittleEndianParser(it, msg.reference_id);
    qiLittleEndianParser(it, msg.mean_corr_age);
    qiLittleEndianParser(it, msg.signal_info);
    qiLittleEndianParser(it, msg.alert_flag);
    if (msg.block_header.revision > 0)
    {
        qiLittleEndianParser(it, msg.nr_bases);
        qiLittleEndianParser(it, msg.ppp_info);
    }
    if (msg.block_header.revision > 1)
    {
        qiLittleEndianParser(it, msg.latency);
        qiLittleEndianParser(it, msg.h_accuracy);
        qiLittleEndianParser(it, msg.v_accuracy);
        qiLittleEndianParser(it, msg.misc);
    }
    if (it > itEnd)
    {
        node->log(LogLevel::ERROR, "Parse error: iterator past end.");
        return false;
    }
    return true;
}

/**
 * PVTGeodeticParser
 * @brief Qi based parser for the SBF block "PVTGeodetic"
 */
template<typename It>
bool PVTGeodeticParser(ROSaicNodeBase* node, It it, It itEnd, PVTGeodeticMsg& msg)
{
    if(!BlockHeaderParser(node, it, msg.block_header))
        return false;
    if (msg.block_header.id != 4007)
    {
        node->log(LogLevel::ERROR, "Parse error: Wrong header ID " + std::to_string(msg.block_header.id));
        return false;
    }
    qiLittleEndianParser(it, msg.mode);
    qiLittleEndianParser(it, msg.error);
    qiLittleEndianParser(it, msg.latitude);
    qiLittleEndianParser(it, msg.longitude);
    qiLittleEndianParser(it, msg.height);
    qiLittleEndianParser(it, msg.undulation);
    qiLittleEndianParser(it, msg.vn);
    qiLittleEndianParser(it, msg.ve);
    qiLittleEndianParser(it, msg.vu);
    qiLittleEndianParser(it, msg.cog);
    qiLittleEndianParser(it, msg.rx_clk_bias);
    qiLittleEndianParser(it, msg.rx_clk_drift);
    qiLittleEndianParser(it, msg.time_system);
    qiLittleEndianParser(it, msg.datum);
    qiLittleEndianParser(it, msg.nr_sv);
    qiLittleEndianParser(it, msg.wa_corr_info);
    qiLittleEndianParser(it, msg.reference_id);
    qiLittleEndianParser(it, msg.mean_corr_age);
    qiLittleEndianParser(it, msg.signal_info);
    qiLittleEndianParser(it, msg.alert_flag);
    if (msg.block_header.revision > 0)
    {
        qiLittleEndianParser(it, msg.nr_bases);
        qiLittleEndianParser(it, msg.ppp_info);
    }
    if (msg.block_header.revision > 1)
    {
        qiLittleEndianParser(it, msg.latency);
        qiLittleEndianParser(it, msg.h_accuracy);
        qiLittleEndianParser(it, msg.v_accuracy);
        qiLittleEndianParser(it, msg.misc);
    }
    if (it > itEnd)
    {
        node->log(LogLevel::ERROR, "Parse error: iterator past end.");
        return false;
    }
    return true;
}

/**
 * AttEulerParser
 * @brief Qi based parser for the SBF block "AttEuler"
 */
template<typename It>
bool AttEulerParser(ROSaicNodeBase* node, It it, It itEnd, AttEulerMsg& msg, bool use_ros_axis_orientation)
{    
    if(!BlockHeaderParser(node, it, msg.block_header))
        return false;
    if (msg.block_header.id != 5938)
    {
        node->log(LogLevel::ERROR, "Parse error: Wrong header ID " + std::to_string(msg.block_header.id));
        return false;
    }
    qiLittleEndianParser(it, msg.nr_sv);
    qiLittleEndianParser(it, msg.error);
    qiLittleEndianParser(it, msg.mode);
    std::advance(it, 2); // reserved
    qiLittleEndianParser(it, msg.heading);
    qiLittleEndianParser(it, msg.pitch);
    qiLittleEndianParser(it, msg.roll);
    qiLittleEndianParser(it, msg.pitch_dot);
    qiLittleEndianParser(it, msg.roll_dot);
    qiLittleEndianParser(it, msg.heading_dot);
    if (use_ros_axis_orientation)
    {
        if (msg.heading != DO_NOT_USE_VALUE)
            msg.heading = -msg.heading + parsing_utilities::pi_half;
        if (msg.pitch != DO_NOT_USE_VALUE)
            msg.pitch = -msg.pitch;
        if (msg.pitch_dot != DO_NOT_USE_VALUE)
            msg.pitch_dot = -msg.pitch_dot;
        if (msg.heading_dot != DO_NOT_USE_VALUE)
            msg.heading_dot = -msg.heading_dot;
    }
    if (it > itEnd)
    {
        node->log(LogLevel::ERROR, "Parse error: iterator past end.");
        return false;
    }
    return true;
};

/**
 * AttCovEulerParser
 * @brief Qi based parser for the SBF block "AttCovEuler"
 */
template<typename It>
bool AttCovEulerParser(ROSaicNodeBase* node, It it, It itEnd, AttCovEulerMsg& msg, bool use_ros_axis_orientation)
{    
    if(!BlockHeaderParser(node, it, msg.block_header))
        return false;
    if (msg.block_header.id != 5939)
    {
        node->log(LogLevel::ERROR, "Parse error: Wrong header ID " + std::to_string(msg.block_header.id));
        return false;
    }
    ++it; // reserved
    qiLittleEndianParser(it, msg.error);
    qiLittleEndianParser(it, msg.cov_headhead);
    qiLittleEndianParser(it, msg.cov_pitchpitch);
    qiLittleEndianParser(it, msg.cov_rollroll);
    qiLittleEndianParser(it, msg.cov_headpitch); 
    qiLittleEndianParser(it, msg.cov_headroll);
    qiLittleEndianParser(it, msg.cov_pitchroll);
    if (use_ros_axis_orientation)
    {        
        if (msg.cov_headroll != DO_NOT_USE_VALUE)
            msg.cov_headroll  = -msg.cov_headroll;
        if (msg.cov_pitchroll != DO_NOT_USE_VALUE)
            msg.cov_pitchroll = -msg.cov_pitchroll;
    }
    if (it > itEnd)
    {
        node->log(LogLevel::ERROR, "Parse error: iterator past end.");
        return false;
    }
    return true;
};

/**
 * INSNavCartParser
 * @brief Qi based parser for the SBF block "INSNavCart"
 */
template<typename It>
bool INSNavCartParser(ROSaicNodeBase* node, It it, It itEnd, INSNavCartMsg& msg, bool use_ros_axis_orientation)
{    
    if(!BlockHeaderParser(node, it, msg.block_header))
        return false;
    if ((msg.block_header.id != 4225) && (msg.block_header.id != 4229))
    {
        node->log(LogLevel::ERROR, "Parse error: Wrong header ID " + std::to_string(msg.block_header.id));
        return false;
    }
    qiLittleEndianParser(it, msg.gnss_mode);
    qiLittleEndianParser(it, msg.error);
    qiLittleEndianParser(it, msg.info);
    qiLittleEndianParser(it, msg.gnss_age);
    qiLittleEndianParser(it, msg.x);
    qiLittleEndianParser(it, msg.y);
    qiLittleEndianParser(it, msg.z);
    qiLittleEndianParser(it, msg.accuracy);
    qiLittleEndianParser(it, msg.latency);
    qiLittleEndianParser(it, msg.datum);
    ++it; //reserved
    qiLittleEndianParser(it, msg.sb_list);
    if((msg.sb_list & 1) !=0)
    {
        qiLittleEndianParser(it, msg.x_std_dev);
        qiLittleEndianParser(it, msg.y_std_dev);
        qiLittleEndianParser(it, msg.z_std_dev);
    }
    else
    {
        msg.x_std_dev = DO_NOT_USE_VALUE;
        msg.y_std_dev = DO_NOT_USE_VALUE;
        msg.z_std_dev = DO_NOT_USE_VALUE;
    }
    if((msg.sb_list & 2) !=0)
    {
        qiLittleEndianParser(it, msg.heading);
        qiLittleEndianParser(it, msg.pitch);
        qiLittleEndianParser(it, msg.roll);
        if (use_ros_axis_orientation)
        {
            msg.heading = -msg.heading + parsing_utilities::pi_half;
            msg.pitch   = -msg.pitch;
        }
    }
    else
    {
        msg.heading = DO_NOT_USE_VALUE;
        msg.pitch   = DO_NOT_USE_VALUE;
        msg.roll    = DO_NOT_USE_VALUE;
    }
    if((msg.sb_list & 4) !=0)
    {
        qiLittleEndianParser(it, msg.heading_std_dev);
        qiLittleEndianParser(it, msg.pitch_std_dev);
        qiLittleEndianParser(it, msg.roll_std_dev);
    }
    else
    {
        msg.heading_std_dev = DO_NOT_USE_VALUE;
        msg.pitch_std_dev   = DO_NOT_USE_VALUE;
        msg.roll_std_dev    = DO_NOT_USE_VALUE;
    }
    if((msg.sb_list & 8) !=0)
    {
        qiLittleEndianParser(it, msg.vx);
        qiLittleEndianParser(it, msg.vy);
        qiLittleEndianParser(it, msg.vz);
    }
    else
    {
        msg.vx = DO_NOT_USE_VALUE;
        msg.vy = DO_NOT_USE_VALUE;
        msg.vz = DO_NOT_USE_VALUE;
    }
    if((msg.sb_list & 16) !=0)
    {
        qiLittleEndianParser(it, msg.vx_std_dev);
        qiLittleEndianParser(it, msg.vy_std_dev);
        qiLittleEndianParser(it, msg.vz_std_dev);
    }
    else
    {
        msg.vx_std_dev = DO_NOT_USE_VALUE;
        msg.vy_std_dev = DO_NOT_USE_VALUE;
        msg.vz_std_dev = DO_NOT_USE_VALUE;
    }
    if((msg.sb_list & 32) !=0)
    {
        qiLittleEndianParser(it, msg.xy_cov);
        qiLittleEndianParser(it, msg.xz_cov);
        qiLittleEndianParser(it, msg.yz_cov);
    }
    else
    {
        msg.xy_cov = DO_NOT_USE_VALUE;
        msg.xz_cov = DO_NOT_USE_VALUE;
        msg.yz_cov = DO_NOT_USE_VALUE;
    }
    if((msg.sb_list & 64) !=0)
    {
        qiLittleEndianParser(it, msg.heading_pitch_cov);
        qiLittleEndianParser(it, msg.heading_roll_cov);
        qiLittleEndianParser(it, msg.pitch_roll_cov);
        if (use_ros_axis_orientation)
        {   
            msg.heading_roll_cov = -msg.heading_roll_cov;
            msg.pitch_roll_cov   = -msg.pitch_roll_cov;
        }
    }
    else
    {
        msg.heading_pitch_cov = DO_NOT_USE_VALUE;
        msg.heading_roll_cov  = DO_NOT_USE_VALUE;
        msg.pitch_roll_cov    = DO_NOT_USE_VALUE;
    }
    if((msg.sb_list & 128) !=0)
    {
        qiLittleEndianParser(it, msg.vx_vy_cov);
        qiLittleEndianParser(it, msg.vx_vz_cov);
        qiLittleEndianParser(it, msg.vy_vz_cov);
    }
    else
    {
        msg.vx_vy_cov = DO_NOT_USE_VALUE;
        msg.vx_vz_cov = DO_NOT_USE_VALUE;
        msg.vy_vz_cov = DO_NOT_USE_VALUE;
    }
    if (it > itEnd)
    {
        node->log(LogLevel::ERROR, "Parse error: iterator past end.");
        return false;
    }
    return true;
};

/**
 * PosCovCartesianParser
 * @brief Qi based parser for the SBF block "PosCovCartesian"
 */
template<typename It>
bool PosCovCartesianParser(ROSaicNodeBase* node, It it, It itEnd, PosCovCartesianMsg& msg)
{    
    if(!BlockHeaderParser(node, it, msg.block_header))
        return false;
    if (msg.block_header.id != 5905)
    {
        node->log(LogLevel::ERROR, "Parse error: Wrong header ID " + std::to_string(msg.block_header.id));
        return false;
    }
    qiLittleEndianParser(it, msg.mode);
    qiLittleEndianParser(it, msg.error);
    qiLittleEndianParser(it, msg.cov_xx);
    qiLittleEndianParser(it,msg.cov_yy);
    qiLittleEndianParser(it,msg.cov_zz);
    qiLittleEndianParser(it,msg.cov_bb);
    qiLittleEndianParser(it,msg.cov_xy);
    qiLittleEndianParser(it,msg.cov_xz);
    qiLittleEndianParser(it,msg.cov_xb);
    qiLittleEndianParser(it,msg.cov_yz);
    qiLittleEndianParser(it,msg.cov_yb);
    qiLittleEndianParser(it,msg.cov_zb);
    if (it > itEnd)
    {
        node->log(LogLevel::ERROR, "Parse error: iterator past end.");
        return false;
    }
    return true;
};

/**
 * PosCovGeodeticParser
 * @brief Qi based parser for the SBF block "PosCovGeodetic"
 */
template<typename It>
bool PosCovGeodeticParser(ROSaicNodeBase* node, It it, It itEnd, PosCovGeodeticMsg& msg)
{    
    if(!BlockHeaderParser(node, it, msg.block_header))
        return false;
    if (msg.block_header.id != 5906)
    {
        node->log(LogLevel::ERROR, "Parse error: Wrong header ID " + std::to_string(msg.block_header.id));
        return false;
    }
    qiLittleEndianParser(it, msg.mode);
    qiLittleEndianParser(it, msg.error);
    qiLittleEndianParser(it, msg.cov_latlat);
    qiLittleEndianParser(it, msg.cov_lonlon);
    qiLittleEndianParser(it, msg.cov_hgthgt);
    qiLittleEndianParser(it, msg.cov_bb);
    qiLittleEndianParser(it, msg.cov_latlon);
    qiLittleEndianParser(it, msg.cov_lathgt);
    qiLittleEndianParser(it, msg.cov_latb);
    qiLittleEndianParser(it, msg.cov_lonhgt);
    qiLittleEndianParser(it, msg.cov_lonb);
    qiLittleEndianParser(it, msg.cov_hb);
    if (it > itEnd)
    {
        node->log(LogLevel::ERROR, "Parse error: iterator past end.");
        return false;
    }
    return true;
};

/**
 * VelCovCartesianParser
 * @brief Qi based parser for the SBF block "VelCovCartesian"
 */
template<typename It>
bool VelCovCartesianParser(ROSaicNodeBase* node, It it, It itEnd, VelCovCartesianMsg& msg)
{    
    if(!BlockHeaderParser(node, it, msg.block_header))
        return false;
    if (msg.block_header.id != 5907)
    {
        node->log(LogLevel::ERROR, "Parse error: Wrong header ID " + std::to_string(msg.block_header.id));
        return false;
    }
    qiLittleEndianParser(it, msg.mode);
    qiLittleEndianParser(it, msg.error);
    qiLittleEndianParser(it, msg.cov_vxvx);
    qiLittleEndianParser(it, msg.cov_vyvy);
    qiLittleEndianParser(it, msg.cov_vzvz);
    qiLittleEndianParser(it, msg.cov_dtdt);
    qiLittleEndianParser(it, msg.cov_vxvy);
    qiLittleEndianParser(it, msg.cov_vxvz);
    qiLittleEndianParser(it, msg.cov_vxdt);
    qiLittleEndianParser(it, msg.cov_vyvz);
    qiLittleEndianParser(it, msg.cov_vydt);
    qiLittleEndianParser(it, msg.cov_vzdt);
    if (it > itEnd)
    {
        node->log(LogLevel::ERROR, "Parse error: iterator past end.");
        return false;
    }
    return true;
};

/**
 * VelCovGeodeticParser
 * @brief Qi based parser for the SBF block "VelCovGeodetic"
 */
template<typename It>
bool VelCovGeodeticParser(ROSaicNodeBase* node, It it, It itEnd, VelCovGeodeticMsg& msg)
{    
    if(!BlockHeaderParser(node, it, msg.block_header))
        return false;
    if (msg.block_header.id != 5908)
    {
        node->log(LogLevel::ERROR, "Parse error: Wrong header ID " + std::to_string(msg.block_header.id));
        return false;
    }
    qiLittleEndianParser(it, msg.mode);
    qiLittleEndianParser(it, msg.error);
    qiLittleEndianParser(it, msg.cov_vnvn);
    qiLittleEndianParser(it, msg.cov_veve);
    qiLittleEndianParser(it, msg.cov_vuvu);
    qiLittleEndianParser(it, msg.cov_dtdt);
    qiLittleEndianParser(it, msg.cov_vnve);
    qiLittleEndianParser(it, msg.cov_vnvu);
    qiLittleEndianParser(it, msg.cov_vndt);
    qiLittleEndianParser(it, msg.cov_vevu);
    qiLittleEndianParser(it, msg.cov_vedt);
    qiLittleEndianParser(it, msg.cov_vudt);
    if (it > itEnd)
    {
        node->log(LogLevel::ERROR, "Parse error: iterator past end.");
        return false;
    }
    return true;
};

/**
 * INSNavGeodParser
 * @brief Qi based parser for the SBF block "INSNavGeod"
 */
template<typename It>
bool INSNavGeodParser(ROSaicNodeBase* node, It it, It itEnd, INSNavGeodMsg& msg, bool use_ros_axis_orientation)
{    
    if(!BlockHeaderParser(node, it, msg.block_header))
        return false;
    if ((msg.block_header.id != 4226) && (msg.block_header.id != 4230))
    {
        node->log(LogLevel::ERROR, "Parse error: Wrong header ID " + std::to_string(msg.block_header.id));
        return false;
    }
    qiLittleEndianParser(it, msg.gnss_mode);
    qiLittleEndianParser(it, msg.error);
    qiLittleEndianParser(it, msg.info);
    qiLittleEndianParser(it, msg.gnss_age);
    qiLittleEndianParser(it, msg.latitude);
    qiLittleEndianParser(it, msg.longitude);
    qiLittleEndianParser(it, msg.height);
    qiLittleEndianParser(it, msg.undulation);
    qiLittleEndianParser(it, msg.accuracy);
    qiLittleEndianParser(it, msg.latency);
    qiLittleEndianParser(it, msg.datum);
    ++it; //reserved
    qiLittleEndianParser(it, msg.sb_list);
    if((msg.sb_list & 1) !=0)
    {
        qiLittleEndianParser(it, msg.latitude_std_dev);
        qiLittleEndianParser(it, msg.longitude_std_dev);
        qiLittleEndianParser(it, msg.height_std_dev);
    }
    else
    {
        msg.latitude_std_dev  = DO_NOT_USE_VALUE;
        msg.longitude_std_dev = DO_NOT_USE_VALUE;
        msg.height_std_dev    = DO_NOT_USE_VALUE;
    }
    if((msg.sb_list & 2) !=0)
    {
        qiLittleEndianParser(it, msg.heading);
        qiLittleEndianParser(it, msg.pitch);
        qiLittleEndianParser(it, msg.roll);
        if (use_ros_axis_orientation)
        {
            msg.heading = -msg.heading + parsing_utilities::pi_half;
            msg.pitch   = -msg.pitch;
        }
    }
    else
    {
        msg.heading = DO_NOT_USE_VALUE;
        msg.pitch   = DO_NOT_USE_VALUE;
        msg.roll    = DO_NOT_USE_VALUE;
    }
    if((msg.sb_list & 4) !=0)
    {
        qiLittleEndianParser(it, msg.heading_std_dev);
        qiLittleEndianParser(it, msg.pitch_std_dev);
        qiLittleEndianParser(it, msg.roll_std_dev);
    }
    else
    {
        msg.heading_std_dev = DO_NOT_USE_VALUE;
        msg.pitch_std_dev   = DO_NOT_USE_VALUE;
        msg.roll_std_dev    = DO_NOT_USE_VALUE;
    }
    if((msg.sb_list & 8) !=0)
    {
        qiLittleEndianParser(it, msg.ve);
        qiLittleEndianParser(it, msg.vn);
        qiLittleEndianParser(it, msg.vu);
    }
    else
    {
        msg.ve = DO_NOT_USE_VALUE;
        msg.vn = DO_NOT_USE_VALUE;
        msg.vu = DO_NOT_USE_VALUE;
    }
    if((msg.sb_list & 16) !=0)
    {
        qiLittleEndianParser(it, msg.ve_std_dev);
        qiLittleEndianParser(it, msg.vn_std_dev);
        qiLittleEndianParser(it, msg.vu_std_dev);
    }
    else
    {
        msg.ve_std_dev = DO_NOT_USE_VALUE;
        msg.vn_std_dev = DO_NOT_USE_VALUE;
        msg.vu_std_dev = DO_NOT_USE_VALUE;
    }
    if((msg.sb_list & 32) !=0)
    {
        qiLittleEndianParser(it, msg.latitude_longitude_cov);
        qiLittleEndianParser(it, msg.latitude_height_cov);
        qiLittleEndianParser(it, msg.longitude_height_cov);
    }
    else
    {
        msg.latitude_longitude_cov = DO_NOT_USE_VALUE;
        msg.latitude_height_cov    = DO_NOT_USE_VALUE;
        msg.longitude_height_cov   = DO_NOT_USE_VALUE;
    }
    if((msg.sb_list & 64) !=0)
    {
        qiLittleEndianParser(it, msg.heading_pitch_cov);
        qiLittleEndianParser(it, msg.heading_roll_cov);
        qiLittleEndianParser(it, msg.pitch_roll_cov);
        if (use_ros_axis_orientation)
        {   
            msg.heading_roll_cov = -msg.heading_roll_cov;
            msg.pitch_roll_cov   = -msg.pitch_roll_cov;
        }
    }
    else
    {
        msg.heading_pitch_cov = DO_NOT_USE_VALUE;
        msg.heading_roll_cov  = DO_NOT_USE_VALUE;
        msg.pitch_roll_cov    = DO_NOT_USE_VALUE;
    }
    if((msg.sb_list & 128) !=0)
    {
        qiLittleEndianParser(it, msg.ve_vn_cov);
        qiLittleEndianParser(it, msg.ve_vu_cov);
        qiLittleEndianParser(it, msg.vn_vu_cov);
    }
    else
    {
        msg.ve_vn_cov = DO_NOT_USE_VALUE;
        msg.ve_vu_cov = DO_NOT_USE_VALUE;
        msg.vn_vu_cov = DO_NOT_USE_VALUE;
    }
    if (it > itEnd)
    {
        node->log(LogLevel::ERROR, "Parse error: iterator past end.");
        return false;
    }
    return true;
};

/**
 * IMUSetupParser
 * @brief Qi based parser for the SBF block "IMUSetup"
 */
template<typename It>
bool IMUSetupParser(ROSaicNodeBase* node, It it, It itEnd, IMUSetupMsg& msg, bool use_ros_axis_orientation)
{    
    if(!BlockHeaderParser(node, it, msg.block_header))
        return false;
    if (msg.block_header.id != 4224)
    {
        node->log(LogLevel::ERROR, "Parse error: Wrong header ID " + std::to_string(msg.block_header.id));
        return false;
    }
    ++it; //reserved
    qiLittleEndianParser(it, msg.serial_port);
    qiLittleEndianParser(it, msg.ant_lever_arm_x);
    qiLittleEndianParser(it, msg.ant_lever_arm_y);
    qiLittleEndianParser(it, msg.ant_lever_arm_z);
    qiLittleEndianParser(it, msg.theta_x);
    qiLittleEndianParser(it, msg.theta_y);
    qiLittleEndianParser(it, msg.theta_z);
    if (use_ros_axis_orientation)
    {
        msg.ant_lever_arm_y = -msg.ant_lever_arm_y;
        msg.ant_lever_arm_z = -msg.ant_lever_arm_z;
        msg.theta_x = parsing_utilities::wrapAngle180to180(msg.theta_x - 180.0f);
    }
    if (it > itEnd)
    {
        node->log(LogLevel::ERROR, "Parse error: iterator past end.");
        return false;
    }
    return true;
};

/**
 * VelSensorSetupParser
 * @brief Qi based parser for the SBF block "VelSensorSetup"
 */
template<typename It>
bool VelSensorSetupParser(ROSaicNodeBase* node, It it, It itEnd, VelSensorSetupMsg& msg, bool use_ros_axis_orientation)
{    
    if(!BlockHeaderParser(node, it, msg.block_header))
        return false;
    if (msg.block_header.id != 4244)
    {
        node->log(LogLevel::ERROR, "Parse error: Wrong header ID " + std::to_string(msg.block_header.id));
        return false;
    }
    ++it; //reserved
    qiLittleEndianParser(it, msg.port);
    qiLittleEndianParser(it, msg.lever_arm_x);
    qiLittleEndianParser(it, msg.lever_arm_y);
    qiLittleEndianParser(it, msg.lever_arm_z);    
    if (use_ros_axis_orientation)
    {
        msg.lever_arm_y = -msg.lever_arm_y;
        msg.lever_arm_z = -msg.lever_arm_z;
    }
    if (it > itEnd)
    {
        node->log(LogLevel::ERROR, "Parse error: iterator past end.");
        return false;
    }
    return true;
};

/**
 * ExtSensorMeasParser
 * @brief Qi based parser for the SBF block "ExtSensorMeas"
 */
template<typename It>
bool ExtSensorMeasParser(ROSaicNodeBase* node, It it, It itEnd, ExtSensorMeasMsg& msg, bool use_ros_axis_orientation)
{    
    if(!BlockHeaderParser(node, it, msg.block_header))
        return false;
    if (msg.block_header.id != 4050)
    {
        node->log(LogLevel::ERROR, "Parse error: Wrong header ID " + std::to_string(msg.block_header.id));
        return false;
    }
    qiLittleEndianParser(it, msg.n);
    qiLittleEndianParser(it, msg.sb_length);
    if (msg.sb_length != 28)
    {
        node->log(LogLevel::ERROR, "Parse error: Wrong sb_length " + std::to_string(msg.sb_length));
        return false;
    }

    msg.acceleration_x = std::numeric_limits<double>::quiet_NaN();
    msg.acceleration_y = std::numeric_limits<double>::quiet_NaN();
    msg.acceleration_z = std::numeric_limits<double>::quiet_NaN();

    msg.angular_rate_x = std::numeric_limits<double>::quiet_NaN();
    msg.angular_rate_y = std::numeric_limits<double>::quiet_NaN();
    msg.angular_rate_z = std::numeric_limits<double>::quiet_NaN();

    msg.velocity_x = std::numeric_limits<double>::quiet_NaN();
    msg.velocity_y = std::numeric_limits<double>::quiet_NaN();
    msg.velocity_z = std::numeric_limits<double>::quiet_NaN();

    msg.std_dev_x = std::numeric_limits<double>::quiet_NaN();
    msg.std_dev_y = std::numeric_limits<double>::quiet_NaN();
    msg.std_dev_z = std::numeric_limits<double>::quiet_NaN();

    msg.sensor_temperature = -32768.0f; //do not use value
    msg.zero_velocity_flag = std::numeric_limits<double>::quiet_NaN();

    msg.source.resize(msg.n);
    msg.sensor_model.resize(msg.n);
    msg.type.resize(msg.n);
    msg.obs_info.resize(msg.n); 
    for (size_t i = 0; i < msg.n; i++)
    {
        qiLittleEndianParser(it, msg.source[i]);
        qiLittleEndianParser(it, msg.sensor_model[i]);
        qiLittleEndianParser(it, msg.type[i]);
        qiLittleEndianParser(it, msg.obs_info[i]);

        switch (msg.type[i])
        {
        case 0:
        {
            qiLittleEndianParser(it, msg.acceleration_x);
            qiLittleEndianParser(it, msg.acceleration_y);
            qiLittleEndianParser(it, msg.acceleration_z);
            if (!use_ros_axis_orientation)
            {
                msg.acceleration_y = -msg.acceleration_y;
                msg.acceleration_z = -msg.acceleration_z;
            }
            break;
        }
        case 1:
        {
            qiLittleEndianParser(it, msg.angular_rate_x);
            qiLittleEndianParser(it, msg.angular_rate_y);
            qiLittleEndianParser(it, msg.angular_rate_z);
            if (!use_ros_axis_orientation)
            {
                msg.angular_rate_y = -msg.angular_rate_y;
                msg.angular_rate_z = -msg.angular_rate_z;
            }
            break;
        }
        case 3:
        {
            qi::parse(it, it + 2, qi::little_word, msg.sensor_temperature);
            msg.sensor_temperature /= 100.0f;
            std::advance(it, 22); // reserved
            break;
        }
        case 4:
        {
            qiLittleEndianParser(it, msg.velocity_x);
            qiLittleEndianParser(it, msg.velocity_y);
            qiLittleEndianParser(it, msg.velocity_z);
            qiLittleEndianParser(it, msg.std_dev_x);
            qiLittleEndianParser(it, msg.std_dev_y);
            qiLittleEndianParser(it, msg.std_dev_z);
            if (use_ros_axis_orientation)
            {
                msg.velocity_y = -msg.velocity_y;
                msg.velocity_z = -msg.velocity_z;
            }
            break;
        }
        case 20:
        {
            qiLittleEndianParser(it, msg.zero_velocity_flag);
            std::advance(it, 16); // reserved
            break;
        }
        default:
        {
            node->log(LogLevel::ERROR, "Unknown external sensor measurement type in SBF ExtSensorMeas.");
            std::advance(it, 24);
            break;
        }
        } 
    }
    if (it > itEnd)
    {
        node->log(LogLevel::ERROR, "Parse error: iterator past end.");
        return false;
    }
    return true;
};

#endif // SBFStructs_HPP