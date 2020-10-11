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
#define NR_OF_LOGICALCHANNELS  80
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
#define MAXSB_NBRANTENNA  4
#endif
//! Max number of bytes that ChannelSatInfo sub-block can consist of
#ifndef MAXSB_CHANNELSATINFO
#define MAXSB_CHANNELSATINFO   (NR_OF_LOGICALCHANNELS + MAX_NB_INMARSATCHANNELS)
#endif
//! Max number of bytes that ChannelStateInfo sub-block can consist of
#ifndef MAXSB_CHANNELSTATEINFO
#define MAXSB_CHANNELSTATEINFO   (MAXSB_CHANNELSATINFO * MAXSB_NBRANTENNA)
#endif
//! Max number of bytes that MeasEpochChannelType1 sub-block can consist of
#ifndef MAXSB_MEASEPOCH_T1
#define MAXSB_MEASEPOCH_T1 (NR_OF_LOGICALCHANNELS + MAX_NB_INMARSATCHANNELS)
#endif
//! Max number of bytes that MeasEpochChannelType2 sub-block can consist of
#ifndef MAXSB_MEASEPOCH_T2
#define MAXSB_MEASEPOCH_T2 ((MAXSB_MEASEPOCH_T1) * ( ((MAX_NR_OF_SIGNALS_PER_SATELLITE) * (NR_OF_ANTENNAS)) -1))
#endif

// ROSaic includes
#include "ssn_types.hpp"

#if defined(__GNUC__) || defined(__ARMCC__)
/* Before the advent of the CPMF platform, double data types were always
 * 32-bit aligned, meaning that the struct were aligned to an address
 * that was divisible by 4. On the CPMF, double data types are 64-bit  
 * aligned. The "packed, aligned(4)" attribute combination is necessary 
 * to enforce 32-bit alignment for double data types and to port the SBF 
 * encoding/decoding functionality to the CPMF.
 */
// The aligned variable attribute specifies a minimum alignment for the variable or structure field, measured in bytes.
// The aligned attribute only increases the alignment for a struct or struct member. For a variable that is not in a 
// structure, the minimum alignment is the natural alignment of the variable type. To set the alignment in a structure 
// to any value greater than 0, use the packed variable attribute. Without packed, the minimum alignment is the natural 
// alignment of the variable type.

#  define SBFDOUBLE double __attribute__((packed, aligned(4)))
#else
#  define SBFDOUBLE double
#endif


/* Force packing the structs on 4-byte alignment (needed for GCC 64 bit compilations) */
#pragma pack(push,4)
// Clearly, there will be padding bytes for some structs, but those will be ignored by our decoding software, as also 
// suggested in the firmware.
// Example usages of pragma directives:
// #pragma warn +xxx (To show the warning)
// #pragma startup func1  and
// #pragma exit func2  would not work with GCC compilers (just ignored)
// printf("Size of A is: %ld", sizeof(A));  [%d works fine with signed, unsigned and negative integer values, l stands 
// for long], recall we want structs to avoid wasting = padding
// To force compiler to use 1 byte packaging: #pragma pack(1) 
// Same result for struct s {int i...} __attribute__((packed)); could be shown via printf("%zu ", offsetof(s, i));
// #pragma pack()   // n defaults to 8; equivalent to /Zp8, Valid values are 1, 2, 4, 8, and 16, forces the maximum 
// alignment of each field to be the value specified by n
// push: Pushes the current alignment setting on an internal stack and then optionally sets the new alignment, 
// identifier is also allowed

/**
 * @file sbf_structs.hpp
 * @brief Declares and defines structs into which SBF blocks are unpacked then shipped to handler functions
 * @date 17/08/20 
*/

/**
 * @brief Struct for the SBF block's header message
 */
typedef struct
{
	uint8_t sync_1; //!< first sync byte is $ or 0x24
	uint8_t sync_2; //!< 2nd sync byte is @ or 0x40
	uint16_t crc; //!< The check sum 
	uint16_t id; //!< This is the block ID
	uint16_t length; //!< Length of the entire message including the header. A multiple of 4 between 8 and 4096
} BlockHeader_t;


/**
 * @class PVTCartesian
 * @brief Struct for the SBF block "PVTCartesian"
 */
struct PVTCartesian
{
	BlockHeader_t  block_header;       

	/* Time Header */
	uint32_t       tow;          
	uint16_t       wnc;          

	uint8_t        mode;
	uint8_t        error;        
	SBFDOUBLE      x;            
	SBFDOUBLE      y;            
	SBFDOUBLE      z;     
	float          undulation;
	float          vx;           
	float          vy;           
	float          vz;  
	float          cog;   
	SBFDOUBLE      rx_clk_bias; 
	float          rx_clk_drift;   
	uint8_t        time_system; 
	uint8_t        datum;
	uint8_t        nr_sv; 
	uint8_t        wa_corr_info;
	uint16_t       reference_id;
	uint16_t       mean_corr_age; 
	uint32_t       signal_info;
	uint8_t        alert_flag;
	uint8_t        nr_bases;
	uint16_t       ppp_info;
	uint16_t       latency;
	uint16_t       h_accuracy;
	uint16_t       v_accuracy;
	uint8_t        misc;
};

/**
 * @class PVTGeodetic
 * @brief Struct for the SBF block "PVTGeodetic"
 */
struct PVTGeodetic
{
	BlockHeader_t  block_header;       

	/* Time Header */
	uint32_t       tow;          
	uint16_t       wnc;          

	uint8_t        mode;
	uint8_t        error;        
	SBFDOUBLE      latitude;            
	SBFDOUBLE      longitude;            
	SBFDOUBLE      height;     
	float          undulation;
	float          vn;           
	float          ve;           
	float          vu;  
	float          cog;   
	SBFDOUBLE      rx_clk_bias; 
	float          rx_clk_drift;   
	uint8_t        time_system; 
	uint8_t        datum;
	uint8_t        nr_sv; 
	uint8_t        wa_corr_info;
	uint16_t       reference_id;
	uint16_t       mean_corr_age; 
	uint32_t       signal_info;
	uint8_t        alert_flag;
	uint8_t        nr_bases;
	uint16_t       ppp_info;
	uint16_t       latency;
	uint16_t       h_accuracy;
	uint16_t       v_accuracy;
	uint8_t        misc;
};

/**
 * @class AttEuler
 * @brief Struct for the SBF block "AttEuler"
 */
struct AttEuler
{
	BlockHeader_t  block_header;       

	/* Time Header */
	uint32_t       tow;          
	uint16_t       wnc;          

	uint8_t        nr_sv;
	uint8_t        error;   
	uint16_t       mode;
	uint16_t       reserved;
	float          heading;
	float          pitch;
	float          roll;
	float          pitch_dot;
	float          roll_dot;
	float          heading_dot;
};


/**
 * @class AttCovEuler
 * @brief Struct for the SBF block "AttCovEuler"
 */
struct AttCovEuler
{
	BlockHeader_t  block_header;       

	/* Time Header */
	uint32_t       tow;          
	uint16_t       wnc;          

	uint8_t        reserved;         
	uint8_t        error; 
	float          cov_headhead;
	float          cov_pitchpitch;
	float          cov_rollroll;
	float          cov_headpitch;
	float          cov_headroll;
	float          cov_pitchroll;
};

/**
 * @class ChannelStateInfo
 * @brief Struct for the SBF sub-block "ChannelStateInfo"
 */
typedef struct 
{
	uint8_t        antenna;      
	uint8_t        reserved;    
	uint16_t       tracking_status;
	uint16_t       pvt_status;    
	uint16_t       pvt_info;      
} ChannelStateInfo;

/**
 * @class ChannelSatInfo
 * @brief Struct for the SBF sub-block "ChannelSatInfo"
 */
typedef struct 
{
	uint8_t        sv_id;         
	uint8_t        freq_nr;       
	uint8_t        reserved1[2]; 
	uint16_t       az_rise_set;   
	uint16_t       health_status; 
	int8_t         elev;         
	uint8_t        n2;           
	uint8_t        channel;      
	uint8_t        reserved2;    
} ChannelSatInfo;

//! Max number of bytes that the Data part of the ChannelStatus struct can consist of
#ifndef SBF_CHANNELSTATUS_DATA_LENGTH
#define SBF_CHANNELSTATUS_DATA_LENGTH  MAXSB_CHANNELSATINFO * sizeof(ChannelSatInfo) + MAXSB_CHANNELSTATEINFO * sizeof(ChannelStateInfo)
#endif

/**
 * @class ChannelStatus
 * @brief Struct for the SBF block "ChannelStatus"
 */
struct ChannelStatus
{
	BlockHeader_t  block_header;       

	/* Time Header */
	uint32_t       tow;          
	uint16_t       wnc;          

	uint8_t        n;            
	uint8_t        sb1_size;      
	uint8_t        sb2_size;      
	uint8_t        reserved[3];  
	uint8_t        data[SBF_CHANNELSTATUS_DATA_LENGTH];
};

/**
 * @class MeasEpochChannelType2
 * @brief Struct for the SBF sub-block "MeasEpochChannelType2"
 */
typedef struct
{
  uint8_t        type;         
  uint8_t        lock_time;     
  uint8_t        cn0;          
  uint8_t        offsets_msb;   
  int8_t         carrier_msb;   
  uint8_t        obs_info;      
  uint16_t       code_offset_lsb;
  uint16_t       carrier_lsb;   
  uint16_t       doppler_offset_lsb;
} MeasEpochChannelType2;

/**
 * @class MeasEpochChannelType1
 * @brief Struct for the SBF sub-block "MeasEpochChannelType1"
 */
typedef struct
{
  uint8_t        rx_channel;    
  uint8_t        type;         
  uint8_t        sv_id;         
  uint8_t        misc;         
  uint32_t       code_lsb;      
  int32_t        doppler;      
  uint16_t       carrier_lsb;   
  int8_t         carrier_msb;   
  uint8_t        cn0;          
  uint16_t       lock_time;     
  uint8_t        obs_info;      
  uint8_t        n_type2;      
} MeasEpochChannelType1;

//! Max number of bytes that the data part of the MeasEpoch struct can consist of
#ifndef MEASEPOCH_DATA_LENGTH
#define MEASEPOCH_DATA_LENGTH (MAXSB_MEASEPOCH_T1 * sizeof(MeasEpochChannelType1) + MAXSB_MEASEPOCH_T2 * sizeof(MeasEpochChannelType2))
#endif 

/**
 * @class MeasEpoch
 * @brief Struct for the SBF block "MeasEpoch"
 */
struct MeasEpoch
{
  BlockHeader_t  block_header;       

  /* Time Header */
  uint32_t       tow;          
  uint16_t       wnc;          

  /* MeasEpoch Header */
  uint8_t        n;            
  uint8_t        sb1_size;      
  uint8_t        sb2_size;      

  uint8_t        common_flags;  
  uint8_t        cum_clk_jumps;  
  uint8_t        reserved;     
  uint8_t        data[MEASEPOCH_DATA_LENGTH];
};

/**
 * @class DOP
 * @brief Struct for the SBF block "DOP"
 */
struct DOP
{
	BlockHeader_t  block_header;       

	/* Time Header */
	uint32_t       tow;          
	uint16_t       wnc;   
       
	uint8_t        nr_sv;
	uint8_t        reserved;
	uint16_t       pdop;
	uint16_t       tdop;
	uint16_t       hdop;
	uint16_t       vdop;
	float          hpl;
	float          vpl;
};

/**
 * @class ReceiverSetup
 * @brief Struct for the SBF block "ReceiverSetup"
 */
struct ReceiverSetup
{
	BlockHeader_t  block_header;       

	/* Time Header */
	uint32_t       tow;          
	uint16_t       wnc;   
	
	uint8_t        reserved[2];
	char           marker_name[60];
	char           marker_number[20];
	char           observer[20];
	char           agency[40];
	char           rx_serial_number[20];
	char           rx_name[20];
	char           rx_version[20];
	char           ant_serial_nbr[20];
	char           ant_type[20];
	float          delta_h;       /* [m] */
	float          delta_e;       /* [m] */
	float          delta_n;       /* [m] */
	char           marker_type[20];
	char           gnss_fw_version[40];
};

/**
 * @class QualityInd
 * @brief Struct for the SBF block "QualityInd"
 */
struct QualityInd
{
	BlockHeader_t  block_header;       

	/* Time Header */
	uint32_t       tow;          
	uint16_t       wnc; 
	
	uint8_t        n;
	uint8_t        reserved;
	uint16_t       indicators[40];
};

/**
 * @brief Struct for the SBF sub-block "AGCState"
 */
typedef struct
{
  uint8_t        frontend_id;   
  int8_t         gain;         
  uint8_t        sample_var;    
  uint8_t        blanking_stat; 
} AGCState_t;

/**
 * @class ReceiverStatus
 * @brief Struct for the SBF block "ReceiverStatus"
 */
struct ReceiverStatus
{
	BlockHeader_t  block_header;       

	/* Time Header */
	uint32_t       tow;          
	uint16_t       wnc; 
	
	uint8_t        cpu_load;      
	uint8_t        ext_error;     
	uint32_t       up_time;       
	uint32_t       rx_status;     
	uint32_t       rx_error;  
	uint8_t        n;
	uint8_t        sb_lngth;
	uint8_t        cmd_count;
	uint8_t        temperature;
	AGCState_t     agc_state[18];
};  

/**
 * @class PosCovCartesian
 * @brief Struct for the SBF block "PosCovCartesian"
 */
struct PosCovCartesian
{
	BlockHeader_t  block_header;       

	/* Time Header */
	uint32_t       tow;          
	uint16_t       wnc;          

	uint8_t        mode;         
	uint8_t        error;        
	float          cov_xx;       
	float          cov_yy;       
	float          cov_zz;       
	float          cov_bb;       
	float          cov_xy;       
	float          cov_xz;       
	float          cov_xb;       
	float          cov_yz;       
	float          cov_yb;       
	float          cov_zb;       
};

/**
 * @class PosCovGeodetic
 * @brief Struct for the SBF block "PosCovGeodetic"
 */
struct PosCovGeodetic
{
	BlockHeader_t  block_header;       

	/* Time Header */
	uint32_t       tow;          
	uint16_t       wnc;          

	uint8_t        mode;         
	uint8_t        error;        
	float          cov_latlat;       
	float          cov_lonlon;       
	float          cov_hgthgt;       
	float          cov_bb;       
	float          cov_latlon;       
	float          cov_lathgt;       
	float          cov_latb;       
	float          cov_lonhgt;       
	float          cov_lonb;       
	float          cov_hb;       
};

/**
 * @class VelCovCartesian
 * @brief Struct for the SBF block "VelCovCartesian"
 */
struct VelCovCartesian
{
	BlockHeader_t  block_header;       

	/* Time Header */
	uint32_t       tow;          
	uint16_t       wnc;          

	uint8_t        mode;         
	uint8_t        error;        
	float          cov_vxvx;     
	float          cov_vyvy;     
	float          cov_vzvz;     
	float          cov_dtdt;     
	float          cov_vxvy;     
	float          cov_vxvz;     
	float          cov_vxdt;     
	float          cov_vyvz;     
	float          cov_vydt;     
	float          cov_vzdt;     
};

/**
 * @class VelCovGeodetic
 * @brief Struct for the SBF block "VelCovGeodetic"
 */
struct VelCovGeodetic
{
	BlockHeader_t  block_header;       

	/* Time Header */
	uint32_t       tow;          
	uint16_t       wnc;          

	uint8_t        mode;         
	uint8_t        error;        
	float          cov_vnvn;     
	float          cov_veve;     
	float          cov_vuvu;     
	float          cov_dtdt;     
	float          cov_vnve;     
	float          cov_vnvu;     
	float          cov_vndt;     
	float          cov_vevu;     
	float          cov_vedt;     
	float          cov_vudt;     
};

#pragma pack ( pop ) 
// The above form of the pack pragma affects only class, struct, and union type declarations between 
// push and pop directives. (A pop directive with no prior push results in a warning diagnostic from the compiler.)

/**
 * @brief CRC look-up table for fast computation of the 16-bit CRC for SBF blocks.
 * 
 * Provided by Septenrio (c) 2020 Septentrio N.V./S.A., Belgium.
 */
static const uint16_t CRC_LOOK_UP[256] = {
    0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50a5, 0x60c6, 0x70e7,
    0x8108, 0x9129, 0xa14a, 0xb16b, 0xc18c, 0xd1ad, 0xe1ce, 0xf1ef,
    0x1231, 0x0210, 0x3273, 0x2252, 0x52b5, 0x4294, 0x72f7, 0x62d6,
    0x9339, 0x8318, 0xb37b, 0xa35a, 0xd3bd, 0xc39c, 0xf3ff, 0xe3de,
    0x2462, 0x3443, 0x0420, 0x1401, 0x64e6, 0x74c7, 0x44a4, 0x5485,
    0xa56a, 0xb54b, 0x8528, 0x9509, 0xe5ee, 0xf5cf, 0xc5ac, 0xd58d,
    0x3653, 0x2672, 0x1611, 0x0630, 0x76d7, 0x66f6, 0x5695, 0x46b4,
    0xb75b, 0xa77a, 0x9719, 0x8738, 0xf7df, 0xe7fe, 0xd79d, 0xc7bc,
    0x48c4, 0x58e5, 0x6886, 0x78a7, 0x0840, 0x1861, 0x2802, 0x3823,
    0xc9cc, 0xd9ed, 0xe98e, 0xf9af, 0x8948, 0x9969, 0xa90a, 0xb92b,
    0x5af5, 0x4ad4, 0x7ab7, 0x6a96, 0x1a71, 0x0a50, 0x3a33, 0x2a12,
    0xdbfd, 0xcbdc, 0xfbbf, 0xeb9e, 0x9b79, 0x8b58, 0xbb3b, 0xab1a,
    0x6ca6, 0x7c87, 0x4ce4, 0x5cc5, 0x2c22, 0x3c03, 0x0c60, 0x1c41,
    0xedae, 0xfd8f, 0xcdec, 0xddcd, 0xad2a, 0xbd0b, 0x8d68, 0x9d49,
    0x7e97, 0x6eb6, 0x5ed5, 0x4ef4, 0x3e13, 0x2e32, 0x1e51, 0x0e70,
    0xff9f, 0xefbe, 0xdfdd, 0xcffc, 0xbf1b, 0xaf3a, 0x9f59, 0x8f78,
    0x9188, 0x81a9, 0xb1ca, 0xa1eb, 0xd10c, 0xc12d, 0xf14e, 0xe16f,
    0x1080, 0x00a1, 0x30c2, 0x20e3, 0x5004, 0x4025, 0x7046, 0x6067,
    0x83b9, 0x9398, 0xa3fb, 0xb3da, 0xc33d, 0xd31c, 0xe37f, 0xf35e,
    0x02b1, 0x1290, 0x22f3, 0x32d2, 0x4235, 0x5214, 0x6277, 0x7256,
    0xb5ea, 0xa5cb, 0x95a8, 0x8589, 0xf56e, 0xe54f, 0xd52c, 0xc50d,
    0x34e2, 0x24c3, 0x14a0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
    0xa7db, 0xb7fa, 0x8799, 0x97b8, 0xe75f, 0xf77e, 0xc71d, 0xd73c,
    0x26d3, 0x36f2, 0x0691, 0x16b0, 0x6657, 0x7676, 0x4615, 0x5634,
    0xd94c, 0xc96d, 0xf90e, 0xe92f, 0x99c8, 0x89e9, 0xb98a, 0xa9ab,
    0x5844, 0x4865, 0x7806, 0x6827, 0x18c0, 0x08e1, 0x3882, 0x28a3,
    0xcb7d, 0xdb5c, 0xeb3f, 0xfb1e, 0x8bf9, 0x9bd8, 0xabbb, 0xbb9a,
    0x4a75, 0x5a54, 0x6a37, 0x7a16, 0x0af1, 0x1ad0, 0x2ab3, 0x3a92,
    0xfd2e, 0xed0f, 0xdd6c, 0xcd4d, 0xbdaa, 0xad8b, 0x9de8, 0x8dc9,
    0x7c26, 0x6c07, 0x5c64, 0x4c45, 0x3ca2, 0x2c83, 0x1ce0, 0x0cc1,
    0xef1f, 0xff3e, 0xcf5d, 0xdf7c, 0xaf9b, 0xbfba, 0x8fd9, 0x9ff8,
    0x6e17, 0x7e36, 0x4e55, 0x5e74, 0x2e93, 0x3eb2, 0x0ed1, 0x1ef0
};

#endif // SBFStructs_HPP