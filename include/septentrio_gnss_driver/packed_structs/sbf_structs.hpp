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

// Boost
#include <boost/spirit/include/qi.hpp>
#include <boost/spirit/include/phoenix.hpp>
#include <boost/fusion/include/adapt_struct.hpp>
#include <boost/spirit/repository/include/qi_advance.hpp>

struct ID_t
{
    ID_t(uint16_t ui1=0, uint8_t ui2=0) : 
        id(ui1), 
        rev(ui2) 
    { }

    static ID_t make(uint16_t w)
    {
        return ID_t((w & 8191), (w >> 13u));
    }

    uint16_t id;
    uint8_t  rev;
};

/**
 * @file sbf_structs.hpp
 * @brief Declares and defines structs into which SBF blocks are unpacked then
 * shipped to handler functions
 * @date 17/08/20
 */

/**
 * @brief Struct for the SBF block's header message
 */
typedef struct
{
    uint8_t sync_1;  //!< first sync byte is $ or 0x24
    uint8_t sync_2;  //!< 2nd sync byte is @ or 0x40
    uint16_t crc;    //!< The check sum
    union
    {
        uint16_t null; //!< This is only used for parsing
        struct
        {
            uint16_t id : 13;
            uint8_t rev :  3;
        } ID; //!< This is the block ID, separated in id and rev
    };
    uint16_t length; //!< Length of the entire message including the header. A
                     //!< multiple of 4 between 8 and 4096
} BlockHeader_t;

/**
 * @class PVTCartesian
 * @brief Struct for the SBF block "PVTCartesian"
 */
struct PVTCartesian
{
    BlockHeader_t block_header;

    /* Time Header */
    uint32_t tow;
    uint16_t wnc;

    uint8_t mode;
    uint8_t error;
    double x;
    double y;
    double z;
    float undulation;
    float vx;
    float vy;
    float vz;
    float cog;
    double rx_clk_bias;
    float rx_clk_drift;
    uint8_t time_system;
    uint8_t datum;
    uint8_t nr_sv;
    uint8_t wa_corr_info;
    uint16_t reference_id;
    uint16_t mean_corr_age;
    uint32_t signal_info;
    uint8_t alert_flag;
    uint8_t nr_bases;
    uint16_t ppp_info;
    uint16_t latency;
    uint16_t h_accuracy;
    uint16_t v_accuracy;
    uint8_t misc;
};

/**
 * @class PVTGeodetic
 * @brief Struct for the SBF block "PVTGeodetic"
 */
struct PVTGeodetic
{
    BlockHeader_t block_header;

    /* Time Header */
    uint32_t tow;
    uint16_t wnc;

    uint8_t mode;
    uint8_t error;
    double latitude;
    double longitude;
    double height;
    float undulation;
    float vn;
    float ve;
    float vu;
    float cog;
    double rx_clk_bias;
    float rx_clk_drift;
    uint8_t time_system;
    uint8_t datum;
    uint8_t nr_sv;
    uint8_t wa_corr_info;
    uint16_t reference_id;
    uint16_t mean_corr_age;
    uint32_t signal_info;
    uint8_t alert_flag;
    uint8_t nr_bases;
    uint16_t ppp_info;
    uint16_t latency;
    uint16_t h_accuracy;
    uint16_t v_accuracy;
    uint8_t misc;
};

/**
 * @class AttEuler
 * @brief Struct for the SBF block "AttEuler"
 */
struct AttEuler
{
    BlockHeader_t block_header;

    /* Time Header */
    uint32_t tow;
    uint16_t wnc;

    uint8_t nr_sv;
    uint8_t error;
    uint16_t mode;
    uint16_t reserved;
    float heading;
    float pitch;
    float roll;
    float pitch_dot;
    float roll_dot;
    float heading_dot;
};

/**
 * @class AttCovEuler
 * @brief Struct for the SBF block "AttCovEuler"
 */
struct AttCovEuler
{
    BlockHeader_t block_header;

    /* Time Header */
    uint32_t tow;
    uint16_t wnc;

    uint8_t reserved;
    uint8_t error;
    float cov_headhead;
    float cov_pitchpitch;
    float cov_rollroll;
    float cov_headpitch;
    float cov_headroll;
    float cov_pitchroll;
};

/**
 * @class ChannelStateInfo
 * @brief Struct for the SBF sub-block "ChannelStateInfo"
 */
typedef struct
{
    uint8_t antenna;
    uint8_t reserved;
    uint16_t tracking_status;
    uint16_t pvt_status;
    uint16_t pvt_info;
} ChannelStateInfo;

/**
 * @class ChannelSatInfo
 * @brief Struct for the SBF sub-block "ChannelSatInfo"
 */
typedef struct
{
    uint8_t sv_id;
    uint8_t freq_nr;
    std::vector<uint8_t> reserved1; // size = 2
    uint16_t az_rise_set;
    uint16_t health_status;
    int8_t elev;
    uint8_t n2;
    uint8_t channel;
    uint8_t reserved2;
    std::vector<ChannelStateInfo> stateInfo;
} ChannelSatInfo;

/**
 * @class ChannelStatus
 * @brief Struct for the SBF block "ChannelStatus"
 */
struct ChannelStatus
{
    BlockHeader_t block_header;

    /* Time Header */
    uint32_t tow;
    uint16_t wnc;

    uint8_t n;
    uint8_t sb1_size;
    uint8_t sb2_size;
    std::vector<uint8_t> reserved; // size = 3
    std::vector<ChannelSatInfo> satInfo;
};

/**
 * @class MeasEpochChannelType2
 * @brief Struct for the SBF sub-block "MeasEpochChannelType2"
 */
typedef struct
{
    uint8_t type;
    uint8_t lock_time;
    uint8_t cn0;
    uint8_t offsets_msb;
    int8_t carrier_msb;
    uint8_t obs_info;
    uint16_t code_offset_lsb;
    uint16_t carrier_lsb;
    uint16_t doppler_offset_lsb;
} MeasEpochChannelType2;

/**
 * @class MeasEpochChannelType1
 * @brief Struct for the SBF sub-block "MeasEpochChannelType1"
 */
typedef struct
{
    uint8_t rx_channel;
    uint8_t type;
    uint8_t sv_id;
    uint8_t misc;
    uint32_t code_lsb;
    int32_t doppler;
    uint16_t carrier_lsb;
    int8_t carrier_msb;
    uint8_t cn0;
    uint16_t lock_time;
    uint8_t obs_info;
    uint8_t n_type2;
    std::vector<MeasEpochChannelType2> type2;
} MeasEpochChannelType1;

/**
 * @class MeasEpoch
 * @brief Struct for the SBF block "MeasEpoch"
 */
struct MeasEpoch
{
    BlockHeader_t block_header;

    /* Time Header */
    uint32_t tow;
    uint16_t wnc;

    /* MeasEpoch Header */
    uint8_t n;
    uint8_t sb1_size;
    uint8_t sb2_size;

    uint8_t common_flags;
    uint8_t cum_clk_jumps;
    uint8_t reserved;
    std::vector<MeasEpochChannelType1> type1;
};

/**
 * @class DOP
 * @brief Struct for the SBF block "DOP"
 */
struct DOP
{
    BlockHeader_t block_header;

    /* Time Header */
    uint32_t tow;
    uint16_t wnc;

    uint8_t nr_sv;
    uint8_t reserved;
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
    BlockHeader_t block_header;

    /* Time Header */
    uint32_t tow;
    uint16_t wnc;

    std::vector<uint8_t> reserved;
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
};

/**
 * @class QualityInd
 * @brief Struct for the SBF block "QualityInd"
 */
struct QualityInd
{
    BlockHeader_t block_header;

    /* Time Header */
    uint32_t tow;
    uint16_t wnc;

    uint8_t n;
    uint8_t reserved;
    std::vector<uint16_t> indicators;
};

/**
 * @brief Struct for the SBF sub-block "AGCState"
 */
typedef struct
{
    uint8_t frontend_id;
    int8_t gain;
    uint8_t sample_var;
    uint8_t blanking_stat;
} AGCState_t;

/**
 * @class ReceiverStatus
 * @brief Struct for the SBF block "ReceiverStatus"
 */
struct ReceiverStatus
{
    BlockHeader_t block_header;

    /* Time Header */
    uint32_t tow;
    uint16_t wnc;

    uint8_t cpu_load;
    uint8_t ext_error;
    uint32_t up_time;
    uint32_t rx_status;
    uint32_t rx_error;
    uint8_t n;
    uint8_t sb_length;
    uint8_t cmd_count;
    uint8_t temperature;
    std::vector<AGCState_t> agc_state;
};

/**
 * @class PosCovCartesian
 * @brief Struct for the SBF block "PosCovCartesian"
 */
struct PosCovCartesian
{
    BlockHeader_t block_header;

    /* Time Header */
    uint32_t tow;
    uint16_t wnc;

    uint8_t mode;
    uint8_t error;
    float cov_xx;
    float cov_yy;
    float cov_zz;
    float cov_bb;
    float cov_xy;
    float cov_xz;
    float cov_xb;
    float cov_yz;
    float cov_yb;
    float cov_zb;
};

/**
 * @class PosCovGeodetic
 * @brief Struct for the SBF block "PosCovGeodetic"
 */
struct PosCovGeodetic
{
    BlockHeader_t block_header;

    /* Time Header */
    uint32_t tow;
    uint16_t wnc;

    uint8_t mode;
    uint8_t error;
    float cov_latlat;
    float cov_lonlon;
    float cov_hgthgt;
    float cov_bb;
    float cov_latlon;
    float cov_lathgt;
    float cov_latb;
    float cov_lonhgt;
    float cov_lonb;
    float cov_hb;
};

/**
 * @class VelCovCartesian
 * @brief Struct for the SBF block "VelCovCartesian"
 */
struct VelCovCartesian
{
    BlockHeader_t block_header;

    /* Time Header */
    uint32_t tow;
    uint16_t wnc;

    uint8_t mode;
    uint8_t error;
    float cov_vxvx;
    float cov_vyvy;
    float cov_vzvz;
    float cov_dtdt;
    float cov_vxvy;
    float cov_vxvz;
    float cov_vxdt;
    float cov_vyvz;
    float cov_vydt;
    float cov_vzdt;
};

/**
 * @class VelCovGeodetic
 * @brief Struct for the SBF block "VelCovGeodetic"
 */
struct VelCovGeodetic
{
    BlockHeader_t block_header;

    /* Time Header */
    uint32_t tow;
    uint16_t wnc;

    uint8_t mode;
    uint8_t error;
    float cov_vnvn;
    float cov_veve;
    float cov_vuvu;
    float cov_dtdt;
    float cov_vnve;
    float cov_vnvu;
    float cov_vndt;
    float cov_vevu;
    float cov_vedt;
    float cov_vudt;
};

// -----------------------------INSNavCart---------------------------------

typedef struct 
{
    float     x_std_dev;
    float     y_std_dev;
    float     z_std_dev;
} INSNavCartPosStdDev_1;

typedef struct
{
    float     xy_cov;
    float     xz_cov;
    float     yz_cov;
} INSNavCartPosCov_1;

typedef struct
{
    float     heading;
    float     pitch;
    float     roll;
} INSNavCartAtt_1;

typedef struct
{
    float     heading_std_dev;
    float     pitch_std_dev;
    float     roll_std_dev;
} INSNavCartAttStdDev_1;

typedef struct
{
    float     heading_pitch_cov;
    float     heading_roll_cov;
    float     pitch_roll_cov;
} INSNavCartAttCov_1;

typedef struct
{
    float     vx;
    float     vy;
    float     vz;
} INSNavCartVel_1;

typedef struct
{
    float     vx_std_dev;
    float     vy_std_dev;
    float     vz_std_dev;
} INSNavCartVelStdDev_1;

typedef struct
{
    float     vx_vy_cov;
    float     vx_vz_cov;
    float     vy_vz_cov;
} INSNavCartVelCov_1;

typedef union
{
  INSNavCartPosStdDev_1 PosStdDev;    
  INSNavCartPosCov_1 PosCov;       
  INSNavCartAtt_1 Att;          
  INSNavCartAttStdDev_1 AttStdDev;    
  INSNavCartAttCov_1 AttCov;       
  INSNavCartVel_1 Vel;          
  INSNavCartVelStdDev_1 VelStdDev;    
  INSNavCartVelCov_1 VelCov;       
} INSNavCartData_1;

typedef struct
{
    BlockHeader_t block_header;

    /* Time Header */
    uint32_t tow;
    uint16_t wnc;

    uint8_t  gnss_mode;
    uint8_t  error;
    uint16_t info;
    uint16_t gnss_age;
    double   x;
    double   y;
    double   z;
    uint16_t accuracy;
    uint16_t latency;
    uint8_t  datum;
    uint16_t sb_list;

    INSNavCartData_1 INSNavCartData[SBF_INSNAVCART_LENGTH_1];
} INSNavCart_1;

typedef INSNavCartPosStdDev_1 INSNavCartPosStdDev;

typedef INSNavCartPosCov_1 INSNavCartPosCov;

typedef INSNavCartAtt_1 INSNavCartAtt;

typedef INSNavCartAttStdDev_1 INSNavCartAttStdDev;

typedef INSNavCartAttCov_1 INSNavCartAttCov;

typedef INSNavCartVel_1 INSNavCartVel;

typedef INSNavCartVelStdDev_1 INSNavCartVelStdDev;

typedef INSNavCartVelCov_1 INSNavCartVelCov;

typedef INSNavCartData_1 INSNavCartData;

typedef INSNavCart_1 INSNavCart;

//-----------------------INSNavGeod---------------------------------
typedef struct
{
    float     latitude_std_dev;
    float     longitude_std_dev;
    float     height_std_dev;
} INSNavGeodPosStdDev_1;

typedef struct
{
    float     latitude_longitude_cov;
    float     latitude_height_cov;
    float     longitude_height_cov;
} INSNavGeodPosCov_1;

typedef struct
{
    float     heading;
    float     pitch;
    float     roll;     
} INSNavGeodAtt_1;

typedef struct
{
    float     heading_std_dev;
    float     pitch_std_dev;
    float     roll_std_dev;
} INSNavGeodAttStdDev_1;

typedef struct
{
    float     heading_pitch_cov;
    float     heading_roll_cov;
    float     pitch_roll_cov;
} INSNavGeodAttCov_1;

typedef struct
{
    float     ve;
    float     vn;
    float     vu;
} INSNavGeodVel_1;

typedef struct 
{
    float     ve_std_dev;
    float     vn_std_dev;
    float     vu_std_dev;
} INSNavGeodVelStdDev_1;

typedef struct
{
    float     ve_vn_cov;
    float     ve_vu_cov;
    float     vn_vu_cov;
} INSNavGeodVelCov_1;

typedef union
{
    INSNavGeodPosStdDev_1 PosStdDev;    
    INSNavGeodPosCov_1 PosCov;       
    INSNavGeodAtt_1 Att; 
    INSNavGeodAttStdDev_1 AttStdDev;
    INSNavGeodAttCov_1 AttCov;
    INSNavGeodVel_1 Vel;
    INSNavGeodVelStdDev_1 VelStdDev;
    INSNavGeodVelCov_1 VelCov;
} INSNavGeodData_1;

/**
 * @class INSNavGeod
 * @brief Struct for the SBF block "INSNavGeod"
 */
typedef struct
{
    BlockHeader_t block_header;

    /* Time Header */
    uint32_t tow;
    uint16_t wnc;

    uint8_t  gnss_mode;
    uint8_t  error;
    uint16_t info;
    uint16_t gnss_age;
    double   latitude;
    double   longitude;
    double   height;
    float    undulation;
    uint16_t accuracy;
    uint16_t latency;
    uint8_t  datum;
    uint16_t sb_list;

    INSNavGeodData_1   INSNavGeodData[SBF_INSNAVGEOD_LENGTH_1];     
} INSNavGeod_1;

typedef INSNavGeodPosStdDev_1 INSNavGeodPosStdDev;
typedef INSNavGeodPosCov_1 INSNavGeodPosCov;
typedef INSNavGeodAtt_1 INSNavGeodAtt;
typedef INSNavGeodAttStdDev_1 INSNavGeodAttStdDev;
typedef INSNavGeodAttCov_1 INSNavGeodAttCov;
typedef INSNavGeodVel_1 INSNavGeodVel;
typedef INSNavGeodVelStdDev_1 INSNavGeodVelStdDev;
typedef INSNavGeodVelCov_1 INSNavGeodVelCov;
typedef INSNavGeodData_1 INSNavGeodData;
typedef INSNavGeod_1 INSNavGeod;

/**
 * @class IMUSetup
 * @brief Struct for the SBF block "IMUSetup"
 */
struct IMUSetup
{
    BlockHeader_t block_header;

    /* Time Header */
    uint32_t tow;
    uint16_t wnc;

    uint8_t  serial_port;
    float ant_lever_arm_x;
    float ant_lever_arm_y;
    float ant_lever_arm_z;
    float theta_x;
    float theta_y;
    float theta_z;
};

/**
 * @class VelSensorSetup
 * @brief Struct for the SBF block "IMUSetup"
 */
struct VelSensorSetup
{
    BlockHeader_t block_header;

    /* Time Header */
    uint32_t tow;
    uint16_t wnc;

    uint8_t port;
    float   lever_arm_x;
    float   lever_arm_y;
    float   lever_arm_z;
};

typedef struct
{
    double  acceleration_x;
    double  acceleration_y;
    double  acceleration_z;
} ExtSensorMeasAcceleration_1;

typedef struct
{
    double  angular_rate_x;
    double  angular_rate_y;
    double  angular_rate_z;
} ExtSensorMeasAngularRate_1;

typedef struct
{
    float velocity_x;
    float velocity_y;
    float velocity_z;
    float std_dev_x;
    float std_dev_y;
    float std_dev_z;
} ExtSensorMeasVelocity_1;

typedef struct
{
    int16_t   sensor_temperature;
} ExtSensorMeasInfo_1;

typedef struct
{
    double zero_velocity_flag;
} ExtSensorMeasZeroVelocityFlag_1;

typedef union
{
  ExtSensorMeasAcceleration_1 Acceleration; 
  ExtSensorMeasAngularRate_1 AngularRate;  
  ExtSensorMeasVelocity_1   Velocity;
  ExtSensorMeasInfo_1 Info;         
  ExtSensorMeasZeroVelocityFlag_1 ZeroVelocityFlag;
} ExtSensorMeasData_1;

typedef struct
{
  uint8_t        Source;       
  uint8_t        SensorModel;  
  uint8_t        type;         
  uint8_t        ObsInfo;      
  ExtSensorMeasData_1 ExtSensorMeasData;
} ExtSensorMeasSet_1;

typedef struct
{
    BlockHeader_t block_header;

    /* Time Header */
    uint32_t tow;
    uint16_t wnc;        

    uint8_t n;            
    uint8_t sb_length;       
    ExtSensorMeasSet_1 ExtSensorMeas[SBF_EXTSENSORMEAS_1_0_EXTSENSORMEAS_LENGTH];
} ExtSensorMeas_1;

typedef ExtSensorMeasAcceleration_1 ExtSensorMeasAcceleration;
typedef ExtSensorMeasAngularRate_1 ExtSensorMeasAngularRate;
typedef ExtSensorMeasVelocity_1 ExtSensorMeasVelocity;
typedef ExtSensorMeasInfo_1 ExtSensorMeasInfo;
typedef ExtSensorMeasZeroVelocityFlag_1 ExtSensorMeasZeroVelocityFlag;
typedef ExtSensorMeasData_1 ExtSensorMeasData;
typedef ExtSensorMeasSet_1 ExtSensorMeasSet;
typedef ExtSensorMeas_1 ExtSensorMeas;

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
BlockHeader_t,
    (uint8_t, sync_1),
    (uint8_t, sync_2),
    (uint16_t, crc),
    (uint16_t, null),
    (uint16_t, length)
)

BOOST_FUSION_ADAPT_STRUCT(
PVTCartesian,
    (BlockHeader_t, block_header),
    (uint32_t, tow),
    (uint16_t, wnc),
    (uint8_t, mode),
    (uint8_t, error),
    (double, x),
    (double, y),
    (double, z),
    (float, undulation),
    (float, vx),
    (float, vy),
    (float, vz),
    (float, cog),
    (double, rx_clk_bias),
    (float, rx_clk_drift),
    (uint8_t, time_system),
    (uint8_t, datum),
    (uint8_t, nr_sv),
    (uint8_t, wa_corr_info),
    (uint16_t, reference_id),
    (uint16_t, mean_corr_age),
    (uint32_t, signal_info),
    (uint8_t, alert_flag),
    (uint8_t, nr_bases),
    (uint16_t, ppp_info),
    (uint16_t, latency),
    (uint16_t, h_accuracy),
    (uint16_t, v_accuracy),
    (uint8_t, misc)
)

BOOST_FUSION_ADAPT_STRUCT(
PVTGeodetic,
    (BlockHeader_t, block_header),
    (uint32_t, tow),
    (uint16_t, wnc),
    (uint8_t, mode),
    (uint8_t, error),
    (double, latitude),
    (double, longitude),
    (double, height),
    (float, undulation),
    (float, vn),
    (float, ve),
    (float, vu),
    (float, cog),
    (double, rx_clk_bias),
    (float, rx_clk_drift),
    (uint8_t, time_system),
    (uint8_t, datum),
    (uint8_t, nr_sv),
    (uint8_t, wa_corr_info),
    (uint16_t, reference_id),
    (uint16_t, mean_corr_age),
    (uint32_t, signal_info),
    (uint8_t, alert_flag),
    (uint8_t, nr_bases),
    (uint16_t, ppp_info),
    (uint16_t, latency),
    (uint16_t, h_accuracy),
    (uint16_t, v_accuracy),
    (uint8_t, misc)
)

BOOST_FUSION_ADAPT_STRUCT(
AttEuler,
    (BlockHeader_t, block_header),
    (uint32_t, tow),
    (uint16_t, wnc),
    (uint8_t, nr_sv),
    (uint8_t, error),
    (uint16_t, mode),
    (uint16_t, reserved),
    (float, heading),
    (float, pitch),
    (float, roll),
    (float, pitch_dot),
    (float, roll_dot),
    (float, heading_dot)
)

BOOST_FUSION_ADAPT_STRUCT(
AttCovEuler,
    (BlockHeader_t, block_header),
    (uint32_t, tow),
    (uint16_t, wnc),
    (uint8_t, reserved),
    (uint8_t, error),
    (float, cov_headhead),
    (float, cov_pitchpitch),
    (float, cov_rollroll),
    (float, cov_headpitch),
    (float, cov_headroll),
    (float, cov_pitchroll)
)

BOOST_FUSION_ADAPT_STRUCT(
ChannelStateInfo,
    (uint8_t, antenna),
    (uint8_t, reserved),
    (uint16_t, tracking_status),
    (uint16_t, pvt_status),
    (uint16_t, pvt_info)
)

BOOST_FUSION_ADAPT_STRUCT(
ChannelSatInfo,
    (uint8_t, sv_id),
    (uint8_t, freq_nr),
    (std::vector<uint8_t>, reserved1),
    (uint16_t, az_rise_set),
    (uint16_t, health_status),
    (int8_t, elev),
    (uint8_t, n2),
    (uint8_t, channel),
    (uint8_t, reserved2),
    (std::vector<ChannelStateInfo>, stateInfo)
)

BOOST_FUSION_ADAPT_STRUCT(
ChannelStatus,
    (BlockHeader_t, block_header),
    (uint32_t, tow),
    (uint16_t, wnc),
    (uint8_t, n),
    (uint8_t, sb1_size),
    (uint8_t, sb2_size),
    (std::vector<uint8_t>, reserved),
    (std::vector<ChannelSatInfo>, satInfo)
)

BOOST_FUSION_ADAPT_STRUCT(
MeasEpochChannelType2,
    (uint8_t, type),
    (uint8_t, lock_time),
    (uint8_t, cn0),
    (uint8_t, offsets_msb),
    (int8_t, carrier_msb),
    (uint8_t, obs_info),
    (uint16_t, code_offset_lsb),
    (uint16_t, carrier_lsb),
    (uint16_t, doppler_offset_lsb)
)

BOOST_FUSION_ADAPT_STRUCT(
MeasEpochChannelType1,
    (uint8_t, rx_channel),
    (uint8_t, type),
    (uint8_t, sv_id),
    (uint8_t, misc),
    (uint32_t, code_lsb),
    (int32_t, doppler),
    (uint16_t, carrier_lsb),
    (int8_t, carrier_msb),
    (uint8_t, cn0),
    (uint16_t, lock_time),
    (uint8_t, obs_info),
    (uint8_t, n_type2),
    (std::vector<MeasEpochChannelType2>, type2)
)

BOOST_FUSION_ADAPT_STRUCT(
MeasEpoch,
    (BlockHeader_t, block_header),
    (uint32_t, tow),
    (uint16_t, wnc),
    (uint8_t, n),
    (uint8_t, sb1_size),
    (uint8_t, sb2_size),
    (uint8_t, common_flags),
    (uint8_t, cum_clk_jumps),
    (uint8_t, reserved),
    (std::vector<MeasEpochChannelType1>, type1)
)

BOOST_FUSION_ADAPT_STRUCT(
DOP,
    (BlockHeader_t, block_header),
    (uint32_t, tow),
    (uint16_t, wnc),
    (uint8_t, nr_sv),
    (uint8_t, reserved),
    (uint16_t, pdop),
    (uint16_t, tdop),
    (uint16_t, hdop),
    (uint16_t, vdop),
    (float, hpl),
    (float, vpl)
)

BOOST_FUSION_ADAPT_STRUCT(
ReceiverSetup,
    (BlockHeader_t, block_header),
    (uint32_t, tow),
    (uint16_t, wnc),
    (std::vector<uint8_t>, reserved),
    (std::string, marker_name),
    (std::string, marker_number),
    (std::string, observer),
    (std::string, agency),
    (std::string, rx_serial_number),
    (std::string, rx_name),
    (std::string, rx_version),
    (std::string, ant_serial_nbr),
    (std::string, ant_type),
    (float, delta_h),
    (float, delta_e),
    (float, delta_n),
    (std::string, marker_type),
    (std::string, gnss_fw_version)
)

BOOST_FUSION_ADAPT_STRUCT(
QualityInd,
    (BlockHeader_t, block_header),
    (uint32_t, tow),
    (uint16_t, wnc),
    (uint8_t, n),
    (uint8_t, reserved),
    (std::vector<uint16_t>, indicators)
)

BOOST_FUSION_ADAPT_STRUCT(
AGCState_t,
    (uint8_t, frontend_id),
    (int8_t, gain),
    (uint8_t, sample_var),
    (uint8_t, blanking_stat)
)

BOOST_FUSION_ADAPT_STRUCT(
ReceiverStatus,
    (BlockHeader_t, block_header),
    (uint32_t, tow),
    (uint16_t, wnc),
    (uint8_t, cpu_load),
    (uint8_t, ext_error),
    (uint32_t, up_time),
    (uint32_t, rx_status),
    (uint32_t, rx_error),
    (uint8_t, n),
    (uint8_t, sb_length),
    (uint8_t, cmd_count),
    (uint8_t, temperature),
    (std::vector<AGCState_t>, agc_state)
)

namespace qi  = boost::spirit::qi;
namespace rep = boost::spirit::repository;
namespace phx = boost::phoenix;

/**
 * @struct BlockHeaderGrammar
 * @brief Spirit grammar for the SBF block "BlockHeader"
 */
template<typename Iterator>
struct BlockHeaderGrammar : qi::grammar<Iterator, BlockHeader_t(uint16_t, uint8_t&, uint16_t&)>
{
	BlockHeaderGrammar() : BlockHeaderGrammar::base_type(blockHeader)
	{
		using namespace qi::labels;
		
        blockHeader %= qi::byte_[_pass = (qi::_1 == 0x24)]
		            >> qi::byte_[_pass = (qi::_1 == 0x40)]
		            >> qi::little_word
		            >> qi::little_word[_pass = ((qi::_1 & 8191) == _r1), _r2 = qi::_1 >> 13] // revision is upper 3 bits
                    >> qi::little_word[_r3 = qi::_1]; // length
	}

	qi::rule<Iterator, BlockHeader_t(uint16_t, uint8_t&, uint16_t&)> blockHeader;
};

/**
 * @struct PVTCartesianGrammar
 * @brief Spirit grammar for the SBF block "PVTCartesian"
 */
template<typename Iterator>
struct PVTCartesianGrammar : qi::grammar<Iterator, PVTCartesian()>
{
	PVTCartesianGrammar() : PVTCartesianGrammar::base_type(pvtCartesian)
	{
        using namespace qi::labels;
		
		pvtCartesianLocal %= header(4006, _a, _b) // id, revision, length
		                  >> qi::little_dword
                          >> qi::little_word
                          >> qi::byte_
                          >> qi::byte_
                          >> qi::little_bin_double
                          >> qi::little_bin_double
                          >> qi::little_bin_double
                          >> qi::little_bin_float
                          >> qi::little_bin_float
                          >> qi::little_bin_float
                          >> qi::little_bin_float
                          >> qi::little_bin_float
                          >> qi::little_bin_double
                          >> qi::little_bin_float
                          >> qi::byte_
                          >> qi::byte_
                          >> qi::byte_
                          >> qi::byte_
                          >> qi::little_word
                          >> qi::little_word
                          >> qi::little_dword
                          >> qi::byte_
                          >> qi::byte_
                          >> qi::little_word
                          >> qi::little_word
                          >> qi::little_word
                          >> qi::little_word
                          >> qi::byte_
                          >> qi::repeat[qi::omit[qi::byte_]]; //skip padding

        pvtCartesian %= pvtCartesianLocal;
	}

    BlockHeaderGrammar<Iterator> header;

	qi::rule<Iterator, qi::locals<uint8_t, uint16_t>, PVTCartesian()> pvtCartesianLocal;
    qi::rule<Iterator, PVTCartesian()> pvtCartesian;
};

/**
 * @struct PVTGeodeticGrammar
 * @brief Spirit grammar for the SBF block "PVTGeodetic"
 */
template<typename Iterator>
struct PVTGeodeticGrammar : qi::grammar<Iterator, PVTGeodetic()>
{
	PVTGeodeticGrammar() : PVTGeodeticGrammar::base_type(pvtGeodetic)
	{
        using namespace qi::labels;

		pvtGeodeticLocal %= header(4007, _a, _b) // id, revision, length
		                 >> qi::little_dword
		                 >> qi::little_word
                         >> qi::byte_
                         >> qi::byte_
                         >> qi::little_bin_double
                         >> qi::little_bin_double
                         >> qi::little_bin_double
                         >> qi::little_bin_float
                         >> qi::little_bin_float
                         >> qi::little_bin_float
                         >> qi::little_bin_float
                         >> qi::little_bin_float
                         >> qi::little_bin_double
                         >> qi::little_bin_float
                         >> qi::byte_
                         >> qi::byte_
                         >> qi::byte_
                         >> qi::byte_
                         >> qi::little_word
                         >> qi::little_word
                         >> qi::little_dword
                         >> qi::byte_
                         >> qi::byte_
                         >> qi::little_word
                         >> qi::little_word
                         >> qi::little_word
                         >> qi::little_word
                         >> qi::byte_
                         >> qi::repeat[qi::omit[qi::byte_]]; //skip padding

        pvtGeodetic %= pvtGeodeticLocal;
	}

    BlockHeaderGrammar<Iterator> header;

	qi::rule<Iterator, qi::locals<uint8_t, uint16_t>, PVTGeodetic()> pvtGeodeticLocal;
    qi::rule<Iterator, PVTGeodetic()> pvtGeodetic;
};

/**
 * @struct AttEulerGrammar
 * @brief Spirit grammar for the SBF block "AttEuler"
 */
template<typename Iterator>
struct AttEulerGrammar : qi::grammar<Iterator, AttEuler()>
{
	AttEulerGrammar() : AttEulerGrammar::base_type(attEuler)
	{
        using namespace qi::labels;

		attEulerLocal %= header(5938, _a, _b) // id, revision, length
		              >> qi::little_dword
		              >> qi::little_word
                      >> qi::byte_
                      >> qi::byte_
                      >> qi::little_word
		              >> qi::little_word
                      >> qi::little_bin_float
                      >> qi::little_bin_float
                      >> qi::little_bin_float
                      >> qi::little_bin_float
                      >> qi::little_bin_float
                      >> qi::repeat[qi::omit[qi::byte_]]; //skip padding

        attEuler %= attEulerLocal;
	}

    BlockHeaderGrammar<Iterator> header;

	qi::rule<Iterator, qi::locals<uint8_t, uint16_t>, AttEuler()> attEulerLocal;
    qi::rule<Iterator, AttEuler()> attEuler;
};

/**
 * @struct AttCovEulerGrammar
 * @brief Spirit grammar for the SBF block "AttCovEuler"
 */
template<typename Iterator>
struct AttCovEulerGrammar : qi::grammar<Iterator, AttCovEuler()>
{
	AttCovEulerGrammar() : AttCovEulerGrammar::base_type(attCovEuler)
	{
        using namespace qi::labels;

		attCovEulerLocal %= header(5939, _a, _b) // id, revision, length
		                 >> qi::little_dword
		                 >> qi::little_word
                         >> qi::byte_
                         >> qi::byte_
                         >> qi::little_bin_float
                         >> qi::little_bin_float
                         >> qi::little_bin_float
                         >> qi::little_bin_float
                         >> qi::little_bin_float
                         >> qi::little_bin_float
                         >> qi::repeat[qi::omit[qi::byte_]]; //skip padding

        attCovEuler %= attCovEulerLocal;
	}

    BlockHeaderGrammar<Iterator> header;

	qi::rule<Iterator, qi::locals<uint8_t, uint16_t>, AttCovEuler()> attCovEulerLocal;
    qi::rule<Iterator, AttCovEuler()> attCovEuler;
};

/**
 * @struct ChannelStatusGrammar
 * @brief Spirit grammar for the SBF block "ChannelStatus"
 */
template<typename Iterator>
struct ChannelStatusGrammar : qi::grammar<Iterator, ChannelStatus()>
{
	ChannelStatusGrammar() : ChannelStatusGrammar::base_type(channelStatus)
	{
        using namespace qi::labels;

        channelStateInfo %= qi::byte_
                         >> qi::byte_
                         >> qi::little_word
                         >> qi::little_word
                         >> qi::little_word
                         >> rep::qi::advance(_r1 - 8); // skip padding: sb2_size - 8 bytes

		channelSatInfo %= qi::byte_
                       >> qi::byte_
                       >> qi::repeat(2)[qi::byte_]
                       >> qi::little_word
                       >> qi::little_word
                       >> qi::char_
                       >> qi::byte_[_pass = (qi::_1 <= MAXSB_CHANNELSTATEINFO), _a = qi::_1] // n2
                       >> qi::byte_
                       >> qi::byte_
                       >> rep::qi::advance(_r1 - 12) // skip padding: sb1_size - 12 bytes
                       >> qi::eps[phx::reserve(phx::at_c<9>(_val), _a)]
		               >> qi::repeat(_a)[channelStateInfo(_r2)]; // pass sb2_size

        channelStatusLocal %= header(4013, _a, _b) // id, revision, length
		                   >> qi::little_dword
		                   >> qi::little_word
                           >> qi::byte_[_pass = (qi::_1 <= MAXSB_CHANNELSATINFO), _c = qi::_1] // n
                           >> qi::byte_[_d = qi::_1] // sb1_size
                           >> qi::byte_[_e = qi::_1] // sb2_size
                           >> qi::repeat(3)[qi::byte_]
                           >> qi::eps[phx::reserve(phx::at_c<7>(_val), _c)]
                           >> qi::repeat(_c)[channelSatInfo(_d, _e)] // pass sb1_size and sb2_size
                           >> qi::repeat[qi::omit[qi::byte_]]; // skip padding

        channelStatus %= channelStatusLocal;
	}

    BlockHeaderGrammar<Iterator> header;

    qi::rule<Iterator, ChannelStateInfo(uint8_t)> channelStateInfo;
    qi::rule<Iterator, qi::locals<uint8_t>, ChannelSatInfo(uint8_t, uint8_t)> channelSatInfo;
	qi::rule<Iterator, qi::locals<uint8_t, uint16_t, uint8_t, uint8_t, uint8_t>, ChannelStatus()> channelStatusLocal;
	qi::rule<Iterator, ChannelStatus()> channelStatus;
};

/**
 * @struct MeasEpochGrammar
 * @brief Spirit grammar for the SBF block "MeasEpoch"
 */
template<typename Iterator>
struct MeasEpochGrammar : qi::grammar<Iterator, MeasEpoch()>
{
	MeasEpochGrammar() : MeasEpochGrammar::base_type(measEpoch)
	{
        using namespace qi::labels;

        measEpochChannelType2 %= qi::byte_
                              >> qi::byte_
                              >> qi::byte_
                              >> qi::byte_
                              >> qi::char_ 
                              >> qi::byte_
                              >> qi::little_word
                              >> qi::little_word
                              >> qi::little_word
                              >> rep::qi::advance(_r1 - 12); // skip padding: sb2_size - 12 bytes

		measEpochChannelType1 %= qi::byte_
                              >> qi::byte_
                              >> qi::byte_
                              >> qi::byte_
                              >> qi::little_dword
                              >> qi::little_dword
                              >> qi::little_word
                              >> qi::char_
                              >> qi::byte_
                              >> qi::little_word
                              >> qi::byte_
                              >> qi::byte_[_pass = (qi::_1 <= MAXSB_MEASEPOCH_T2), _a = qi::_1] // n2
                              >> rep::qi::advance(_r1 - 20) // skip padding: sb1_size - 20 bytes
                              >> qi::eps[phx::reserve(phx::at_c<12>(_val), _a)]
		                      >> qi::repeat(_a)[measEpochChannelType2(_r2)]; // pass sb2_size

        measEpochLocal %= header(4027, _a, _b) // id, revision, length
		               >> qi::little_dword
		               >> qi::little_word
                       >> qi::byte_[_pass = (qi::_1 <= MAXSB_MEASEPOCH_T1), _c = qi::_1] // n
                       >> qi::byte_[_d = qi::_1] // sb1_size
                       >> qi::byte_[_e = qi::_1] // sb2_size
                       >> qi::byte_
                       >> qi::byte_
                       >> qi::byte_
                       >> qi::eps[phx::reserve(phx::at_c<9>(_val), _c)]
                       >> qi::repeat(_c)[measEpochChannelType1(_d, _e)] // pass sb1_size and sb2_size
                       >> qi::repeat[qi::omit[qi::byte_]]; // skip padding

        measEpoch %= measEpochLocal;
	}

    BlockHeaderGrammar<Iterator> header;

    qi::rule<Iterator, MeasEpochChannelType2(uint8_t)> measEpochChannelType2;
    qi::rule<Iterator, qi::locals<uint8_t>, MeasEpochChannelType1(uint8_t, uint8_t)> measEpochChannelType1;
	qi::rule<Iterator, qi::locals<uint8_t, uint16_t, uint8_t, uint8_t, uint8_t>, MeasEpoch()> measEpochLocal;
	qi::rule<Iterator, MeasEpoch()> measEpoch;
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

		dopLocal %= header(4001, _a, _b) // id, revision, length
		         >> qi::little_dword
		         >> qi::little_word
                 >> qi::byte_
                 >> qi::byte_
                 >> qi::little_word
		         >> qi::little_word
                 >> qi::little_word
		         >> qi::little_word
                 >> qi::little_bin_float
                 >> qi::little_bin_float
                 >> qi::repeat[qi::omit[qi::byte_]]; //skip padding

        dop %= dopLocal;
	}

    BlockHeaderGrammar<Iterator> header;

	qi::rule<Iterator, qi::locals<uint8_t, uint16_t>, DOP()> dopLocal;
    qi::rule<Iterator, DOP()> dop;
};

/**
 * @struct ReceiverSetupGrammar
 * @brief Spirit grammar for the SBF block "ReceiverSetup"
 */
template<typename Iterator>
struct ReceiverSetupGrammar : qi::grammar<Iterator, ReceiverSetup()>
{
	ReceiverSetupGrammar() : ReceiverSetupGrammar::base_type(receiverSetup)
	{
        using namespace qi::labels;

		receiverSetupLocal %= header(5902, _a, _b) // id, revision, length
		                   >> qi::little_dword
		                   >> qi::little_word
                           >> qi::repeat(2)[qi::byte_]
                           >> qi::as_string[qi::repeat(60)[qi::char_]]
                           >> qi::as_string[qi::repeat(20)[qi::char_]]
                           >> qi::as_string[qi::repeat(20)[qi::char_]]
                           >> qi::as_string[qi::repeat(40)[qi::char_]]
                           >> qi::as_string[qi::repeat(20)[qi::char_]]
                           >> qi::as_string[qi::repeat(20)[qi::char_]]
                           >> qi::as_string[qi::repeat(20)[qi::char_]]
                           >> qi::as_string[qi::repeat(20)[qi::char_]]
                           >> qi::as_string[qi::repeat(20)[qi::char_]]
                           >> qi::little_bin_float
                           >> qi::little_bin_float
                           >> qi::little_bin_float
                           >> qi::as_string[qi::repeat(20)[qi::char_]]
                           >> qi::as_string[qi::repeat(40)[qi::char_]]
                           >> qi::repeat[qi::omit[qi::byte_]]; //skip padding

        receiverSetup %= receiverSetupLocal;
	}

    BlockHeaderGrammar<Iterator> header;

	qi::rule<Iterator, qi::locals<uint8_t, uint16_t>, ReceiverSetup()> receiverSetupLocal;
    qi::rule<Iterator, ReceiverSetup()> receiverSetup;
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

		qualityIndLocal %= header(4082, _a, _b) // id, revision, length
		                >> qi::little_dword
		                >> qi::little_word
                        >> qi::byte_[_c = qi::_1] // n
                        >> qi::byte_
                        >> qi::eps[phx::reserve(phx::at_c<5>(_val), _c)]
                        >> qi::repeat(_c)[qi::little_word]
		                >> qi::repeat[qi::omit[qi::byte_]]; //skip padding

        qualityInd %= qualityIndLocal;
	}

    BlockHeaderGrammar<Iterator> header;

	qi::rule<Iterator, qi::locals<uint8_t, uint16_t, uint8_t>, QualityInd()> qualityIndLocal;
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
                 >> rep::qi::advance(_r1 - 4); // skip padding: sb_length - 4 bytes

        receiverStatusLocal %= header(4014, _a, _b) // id, revision, length
		                    >> qi::little_dword
		                    >> qi::little_word
                            >> qi::byte_
                            >> qi::byte_
                            >> qi::little_qword
                            >> qi::little_qword
                            >> qi::little_qword
                            >> qi::byte_[_c = qi::_1] // n
                            >> qi::byte_[_d = qi::_1] // sb_length
                            >> qi::byte_
                            >> qi::byte_
                            >> qi::eps[phx::reserve(phx::at_c<12>(_val), _c)]
                            >> qi::repeat(_c)[agcState(_d)] // pass sb_length
                            >> qi::repeat[qi::omit[qi::byte_]]; // skip padding

        receiverStatus %= receiverStatusLocal;
	}

    BlockHeaderGrammar<Iterator> header;

    qi::rule<Iterator, qi::locals<uint8_t>, AGCState_t(uint8_t)> agcState;
	qi::rule<Iterator, qi::locals<uint8_t, uint16_t, uint8_t, uint8_t>, ReceiverStatus()> receiverStatusLocal;
	qi::rule<Iterator, ReceiverStatus()> receiverStatus;
};

#endif // SBFStructs_HPP