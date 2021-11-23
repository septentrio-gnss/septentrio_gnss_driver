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

#include <septentrio_gnss_driver/communication/rx_message.hpp>

/**
 * @file rx_message.cpp
 * @date 20/08/20
 * @brief Defines a class that reads messages handed over from the circular buffer
 */

septentrio_gnss_driver::PVTGeodeticPtr
io_comm_rx::RxMessage::PVTGeodeticCallback(PVTGeodetic& data)
{
    septentrio_gnss_driver::PVTGeodeticPtr msg =
        boost::make_shared<septentrio_gnss_driver::PVTGeodetic>();
    msg->block_header.sync_1 = data.block_header.sync_1;
    msg->block_header.sync_2 = data.block_header.sync_2;
    msg->block_header.crc = data.block_header.crc;
    msg->block_header.id = data.block_header.id;
    msg->block_header.length = data.block_header.length;
    msg->block_header.tow = data.tow;
    msg->block_header.wnc = data.wnc;
    msg->mode = data.mode;
    msg->error = data.error;
    msg->latitude = data.latitude;
    msg->longitude = data.longitude;
    msg->height = data.height;
    msg->undulation = data.undulation;
    msg->vn = data.vn;
    msg->ve = data.ve;
    msg->vu = data.vu;
    msg->cog = data.cog;
    msg->rx_clk_bias = data.rx_clk_bias;
    msg->rx_clk_drift = data.rx_clk_drift;
    msg->time_system = data.time_system;
    msg->datum = data.datum;
    msg->nr_sv = data.nr_sv;
    msg->wa_corr_info = data.wa_corr_info;
    msg->reference_id = data.reference_id;
    msg->mean_corr_age = data.mean_corr_age;
    msg->signal_info = data.signal_info;
    msg->alert_flag = data.alert_flag;
    msg->nr_bases = data.nr_bases;
    msg->ppp_info = data.ppp_info;
    msg->latency = data.latency;
    msg->h_accuracy = data.h_accuracy;
    msg->v_accuracy = data.v_accuracy;
    msg->misc = data.misc;
    return msg;
}

septentrio_gnss_driver::PVTCartesianPtr
io_comm_rx::RxMessage::PVTCartesianCallback(PVTCartesian& data)
{
    septentrio_gnss_driver::PVTCartesianPtr msg =
        boost::make_shared<septentrio_gnss_driver::PVTCartesian>();
    msg->block_header.sync_1 = data.block_header.sync_1;
    msg->block_header.sync_2 = data.block_header.sync_2;
    msg->block_header.crc = data.block_header.crc;
    msg->block_header.id = data.block_header.id;
    msg->block_header.length = data.block_header.length;
    msg->block_header.tow = data.tow;
    msg->block_header.wnc = data.wnc;
    msg->mode = data.mode;
    msg->error = data.error;
    msg->x = data.x;
    msg->y = data.y;
    msg->z = data.z;
    msg->undulation = data.undulation;
    msg->vx = data.vx;
    msg->vy = data.vy;
    msg->vz = data.vz;
    msg->cog = data.cog;
    msg->rx_clk_bias = data.rx_clk_bias;
    msg->rx_clk_drift = data.rx_clk_drift;
    msg->time_system = data.time_system;
    msg->datum = data.datum;
    msg->nr_sv = data.nr_sv;
    msg->wa_corr_info = data.wa_corr_info;
    msg->reference_id = data.reference_id;
    msg->mean_corr_age = data.mean_corr_age;
    msg->signal_info = data.signal_info;
    msg->alert_flag = data.alert_flag;
    msg->nr_bases = data.nr_bases;
    msg->ppp_info = data.ppp_info;
    msg->latency = data.latency;
    msg->h_accuracy = data.h_accuracy;
    msg->v_accuracy = data.v_accuracy;
    msg->misc = data.misc;
    return msg;
}

septentrio_gnss_driver::PosCovCartesianPtr
io_comm_rx::RxMessage::PosCovCartesianCallback(PosCovCartesian& data)
{
    septentrio_gnss_driver::PosCovCartesianPtr msg =
        boost::make_shared<septentrio_gnss_driver::PosCovCartesian>();
    msg->block_header.sync_1 = data.block_header.sync_1;
    msg->block_header.sync_2 = data.block_header.sync_2;
    msg->block_header.crc = data.block_header.crc;
    msg->block_header.id = data.block_header.id;
    msg->block_header.length = data.block_header.length;
    msg->block_header.tow = data.tow;
    msg->block_header.wnc = data.wnc;
    msg->mode = data.mode;
    msg->error = data.error;
    msg->cov_xx = data.cov_xx;
    msg->cov_yy = data.cov_yy;
    msg->cov_zz = data.cov_zz;
    msg->cov_bb = data.cov_bb;
    msg->cov_xy = data.cov_xy;
    msg->cov_xz = data.cov_xz;
    msg->cov_xb = data.cov_xb;
    msg->cov_yz = data.cov_yz;
    msg->cov_yb = data.cov_yb;
    msg->cov_zb = data.cov_zb;
    return msg;
}

septentrio_gnss_driver::PosCovGeodeticPtr
io_comm_rx::RxMessage::PosCovGeodeticCallback(PosCovGeodetic& data)
{
    septentrio_gnss_driver::PosCovGeodeticPtr msg =
        boost::make_shared<septentrio_gnss_driver::PosCovGeodetic>();
    msg->block_header.sync_1 = data.block_header.sync_1;
    msg->block_header.sync_2 = data.block_header.sync_2;
    msg->block_header.crc = data.block_header.crc;
    msg->block_header.id = data.block_header.id;
    msg->block_header.length = data.block_header.length;
    msg->block_header.tow = data.tow;
    msg->block_header.wnc = data.wnc;
    msg->mode = data.mode;
    msg->error = data.error;
    msg->cov_latlat = data.cov_latlat;
    msg->cov_lonlon = data.cov_lonlon;
    msg->cov_hgthgt = data.cov_hgthgt;
    msg->cov_bb = data.cov_bb;
    msg->cov_latlon = data.cov_latlon;
    msg->cov_lathgt = data.cov_lathgt;
    msg->cov_latb = data.cov_latb;
    msg->cov_lonhgt = data.cov_lonhgt;
    msg->cov_lonb = data.cov_lonb;
    msg->cov_hb = data.cov_hb;
    return msg;
}

septentrio_gnss_driver::VelCovGeodeticPtr
io_comm_rx::RxMessage::VelCovGeodeticCallback(VelCovGeodetic& data)
{
    septentrio_gnss_driver::VelCovGeodeticPtr msg =
        boost::make_shared<septentrio_gnss_driver::VelCovGeodetic>();
    msg->block_header.sync_1 = data.block_header.sync_1;
    msg->block_header.sync_2 = data.block_header.sync_2;
    msg->block_header.crc = data.block_header.crc;
    msg->block_header.id = data.block_header.id;
    msg->block_header.length = data.block_header.length;
    msg->block_header.tow = data.tow;
    msg->block_header.wnc = data.wnc;
    msg->mode = data.mode;
    msg->error = data.error;
    msg->cov_vnvn = data.cov_vnvn;
    msg->cov_veve = data.cov_veve;
    msg->cov_vuvu = data.cov_vuvu;
    msg->cov_dtdt = data.cov_dtdt;
    msg->cov_vnve = data.cov_vnve;
    msg->cov_vnvu = data.cov_vnvu;
    msg->cov_vndt = data.cov_vndt;
    msg->cov_vevu = data.cov_vevu;
    msg->cov_vedt = data.cov_vedt;
    msg->cov_vudt = data.cov_vudt;
    return msg;
}

septentrio_gnss_driver::AttEulerPtr
io_comm_rx::RxMessage::AttEulerCallback(AttEuler& data)
{
    septentrio_gnss_driver::AttEulerPtr msg =
        boost::make_shared<septentrio_gnss_driver::AttEuler>();
    msg->block_header.sync_1 = data.block_header.sync_1;
    msg->block_header.sync_2 = data.block_header.sync_2;
    msg->block_header.crc = data.block_header.crc;
    msg->block_header.id = data.block_header.id;
    msg->block_header.length = data.block_header.length;
    msg->block_header.tow = data.tow;
    msg->block_header.wnc = data.wnc;
    msg->nr_sv = data.nr_sv;
    msg->error = data.error;
    msg->mode = data.mode;
    msg->heading = data.heading;
    msg->pitch = data.pitch;
    msg->roll = data.roll;
    msg->pitch_dot = data.pitch_dot;
    msg->roll_dot = data.roll_dot;
    msg->heading_dot = data.heading_dot;
    return msg;
};

septentrio_gnss_driver::AttCovEulerPtr
io_comm_rx::RxMessage::AttCovEulerCallback(AttCovEuler& data)
{
    septentrio_gnss_driver::AttCovEulerPtr msg =
        boost::make_shared<septentrio_gnss_driver::AttCovEuler>();
    msg->block_header.sync_1 = data.block_header.sync_1;
    msg->block_header.sync_2 = data.block_header.sync_2;
    msg->block_header.crc = data.block_header.crc;
    msg->block_header.id = data.block_header.id;
    msg->block_header.length = data.block_header.length;
    msg->block_header.tow = data.tow;
    msg->block_header.wnc = data.wnc;
    msg->error = data.error;
    msg->cov_headhead = data.cov_headhead;
    msg->cov_pitchpitch = data.cov_pitchpitch;
    msg->cov_rollroll = data.cov_rollroll;
    msg->cov_headpitch = data.cov_headpitch;
    msg->cov_headroll = data.cov_headroll;
    msg->cov_pitchroll = data.cov_pitchroll;
    return msg;
};

septentrio_gnss_driver::INSNavCartPtr
io_comm_rx::RxMessage::INSNavCartCallback(INSNavCart& data)
{
    int SBI_dx = 0;
    septentrio_gnss_driver::INSNavCartPtr msg =
        boost::make_shared<septentrio_gnss_driver::INSNavCart>();
    
    msg->block_header.sync_1 = data.block_header.sync_1;
    msg->block_header.sync_2 = data.block_header.sync_2;
    msg->block_header.crc = data.block_header.crc;
    msg->block_header.id = data.block_header.id;
    msg->block_header.length = data.block_header.length;
    msg->block_header.tow = data.tow;
    msg->block_header.wnc = data.wnc;
    msg->gnss_mode = data.gnss_mode;
    msg->error = data.error;
    msg->info = data.info;
    msg->gnss_age = data.gnss_age;
    msg->x = data.x;
    msg->y = data.y;
    msg->z = data.z;
    msg->accuracy = data.accuracy;
    msg->latency = data.latency;
    msg->datum = data.datum;
    msg->sb_list = data.sb_list;

    // Reading sub-block from corresponding SBF block
    if((msg->sb_list & 1) !=0)
    {
        msg->x_std_dev = data.INSNavCartData[SBI_dx].PosStdDev.x_std_dev;
        msg->y_std_dev = data.INSNavCartData[SBI_dx].PosStdDev.y_std_dev;
        msg->z_std_dev = data.INSNavCartData[SBI_dx].PosStdDev.z_std_dev;
        SBI_dx++;
    }
    // if this sub block is not available then output DO_NOT_USE_VALUE
    else
    {
        msg->x_std_dev = DO_NOT_USE_VALUE;
        msg->y_std_dev = DO_NOT_USE_VALUE;
        msg->z_std_dev = DO_NOT_USE_VALUE;
    }

    if((msg->sb_list & 2) !=0)
    {
        msg->heading = data.INSNavCartData[SBI_dx].Att.heading;
        msg->pitch= data.INSNavCartData[SBI_dx].Att.pitch;
        msg->roll = data.INSNavCartData[SBI_dx].Att.roll;
        SBI_dx++;
    }
    else
    {
        msg->heading = DO_NOT_USE_VALUE;
        msg->pitch = DO_NOT_USE_VALUE;
        msg->roll = DO_NOT_USE_VALUE;
    }

    if((msg->sb_list & 4) !=0)
    {
        msg->heading_std_dev = data.INSNavCartData[SBI_dx].AttStdDev.heading_std_dev;
        msg->pitch_std_dev= data.INSNavCartData[SBI_dx].AttStdDev.pitch_std_dev;
        msg->roll_std_dev = data.INSNavCartData[SBI_dx].AttStdDev.roll_std_dev;
        SBI_dx++;
    }
    else
    {
        msg->heading_std_dev = DO_NOT_USE_VALUE;
        msg->pitch_std_dev = DO_NOT_USE_VALUE;
        msg->roll_std_dev = DO_NOT_USE_VALUE;
    }

    if((msg->sb_list & 8) !=0)
    {
        msg->vx = data.INSNavCartData[SBI_dx].Vel.vx;
        msg->vy = data.INSNavCartData[SBI_dx].Vel.vy;
        msg->vz = data.INSNavCartData[SBI_dx].Vel.vz;
        SBI_dx++;
    }
    else
    {
        msg->vx = DO_NOT_USE_VALUE;
        msg->vy = DO_NOT_USE_VALUE;
        msg->vz = DO_NOT_USE_VALUE;
    }

    if((msg->sb_list & 16) !=0)
    {
        msg->vx_std_dev = data.INSNavCartData[SBI_dx].VelStdDev.vx_std_dev;
        msg->vy_std_dev = data.INSNavCartData[SBI_dx].VelStdDev.vy_std_dev;
        msg->vz_std_dev = data.INSNavCartData[SBI_dx].VelStdDev.vz_std_dev;
        SBI_dx++;
    }
    else
    {
        msg->vx_std_dev = DO_NOT_USE_VALUE;
        msg->vy_std_dev = DO_NOT_USE_VALUE;
        msg->vz_std_dev = DO_NOT_USE_VALUE;
    }

    if((msg->sb_list & 32) !=0)
    {
        msg->xy_cov = data.INSNavCartData[SBI_dx].PosCov.xy_cov;
        msg->xz_cov = data.INSNavCartData[SBI_dx].PosCov.xz_cov;
        msg->yz_cov = data.INSNavCartData[SBI_dx].PosCov.yz_cov;
        SBI_dx++;
    }
    else
    {
        msg->xy_cov = DO_NOT_USE_VALUE;
        msg->xz_cov = DO_NOT_USE_VALUE;
        msg->yz_cov = DO_NOT_USE_VALUE;
    }

    if((msg->sb_list & 64) !=0)
    {
        msg->heading_pitch_cov = data.INSNavCartData[SBI_dx].AttCov.heading_pitch_cov;
        msg->heading_roll_cov = data.INSNavCartData[SBI_dx].AttCov.heading_roll_cov;
        msg->pitch_roll_cov = data.INSNavCartData[SBI_dx].AttCov.pitch_roll_cov;
        SBI_dx++;
    }
    else
    {
        msg->heading_pitch_cov = DO_NOT_USE_VALUE;
        msg->heading_roll_cov = DO_NOT_USE_VALUE;
        msg->pitch_roll_cov = DO_NOT_USE_VALUE;
    }

    if((msg->sb_list & 128) !=0)
    {
        msg->vx_vy_cov = data.INSNavCartData[SBI_dx].VelCov.vx_vy_cov;
        msg->vx_vz_cov = data.INSNavCartData[SBI_dx].VelCov.vx_vz_cov;
        msg->vy_vz_cov = data.INSNavCartData[SBI_dx].VelCov.vy_vz_cov;
        SBI_dx++;
    }
    else
    {
        msg->vx_vy_cov = DO_NOT_USE_VALUE;
        msg->vx_vz_cov = DO_NOT_USE_VALUE;
        msg->vy_vz_cov = DO_NOT_USE_VALUE;
    }
    return msg;
};

septentrio_gnss_driver::INSNavGeodPtr
io_comm_rx::RxMessage::INSNavGeodCallback(INSNavGeod& data)
{
    int SBIdx = 0;
    septentrio_gnss_driver::INSNavGeodPtr msg =
        boost::make_shared<septentrio_gnss_driver::INSNavGeod>();
    
    msg->block_header.sync_1 = data.block_header.sync_1;
    msg->block_header.sync_2 = data.block_header.sync_2;
    msg->block_header.crc = data.block_header.crc;
    msg->block_header.id = data.block_header.id;
    msg->block_header.length = data.block_header.length;
    msg->block_header.tow = data.tow;
    msg->block_header.wnc = data.wnc;
    msg->gnss_mode = data.gnss_mode;
    msg->error = data.error;
    msg->info = data.info;
    msg->gnss_age = data.gnss_age;
    msg->latitude = data.latitude;
    msg->longitude = data.longitude;
    msg->height = data.height;
    msg->undulation = data.undulation;
    msg->accuracy = data.accuracy;
    msg->latency = data.latency;
    msg->datum = data.datum;
    msg->sb_list = data.sb_list;

    // Reading sub-block from corresponding SBF block
    if((msg->sb_list & 1) !=0)
    {
        msg->latitude_std_dev = data.INSNavGeodData[SBIdx].PosStdDev.latitude_std_dev;
        msg->longitude_std_dev = data.INSNavGeodData[SBIdx].PosStdDev.longitude_std_dev;
        msg->height_std_dev = data.INSNavGeodData[SBIdx].PosStdDev.height_std_dev;
        SBIdx++;
    }
    // if this sub block is not available then output DO_NOT_USE_VALUE
    else
    {
        msg->latitude_std_dev = DO_NOT_USE_VALUE;
        msg->longitude_std_dev = DO_NOT_USE_VALUE;
        msg->height_std_dev = DO_NOT_USE_VALUE;
    }

    if((msg->sb_list & 2) !=0)
    {
        msg->heading = data.INSNavGeodData[SBIdx].Att.heading;
        msg->pitch = data.INSNavGeodData[SBIdx].Att.pitch;
        msg->roll = data.INSNavGeodData[SBIdx].Att.roll;
        SBIdx++;
    }
    else
    {
        msg->heading = DO_NOT_USE_VALUE;
        msg->pitch = DO_NOT_USE_VALUE;
        msg->roll = DO_NOT_USE_VALUE;
    }

    if((msg->sb_list & 4) !=0)
    {
        msg->heading_std_dev = data.INSNavGeodData[SBIdx].AttStdDev.heading_std_dev;
        msg->pitch_std_dev = data.INSNavGeodData[SBIdx].AttStdDev.pitch_std_dev;
        msg->roll_std_dev = data.INSNavGeodData[SBIdx].AttStdDev.roll_std_dev;
        SBIdx++;
    }
    else
    {
        msg->heading_std_dev = DO_NOT_USE_VALUE;
        msg->pitch_std_dev = DO_NOT_USE_VALUE;
        msg->roll_std_dev = DO_NOT_USE_VALUE;
    }

    if((msg->sb_list & 8) !=0)
    {
        msg->ve = data.INSNavGeodData[SBIdx].Vel.ve;
        msg->vn = data.INSNavGeodData[SBIdx].Vel.vn;
        msg->vu = data.INSNavGeodData[SBIdx].Vel.vu;
        SBIdx++;
    }
    else
    {
        msg->ve = DO_NOT_USE_VALUE;
        msg->vn = DO_NOT_USE_VALUE;
        msg->vu = DO_NOT_USE_VALUE;
    }

    if((msg->sb_list & 16) !=0)
    {
        msg->ve_std_dev = data.INSNavGeodData[SBIdx].VelStdDev.ve_std_dev;
        msg->vn_std_dev = data.INSNavGeodData[SBIdx].VelStdDev.vn_std_dev;
        msg->vu_std_dev = data.INSNavGeodData[SBIdx].VelStdDev.vu_std_dev;
        SBIdx++;
    }
    else
    {
        msg->ve_std_dev = DO_NOT_USE_VALUE;
        msg->vn_std_dev = DO_NOT_USE_VALUE;
        msg->vu_std_dev = DO_NOT_USE_VALUE;
    }

    if((msg->sb_list & 32) !=0)
    {
        msg->latitude_longitude_cov = data.INSNavGeodData[SBIdx].PosCov.latitude_longitude_cov;
        msg->latitude_height_cov = data.INSNavGeodData[SBIdx].PosCov.latitude_height_cov;
        msg->longitude_height_cov = data.INSNavGeodData[SBIdx].PosCov.longitude_height_cov;
        SBIdx++;
    }
    else
    {
        msg->latitude_longitude_cov = DO_NOT_USE_VALUE;
        msg->latitude_height_cov = DO_NOT_USE_VALUE;
        msg->longitude_height_cov = DO_NOT_USE_VALUE;
    }


    if((msg->sb_list & 64) !=0)
    {
        msg->heading_pitch_cov = data.INSNavGeodData[SBIdx].AttCov.heading_pitch_cov;
        msg->heading_roll_cov = data.INSNavGeodData[SBIdx].AttCov.heading_roll_cov;
        msg->pitch_roll_cov = data.INSNavGeodData[SBIdx].AttCov.pitch_roll_cov;
        SBIdx++;
    }
    else
    {
        msg->heading_pitch_cov = DO_NOT_USE_VALUE;
        msg->heading_roll_cov = DO_NOT_USE_VALUE;
        msg->pitch_roll_cov = DO_NOT_USE_VALUE;
    }

    if((msg->sb_list & 128) !=0)
    {
        msg->ve_vn_cov = data.INSNavGeodData[SBIdx].VelCov.ve_vn_cov;
        msg->ve_vu_cov = data.INSNavGeodData[SBIdx].VelCov.ve_vu_cov;
        msg->vn_vu_cov = data.INSNavGeodData[SBIdx].VelCov.vn_vu_cov;
        SBIdx++;
    }
    else
    {
        msg->ve_vn_cov = DO_NOT_USE_VALUE;
        msg->ve_vu_cov = DO_NOT_USE_VALUE;
        msg->vn_vu_cov = DO_NOT_USE_VALUE;
    }
    return msg;
};

septentrio_gnss_driver::IMUSetupPtr
io_comm_rx::RxMessage::IMUSetupCallback(IMUSetup& data)
{
    septentrio_gnss_driver::IMUSetupPtr msg = 
        boost::make_shared<septentrio_gnss_driver::IMUSetup>();
    msg->block_header.sync_1 = data.block_header.sync_1;
    msg->block_header.sync_2 = data.block_header.sync_2;
    msg->block_header.crc = data.block_header.crc;
    msg->block_header.id = data.block_header.id;
    msg->block_header.length = data.block_header.length;
    msg->block_header.tow = data.tow;
    msg->block_header.wnc = data.wnc;
    msg->serial_port = data.serial_port;
    msg->ant_lever_arm_X = data.ant_lever_arm_X;
    msg->ant_lever_arm_Y = data.ant_lever_arm_Y;
    msg->ant_lever_arm_Z = data.ant_lever_arm_Z;
    msg->theta_X = data.theta_X;
    msg->theta_Y = data.theta_Y;
    msg->theta_Z = data.theta_Z;
    return msg;
};

septentrio_gnss_driver::VelSensorSetupPtr
io_comm_rx::RxMessage::VelSensorSetupCallback(VelSensorSetup& data)
{
    septentrio_gnss_driver::VelSensorSetupPtr msg = 
        boost::make_shared<septentrio_gnss_driver::VelSensorSetup>();

    msg->block_header.sync_1 = data.block_header.sync_1;
    msg->block_header.sync_2 = data.block_header.sync_2;
    msg->block_header.crc = data.block_header.crc;
    msg->block_header.id = data.block_header.id;
    msg->block_header.length = data.block_header.length;
    msg->block_header.tow = data.tow;
    msg->block_header.wnc = data.wnc;
    msg->port = data.port;
    msg->lever_arm_X = data.lever_arm_X;
    msg->lever_arm_Y = data.lever_arm_Y;
    msg->lever_arm_Z = data.lever_arm_Z;
    return msg;
};

septentrio_gnss_driver::ExtEventINSNavGeodPtr
io_comm_rx::RxMessage::ExtEventINSNavGeodCallback(ExtEventINSNavGeod& data)
{
    int SBIdx = 0;
    septentrio_gnss_driver::ExtEventINSNavGeodPtr msg =
        boost::make_shared<septentrio_gnss_driver::ExtEventINSNavGeod>();
    
    msg->block_header.sync_1 = data.block_header.sync_1;
    msg->block_header.sync_2 = data.block_header.sync_2;
    msg->block_header.crc = data.block_header.crc;
    msg->block_header.id = data.block_header.id;
    msg->block_header.length = data.block_header.length;
    msg->block_header.tow = data.tow;
    msg->block_header.wnc = data.wnc;
    msg->gnss_mode = data.gnss_mode;
    msg->error = data.error;
    msg->info = data.info;
    msg->gnss_age = data.gnss_age;
    msg->latitude = data.latitude;
    msg->longitude = data.longitude;
    msg->height = data.height;
    msg->undulation = data.undulation;
    msg->accuracy = data.accuracy;
    msg->datum = data.datum;
    msg->sb_list = data.sb_list;

    // Reading sub-block from corresponding SBF block
    if((msg->sb_list & 1) !=0)
    {
        msg->latitude_std_dev = data.ExtEventINSNavGeodData[SBIdx].PosStdDev.latitude_std_dev;
        msg->longitude_std_dev = data.ExtEventINSNavGeodData[SBIdx].PosStdDev.longitude_std_dev;
        msg->height_std_dev = data.ExtEventINSNavGeodData[SBIdx].PosStdDev.height_std_dev;
        SBIdx++;
    }
    // if this sub block is not available then output DO_NOT_USE_VALUE
    else
    {
        msg->latitude_std_dev = DO_NOT_USE_VALUE;
        msg->longitude_std_dev = DO_NOT_USE_VALUE;
        msg->height_std_dev = DO_NOT_USE_VALUE;
    }

    if((msg->sb_list & 2) !=0)
    {
        msg->heading = data.ExtEventINSNavGeodData[SBIdx].Att.heading;
        msg->pitch = data.ExtEventINSNavGeodData[SBIdx].Att.pitch;
        msg->roll = data.ExtEventINSNavGeodData[SBIdx].Att.roll;
        SBIdx++;
    }
    else
    {
        msg->heading = DO_NOT_USE_VALUE;
        msg->pitch = DO_NOT_USE_VALUE;
        msg->roll = DO_NOT_USE_VALUE;
    }

    if((msg->sb_list & 4) !=0)
    {
        msg->heading_std_dev = data.ExtEventINSNavGeodData[SBIdx].AttStdDev.heading_std_dev;
        msg->pitch_std_dev = data.ExtEventINSNavGeodData[SBIdx].AttStdDev.pitch_std_dev;
        msg->roll_std_dev = data.ExtEventINSNavGeodData[SBIdx].AttStdDev.roll_std_dev;
        SBIdx++;
    }
    else
    {
        msg->heading_std_dev = DO_NOT_USE_VALUE;
        msg->pitch_std_dev = DO_NOT_USE_VALUE;
        msg->roll_std_dev = DO_NOT_USE_VALUE;
    }

    if((msg->sb_list & 8) !=0)
    {
        msg->ve = data.ExtEventINSNavGeodData[SBIdx].Vel.ve;
        msg->vn = data.ExtEventINSNavGeodData[SBIdx].Vel.vn;
        msg->vu = data.ExtEventINSNavGeodData[SBIdx].Vel.vu;
        SBIdx++;
    }
    else
    {
        msg->ve = DO_NOT_USE_VALUE;
        msg->vn = DO_NOT_USE_VALUE;
        msg->vu = DO_NOT_USE_VALUE;
    }

    if((msg->sb_list & 16) !=0)
    {
        msg->ve_std_dev = data.ExtEventINSNavGeodData[SBIdx].VelStdDev.ve_std_dev;
        msg->vn_std_dev = data.ExtEventINSNavGeodData[SBIdx].VelStdDev.vn_std_dev;
        msg->vu_std_dev = data.ExtEventINSNavGeodData[SBIdx].VelStdDev.vu_std_dev;
        SBIdx++;
    }
    else
    {
        msg->ve_std_dev = DO_NOT_USE_VALUE;
        msg->vn_std_dev = DO_NOT_USE_VALUE;
        msg->vu_std_dev = DO_NOT_USE_VALUE;
    }
    return msg;
};

septentrio_gnss_driver::ExtEventINSNavCartPtr
io_comm_rx::RxMessage::ExtEventINSNavCartCallback(ExtEventINSNavCart& data)
{
    int SBI_dx = 0;
    septentrio_gnss_driver::ExtEventINSNavCartPtr msg =
        boost::make_shared<septentrio_gnss_driver::ExtEventINSNavCart>();
    
    msg->block_header.sync_1 = data.block_header.sync_1;
    msg->block_header.sync_2 = data.block_header.sync_2;
    msg->block_header.crc = data.block_header.crc;
    msg->block_header.id = data.block_header.id;
    msg->block_header.length = data.block_header.length;
    msg->block_header.tow = data.tow;
    msg->block_header.wnc = data.wnc;
    msg->gnss_mode = data.gnss_mode;
    msg->error = data.error;
    msg->info = data.info;
    msg->gnss_age = data.gnss_age;
    msg->x = data.x;
    msg->y = data.y;
    msg->z = data.z;
    msg->accuracy = data.accuracy;
    msg->datum = data.datum;
    msg->sb_list = data.sb_list;

    // Reading sub-block from corresponding SBF block
    if((msg->sb_list & 1) !=0)
    {
        msg->x_std_dev = data.ExtEventINSNavCartData[SBI_dx].ExtEventPosStdDev.x_std_dev;
        msg->y_std_dev = data.ExtEventINSNavCartData[SBI_dx].ExtEventPosStdDev.y_std_dev;
        msg->z_std_dev = data.ExtEventINSNavCartData[SBI_dx].ExtEventPosStdDev.z_std_dev;
        SBI_dx++;
    }
    // if this sub block is not available then output DO_NOT_USE_VALUE
    else
    {
        msg->x_std_dev = DO_NOT_USE_VALUE;
        msg->y_std_dev = DO_NOT_USE_VALUE;
        msg->z_std_dev = DO_NOT_USE_VALUE;
    }

    if((msg->sb_list & 2) !=0)
    {
        msg->heading = data.ExtEventINSNavCartData[SBI_dx].ExtEventAtt.heading;
        msg->pitch= data.ExtEventINSNavCartData[SBI_dx].ExtEventAtt.pitch;
        msg->roll = data.ExtEventINSNavCartData[SBI_dx].ExtEventAtt.roll;
        SBI_dx++;
    }
    else
    {
        msg->heading = DO_NOT_USE_VALUE;
        msg->pitch = DO_NOT_USE_VALUE;
        msg->roll = DO_NOT_USE_VALUE;
    }

    if((msg->sb_list & 4) !=0)
    {
        msg->heading_std_dev = data.ExtEventINSNavCartData[SBI_dx].ExtEventAttStdDev.heading_std_dev;
        msg->pitch_std_dev= data.ExtEventINSNavCartData[SBI_dx].ExtEventAttStdDev.pitch_std_dev;
        msg->roll_std_dev = data.ExtEventINSNavCartData[SBI_dx].ExtEventAttStdDev.roll_std_dev;
        SBI_dx++;
    }
    else
    {
        msg->heading_std_dev = DO_NOT_USE_VALUE;
        msg->pitch_std_dev = DO_NOT_USE_VALUE;
        msg->roll_std_dev = DO_NOT_USE_VALUE;
    }

    if((msg->sb_list & 8) !=0)
    {
        msg->vx = data.ExtEventINSNavCartData[SBI_dx].ExtEventVel.vx;
        msg->vy = data.ExtEventINSNavCartData[SBI_dx].ExtEventVel.vy;
        msg->vz = data.ExtEventINSNavCartData[SBI_dx].ExtEventVel.vz;
        SBI_dx++;
    }
    else
    {
        msg->vx = DO_NOT_USE_VALUE;
        msg->vy = DO_NOT_USE_VALUE;
        msg->vz = DO_NOT_USE_VALUE;
    }

    if((msg->sb_list & 16) !=0)
    {
        msg->vx_std_dev = data.ExtEventINSNavCartData[SBI_dx].ExtEventVelStdDev.vx_std_dev;
        msg->vy_std_dev = data.ExtEventINSNavCartData[SBI_dx].ExtEventVelStdDev.vy_std_dev;
        msg->vz_std_dev = data.ExtEventINSNavCartData[SBI_dx].ExtEventVelStdDev.vz_std_dev;
        SBI_dx++;
    }
    else
    {
        msg->vx_std_dev = DO_NOT_USE_VALUE;
        msg->vy_std_dev = DO_NOT_USE_VALUE;
        msg->vz_std_dev = DO_NOT_USE_VALUE;
    }
    return msg;
};

septentrio_gnss_driver::ExtSensorMeasPtr
io_comm_rx::RxMessage::ExtSensorMeasCallback(ExtSensorMeas& data)
{
    int i =0;
    septentrio_gnss_driver::ExtSensorMeasPtr msg=
        boost::make_shared<septentrio_gnss_driver::ExtSensorMeas>();

    msg->block_header.sync_1 = data.block_header.sync_1;
    msg->block_header.sync_2 = data.block_header.sync_2;
    msg->block_header.crc = data.block_header.crc;
    msg->block_header.id = data.block_header.id;
    msg->block_header.length = data.block_header.length;
    msg->block_header.tow = data.tow;
    msg->block_header.wnc = data.wnc;
    msg->n = data.n;
    msg->sb_length = data.sb_length;
    
    for (i=0; i<msg->n;i++)
    {
        msg->source = data.ExtSensorMeas[i].Source;
        msg->sensor_model = data.ExtSensorMeas[i].SensorModel;
        msg->type = data.ExtSensorMeas[i].type;
        msg->obs_info = data.ExtSensorMeas[i].ObsInfo;

        msg->acceleration_X = data.ExtSensorMeas[i].ExtSensorMeasData.Acceleration.acceleration_X;
        msg->acceleration_Y = data.ExtSensorMeas[i].ExtSensorMeasData.Acceleration.acceleration_Y;
        msg->acceleration_Z = data.ExtSensorMeas[i].ExtSensorMeasData.Acceleration.acceleration_Z;

        msg->angular_rate_X = data.ExtSensorMeas[i].ExtSensorMeasData.AngularRate.angular_rate_X;
        msg->angular_rate_Y = data.ExtSensorMeas[i].ExtSensorMeasData.AngularRate.angular_rate_Y;
        msg->angular_rate_Z = data.ExtSensorMeas[i].ExtSensorMeasData.AngularRate.angular_rate_Z;

        msg->velocity_X = data.ExtSensorMeas[i].ExtSensorMeasData.Velocity.velocity_X;
        msg->velocity_Y = data.ExtSensorMeas[i].ExtSensorMeasData.Velocity.velocity_Y;
        msg->velocity_Z = data.ExtSensorMeas[i].ExtSensorMeasData.Velocity.velocity_Z;
        msg->std_dev_X = data.ExtSensorMeas[i].ExtSensorMeasData.Velocity.std_dev_X;
        msg->std_dev_Y = data.ExtSensorMeas[i].ExtSensorMeasData.Velocity.std_dev_Y;
        msg->std_dev_Z = data.ExtSensorMeas[i].ExtSensorMeasData.Velocity.std_dev_Z;

        msg->sensor_temperature = data.ExtSensorMeas[i].ExtSensorMeasData.Info.sensor_temperature;

        msg->zero_velocity_flag = data.ExtSensorMeas[i].ExtSensorMeasData.ZeroVelocityFlag.zero_velocity_flag;
    }
    return msg;
};

/**
 * The position_covariance array is populated in row-major order, where the basis of
 * the correspond matrix is (E, N, U, Roll, Pitch, Heading). Important: The Euler
 * angles (Roll, Pitch, Heading) are with respect to a vehicle-fixed (e.g. for
 * mosaic-x5 in moving base mode via the command setAntennaLocation, ...) !local! NED
 * frame. Thus the orientation is !not! given with respect to the same frame as the
 * position is given in. The cross-covariances are hence (apart from the fact that
 * e.g. mosaic receivers do not calculate these quantities) set to zero. The position
 * and the partial (with 2 antennas) or full (for INS receivers) orientation have
 * covariance matrices available e.g. in the PosCovGeodetic or AttCovEuler blocks,
 * yet those are separate computations.
 */
geometry_msgs::PoseWithCovarianceStampedPtr
io_comm_rx::RxMessage::PoseWithCovarianceStampedCallback()
{
	geometry_msgs::PoseWithCovarianceStampedPtr msg =
            boost::make_shared<geometry_msgs::PoseWithCovarianceStamped>();
    if (septentrio_receiver_type_ == "gnss")
    {
        // Filling in the pose data
        msg->pose.pose.orientation = parsing_utilities::convertEulerToQuaternion(
            static_cast<double>(last_atteuler_.heading),
            static_cast<double>(last_atteuler_.pitch),
            static_cast<double>(last_atteuler_.roll));
        msg->pose.pose.position.x = static_cast<double>(last_pvtgeodetic_.longitude) *
                                    360 / (2 * boost::math::constants::pi<double>());
        msg->pose.pose.position.y = static_cast<double>(last_pvtgeodetic_.latitude) *
                                    360 / (2 * boost::math::constants::pi<double>());
        msg->pose.pose.position.z = static_cast<double>(last_pvtgeodetic_.height);
        // Filling in the covariance data in row-major order
        msg->pose.covariance[0] = static_cast<double>(last_poscovgeodetic_.cov_lonlon);
        msg->pose.covariance[1] = static_cast<double>(last_poscovgeodetic_.cov_latlon);
        msg->pose.covariance[2] = static_cast<double>(last_poscovgeodetic_.cov_lonhgt);
        msg->pose.covariance[3] = 0;
        msg->pose.covariance[4] = 0;
        msg->pose.covariance[5] = 0;
        msg->pose.covariance[6] = static_cast<double>(last_poscovgeodetic_.cov_latlon);
        msg->pose.covariance[7] = static_cast<double>(last_poscovgeodetic_.cov_latlat);
        msg->pose.covariance[8] = static_cast<double>(last_poscovgeodetic_.cov_lathgt);
        msg->pose.covariance[9] = 0;
        msg->pose.covariance[10] = 0;
        msg->pose.covariance[11] = 0;
        msg->pose.covariance[12] = static_cast<double>(last_poscovgeodetic_.cov_lonhgt);
        msg->pose.covariance[13] = static_cast<double>(last_poscovgeodetic_.cov_lathgt);
        msg->pose.covariance[14] = static_cast<double>(last_poscovgeodetic_.cov_hgthgt);
        msg->pose.covariance[15] = 0;
        msg->pose.covariance[16] = 0;
        msg->pose.covariance[17] = 0;
        msg->pose.covariance[18] = 0;
        msg->pose.covariance[19] = 0;
        msg->pose.covariance[20] = 0;
        msg->pose.covariance[21] = static_cast<double>(last_attcoveuler_.cov_rollroll);
        msg->pose.covariance[22] = static_cast<double>(last_attcoveuler_.cov_pitchroll);
        msg->pose.covariance[23] = static_cast<double>(last_attcoveuler_.cov_headroll);
        msg->pose.covariance[24] = 0;
        msg->pose.covariance[25] = 0;
        msg->pose.covariance[26] = 0;
        msg->pose.covariance[27] = static_cast<double>(last_attcoveuler_.cov_pitchroll);
        msg->pose.covariance[28] = static_cast<double>(last_attcoveuler_.cov_pitchpitch);
        msg->pose.covariance[29] = static_cast<double>(last_attcoveuler_.cov_headpitch);
        msg->pose.covariance[30] = 0;
        msg->pose.covariance[31] = 0;
        msg->pose.covariance[32] = 0;
        msg->pose.covariance[33] = static_cast<double>(last_attcoveuler_.cov_headroll);
        msg->pose.covariance[34] = static_cast<double>(last_attcoveuler_.cov_pitchroll);
        msg->pose.covariance[35] = static_cast<double>(last_attcoveuler_.cov_headhead);

	}
    if (septentrio_receiver_type_ == "ins")
    {
        // Filling in the pose data
		int SBIdx = 0;
        if ((last_insnavgeod_.sb_list & 2) !=0)
        {
            msg->pose.pose.orientation = parsing_utilities::convertEulerToQuaternion(
            static_cast<double>(last_insnavgeod_.INSNavGeodData[SBIdx].Att.heading),
            static_cast<double>(last_insnavgeod_.INSNavGeodData[SBIdx].Att.pitch),
            static_cast<double>(last_insnavgeod_.INSNavGeodData[SBIdx].Att.roll));
        }
        msg->pose.pose.position.x = static_cast<double>(last_insnavgeod_.longitude) *
                                    360 / (2 * boost::math::constants::pi<double>());
        msg->pose.pose.position.y = static_cast<double>(last_insnavgeod_.latitude) *
                                    360 / (2 * boost::math::constants::pi<double>());
        msg->pose.pose.position.z = static_cast<double>(last_insnavgeod_.height);
        // Filling in the covariance data in row-major order
        if((last_insnavgeod_.sb_list & 1) !=0)
        {
            msg->pose.covariance[0] = std::pow(static_cast<double>(last_insnavgeod_.
                                            INSNavGeodData[SBIdx].PosStdDev.longitude_std_dev),2);
            //SBIdx++;
        }
        if((last_insnavgeod_.sb_list & 32) !=0)
        {
            msg->pose.covariance[1] = static_cast<double>(last_insnavgeod_.
                                            INSNavGeodData[SBIdx].PosCov.latitude_longitude_cov);
            msg->pose.covariance[2] = static_cast<double>(last_insnavgeod_.
                                            INSNavGeodData[SBIdx].PosCov.longitude_height_cov);
        }
        msg->pose.covariance[3] = 0;
        msg->pose.covariance[4] = 0;
        msg->pose.covariance[5] = 0;
        if((last_insnavgeod_.sb_list & 32) !=0)
        {
            msg->pose.covariance[6] = static_cast<double>(last_insnavgeod_.
                                            INSNavGeodData[SBIdx].PosCov.latitude_longitude_cov);
        }
        if((last_insnavgeod_.sb_list & 1) !=0)
        {
            msg->pose.covariance[7] = std::pow(static_cast<double>(last_insnavgeod_.
                                            INSNavGeodData[SBIdx].PosStdDev.latitude_std_dev),2);
            //SBIdx++;
        }
        if((last_insnavgeod_.sb_list & 32) !=0)
        {
            msg->pose.covariance[8] = static_cast<double>(last_insnavgeod_.
                                            INSNavGeodData[SBIdx].PosCov.latitude_height_cov);
        }
        msg->pose.covariance[9] = 0;
        msg->pose.covariance[10] = 0;
        msg->pose.covariance[11] = 0;
        if((last_insnavgeod_.sb_list & 32) !=0)
        {
            msg->pose.covariance[12] = static_cast<double>(last_insnavgeod_.
                                            INSNavGeodData[SBIdx].PosCov.longitude_height_cov);
            msg->pose.covariance[13] = static_cast<double>(last_insnavgeod_.
                                            INSNavGeodData[SBIdx].PosCov.latitude_height_cov);
        }
        if((last_insnavgeod_.sb_list & 1) !=0)
        {
            msg->pose.covariance[14] = std::pow(static_cast<double>(last_insnavgeod_.
                                            INSNavGeodData[SBIdx].PosStdDev.height_std_dev),2);
            //SBIdx++;
        }
        msg->pose.covariance[15] = 0;
        msg->pose.covariance[16] = 0;
        msg->pose.covariance[17] = 0;
        msg->pose.covariance[18] = 0;
        msg->pose.covariance[19] = 0;
        msg->pose.covariance[20] = 0;
        if ((last_insnavgeod_.sb_list & 2) !=0)
        {
            msg->pose.covariance[21] = std::pow(static_cast<double>(last_insnavgeod_.
                                            INSNavGeodData[SBIdx].Att.roll),2);
            msg->pose.covariance[22] = static_cast<double>(last_insnavgeod_.
                                            INSNavGeodData[SBIdx].AttCov.pitch_roll_cov);
            msg->pose.covariance[23] = static_cast<double>(last_insnavgeod_.
                                            INSNavGeodData[SBIdx].AttCov.heading_roll_cov);
        }
        msg->pose.covariance[24] = 0;
        msg->pose.covariance[25] = 0;
        msg->pose.covariance[26] = 0;
        if ((last_insnavgeod_.sb_list & 2) !=0)
        {
            msg->pose.covariance[27] = static_cast<double>(last_insnavgeod_.
                                            INSNavGeodData[SBIdx].AttCov.pitch_roll_cov);
            msg->pose.covariance[28] = std::pow(static_cast<double>(last_insnavgeod_.
                                            INSNavGeodData[SBIdx].Att.pitch),2);
            msg->pose.covariance[29] = static_cast<double>(last_insnavgeod_.
                                            INSNavGeodData[SBIdx].AttCov.heading_pitch_cov);
        }
        msg->pose.covariance[30] = 0;
        msg->pose.covariance[31] = 0;
        msg->pose.covariance[32] = 0;
        if ((last_insnavgeod_.sb_list & 2) !=0)
        {
            msg->pose.covariance[33] = static_cast<double>(last_insnavgeod_.
                                            INSNavGeodData[SBIdx].AttCov.heading_roll_cov);
            msg->pose.covariance[34] = static_cast<double>(last_insnavgeod_.
                                            INSNavGeodData[SBIdx].AttCov.pitch_roll_cov);
            msg->pose.covariance[28] = std::pow(static_cast<double>(last_insnavgeod_.
                                            INSNavGeodData[SBIdx].Att.heading),2);
        }
    }
	return msg;
};

diagnostic_msgs::DiagnosticArrayPtr io_comm_rx::RxMessage::DiagnosticArrayCallback()
{
    diagnostic_msgs::DiagnosticArrayPtr msg =
        boost::make_shared<diagnostic_msgs::DiagnosticArray>();
	std::string serialnumber(last_receiversetup_.rx_serial_number);
	diagnostic_msgs::DiagnosticStatusPtr gnss_status =
		boost::make_shared<diagnostic_msgs::DiagnosticStatus>();
	// Constructing the "level of operation" field
	uint16_t indicators_type_mask = static_cast<uint16_t>(255);
	uint16_t indicators_value_mask = static_cast<uint16_t>(3840);
	uint16_t qualityind_pos;
	for (uint16_t i = static_cast<uint16_t>(0); i != last_qualityind_.n; ++i)
	{
		if ((last_qualityind_.indicators[i] & indicators_type_mask) ==
			static_cast<uint16_t>(0))
		{
			qualityind_pos = i;
			if (((last_qualityind_.indicators[i] & indicators_value_mask) >> 8) ==
				static_cast<uint16_t>(0))
			{
				gnss_status->level = diagnostic_msgs::DiagnosticStatus::STALE;
			} else if (((last_qualityind_.indicators[i] & indicators_value_mask) >>
						8) == static_cast<uint16_t>(1) ||
					((last_qualityind_.indicators[i] & indicators_value_mask) >>
						8) == static_cast<uint16_t>(2))
			{
				gnss_status->level = diagnostic_msgs::DiagnosticStatus::WARN;
			} else
			{
				gnss_status->level = diagnostic_msgs::DiagnosticStatus::OK;
			}
			break;
		}
	}
	// If the ReceiverStatus's RxError field is not 0, then at least one error has
	// been detected.
	if (last_receiverstatus_.rx_error != static_cast<uint32_t>(0))
	{
		gnss_status->level = diagnostic_msgs::DiagnosticStatus::ERROR;
	}
	// Creating an array of values associated with the GNSS status
	gnss_status->values.resize(static_cast<uint16_t>(last_qualityind_.n - 1));
	for (uint16_t i = static_cast<uint16_t>(0);
		i != static_cast<uint16_t>(last_qualityind_.n); ++i)
	{
		if (i == qualityind_pos)
		{
			continue;
		}
		if ((last_qualityind_.indicators[i] & indicators_type_mask) ==
			static_cast<uint16_t>(1))
		{
			gnss_status->values[i].key = "GNSS Signals, Main Antenna";
			gnss_status->values[i].value = std::to_string(
				(last_qualityind_.indicators[i] & indicators_value_mask) >> 8);
		} else if ((last_qualityind_.indicators[i] & indicators_type_mask) ==
				static_cast<uint16_t>(2))
		{
			gnss_status->values[i].key = "GNSS Signals, Aux1 Antenna";
			gnss_status->values[i].value = std::to_string(
				(last_qualityind_.indicators[i] & indicators_value_mask) >> 8);
		} else if ((last_qualityind_.indicators[i] & indicators_type_mask) ==
				static_cast<uint16_t>(11))
		{
			gnss_status->values[i].key = "RF Power, Main Antenna";
			gnss_status->values[i].value = std::to_string(
				(last_qualityind_.indicators[i] & indicators_value_mask) >> 8);
		} else if ((last_qualityind_.indicators[i] & indicators_type_mask) ==
				static_cast<uint16_t>(12))
		{
			gnss_status->values[i].key = "RF Power, Aux1 Antenna";
			gnss_status->values[i].value = std::to_string(
				(last_qualityind_.indicators[i] & indicators_value_mask) >> 8);
		} else if ((last_qualityind_.indicators[i] & indicators_type_mask) ==
				static_cast<uint16_t>(21))
		{
			gnss_status->values[i].key = "CPU Headroom";
			gnss_status->values[i].value = std::to_string(
				(last_qualityind_.indicators[i] & indicators_value_mask) >> 8);
		} else if ((last_qualityind_.indicators[i] & indicators_type_mask) ==
				static_cast<uint16_t>(25))
		{
			gnss_status->values[i].key = "OCXO Stability";
			gnss_status->values[i].value = std::to_string(
				(last_qualityind_.indicators[i] & indicators_value_mask) >> 8);
		} else if ((last_qualityind_.indicators[i] & indicators_type_mask) ==
				static_cast<uint16_t>(30))
		{
			gnss_status->values[i].key = "Base Station Measurements";
			gnss_status->values[i].value = std::to_string(
				(last_qualityind_.indicators[i] & indicators_value_mask) >> 8);
		} else
		{
			assert((last_qualityind_.indicators[i] & indicators_type_mask) ==
				static_cast<uint16_t>(31));
			gnss_status->values[i].key = "RTK Post-Processing";
			gnss_status->values[i].value = std::to_string(
				(last_qualityind_.indicators[i] & indicators_value_mask) >> 8);
		}
    }
	gnss_status->hardware_id = serialnumber;
	gnss_status->name = "gnss";
	gnss_status->message =
		"Quality Indicators (from 0 for low quality to 10 for high quality, 15 if unknown)";
	msg->status.push_back(*gnss_status);
	return msg;
};

/**
 * The position_covariance array is populated in row-major order, where the basis of
 * the corresponding matrix is ENU (so Cov_lonlon is in location 11 of the matrix).
 * The B2b signal type of BeiDou is not checked for usage, since the SignalInfo field
 * of the PVTGeodetic block does not disclose it. For that, one would need to go to
 * the ObsInfo field of the MeasEpochChannelType1 sub-block.
 */
sensor_msgs::NavSatFixPtr io_comm_rx::RxMessage::NavSatFixCallback()
{
    sensor_msgs::NavSatFixPtr msg = boost::make_shared<sensor_msgs::NavSatFix>();
	uint16_t mask = 15; // We extract the first four bits using this mask.
    if (septentrio_receiver_type_ == "gnss")
    {
        uint16_t type_of_pvt = ((uint16_t)(last_pvtgeodetic_.mode)) & mask;
        switch (type_of_pvt_map[type_of_pvt])
        {
        case evNoPVT:
        {
            msg->status.status = sensor_msgs::NavSatStatus::STATUS_NO_FIX;
            break;
        }
        case evStandAlone:
        case evFixed:
        {
            msg->status.status = sensor_msgs::NavSatStatus::STATUS_FIX;
            break;
        }
        case evDGPS:
        case evRTKFixed:
        case evRTKFloat:
        case evMovingBaseRTKFixed:
        case evMovingBaseRTKFloat:
        case evPPP:
        {
            msg->status.status = sensor_msgs::NavSatStatus::STATUS_GBAS_FIX;
            break;
        }
        case evSBAS:
        {
            msg->status.status = sensor_msgs::NavSatStatus::STATUS_SBAS_FIX;
            break;
        }
        default:
        {
            throw std::runtime_error(
                "PVTGeodetic's Mode field contains an invalid type of PVT solution.");
        }
        }
        bool gps_in_pvt = false;
        bool glo_in_pvt = false;
        bool com_in_pvt = false;
        bool gal_in_pvt = false;
        uint32_t mask_2 = 1;
        for (int bit = 0; bit != 31; ++bit)
        {
            bool in_use = last_pvtgeodetic_.signal_info & mask_2;
            if (bit <= 5 && in_use)
            {
                gps_in_pvt = true;
            }
            if (8 <= bit && bit <= 12 && in_use)
                glo_in_pvt = true;
            if (((13 <= bit && bit <= 14) || (28 <= bit && bit <= 30)) && in_use)
                com_in_pvt = true;
            if ((bit == 17 || (19 <= bit && bit <= 22)) && in_use)
                gal_in_pvt = true;
            mask_2 *= 2;
        }
        // Below, booleans will be promoted to integers automatically.
        uint16_t service =
            gps_in_pvt * 1 + glo_in_pvt * 2 + com_in_pvt * 4 + gal_in_pvt * 8;
        msg->status.service = service;
        msg->latitude = last_pvtgeodetic_.latitude * 360 /
                        (2 * boost::math::constants::pi<double>());
        msg->longitude = last_pvtgeodetic_.longitude * 360 /
                        (2 * boost::math::constants::pi<double>());
        msg->altitude = last_pvtgeodetic_.height;
        msg->position_covariance[0] =
            static_cast<double>(last_poscovgeodetic_.cov_lonlon);
        msg->position_covariance[1] =
            static_cast<double>(last_poscovgeodetic_.cov_latlon);
        msg->position_covariance[2] =
            static_cast<double>(last_poscovgeodetic_.cov_lonhgt);
        msg->position_covariance[3] =
            static_cast<double>(last_poscovgeodetic_.cov_latlon);
        msg->position_covariance[4] =
            static_cast<double>(last_poscovgeodetic_.cov_latlat);
        msg->position_covariance[5] =
            static_cast<double>(last_poscovgeodetic_.cov_lathgt);
        msg->position_covariance[6] =
            static_cast<double>(last_poscovgeodetic_.cov_lonhgt);
        msg->position_covariance[7] =
            static_cast<double>(last_poscovgeodetic_.cov_lathgt);
        msg->position_covariance[8] =
            static_cast<double>(last_poscovgeodetic_.cov_hgthgt);
        msg->position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_KNOWN;
        return msg;
    }

    if (septentrio_receiver_type_ == "ins")
    {
        int SBIdx = 0;
        sensor_msgs::NavSatFixPtr msg = boost::make_shared<sensor_msgs::NavSatFix>();
		uint16_t type_of_pvt = ((uint16_t)(last_insnavgeod_.gnss_mode)) & mask;
		switch (type_of_pvt_map[type_of_pvt])
		{
		case evNoPVT:
		{
			msg->status.status = sensor_msgs::NavSatStatus::STATUS_NO_FIX;
			break;
		}
		case evStandAlone:
		case evFixed:
		{
			msg->status.status = sensor_msgs::NavSatStatus::STATUS_FIX;
			break;
		}
		case evDGPS:
		case evRTKFixed:
		case evRTKFloat:
		case evMovingBaseRTKFixed:
		case evMovingBaseRTKFloat:
		case evPPP:
		{
			msg->status.status = sensor_msgs::NavSatStatus::STATUS_GBAS_FIX;
			break;
		}
		case evSBAS:
		{
			msg->status.status = sensor_msgs::NavSatStatus::STATUS_SBAS_FIX;
			break;
		}
		default:
		{
			throw std::runtime_error(
				"INSNavGeod's Mode field contains an invalid type of PVT solution.");
		}
		}
		bool gps_in_pvt = false;
		bool glo_in_pvt = false;
		bool com_in_pvt = false;
		bool gal_in_pvt = false;
		uint32_t mask_2 = 1;
		for (int bit = 0; bit != 31; ++bit)
		{
			bool in_use = last_pvtgeodetic_.signal_info & mask_2;
			if (bit <= 5 && in_use)
			{
				gps_in_pvt = true;
			}
			if (8 <= bit && bit <= 12 && in_use)
				glo_in_pvt = true;
			if (((13 <= bit && bit <= 14) || (28 <= bit && bit <= 30)) && in_use)
				com_in_pvt = true;
			if ((bit == 17 || (19 <= bit && bit <= 22)) && in_use)
				gal_in_pvt = true;
			mask_2 *= 2;
		}
		// Below, booleans will be promoted to integers automatically.
		uint16_t service =
			gps_in_pvt * 1 + glo_in_pvt * 2 + com_in_pvt * 4 + gal_in_pvt * 8;
		msg->status.service = service;
		msg->latitude = last_insnavgeod_.latitude * 360 /
						(2 * boost::math::constants::pi<double>());
		msg->longitude = last_insnavgeod_.longitude * 360 /
						(2 * boost::math::constants::pi<double>());
		msg->altitude = last_insnavgeod_.height;

		if((last_insnavgeod_.sb_list & 1) !=0)
		{
			msg->position_covariance[0] = std::pow(static_cast<double>(last_insnavgeod_.
											INSNavGeodData[SBIdx].PosStdDev.longitude_std_dev),2);
			msg->position_covariance[4] = std::pow(static_cast<double>(last_insnavgeod_.
											INSNavGeodData[SBIdx].PosStdDev.latitude_std_dev),2);
			msg->position_covariance[8] = std::pow(static_cast<double>(last_insnavgeod_.
											INSNavGeodData[SBIdx].PosStdDev.height_std_dev),2);
		}
		SBIdx++;
		if((last_insnavgeod_.sb_list & 32) !=0)
		{
			msg->position_covariance[1] = static_cast<double>(last_insnavgeod_.
											INSNavGeodData[SBIdx].PosCov.latitude_longitude_cov);
			msg->position_covariance[2] = static_cast<double>(last_insnavgeod_.
											INSNavGeodData[SBIdx].PosCov.longitude_height_cov);
			msg->position_covariance[3] = static_cast<double>(last_insnavgeod_.
											INSNavGeodData[SBIdx].PosCov.latitude_longitude_cov);
			msg->position_covariance[5] = static_cast<double>(last_insnavgeod_.
											INSNavGeodData[SBIdx].PosCov.latitude_height_cov);
			msg->position_covariance[6] = static_cast<double>(last_insnavgeod_.
											INSNavGeodData[SBIdx].PosCov.longitude_height_cov);
			msg->position_covariance[7] = static_cast<double>(last_insnavgeod_.
											INSNavGeodData[SBIdx].PosCov.latitude_height_cov);
		}
		msg->position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
    }
	return msg;
};

/**
 * Note that the field "dip" denotes the local magnetic inclination in degrees 
 * (positive when the magnetic field points downwards (into the Earth)). 
 * This quantity cannot be calculated by most Septentrio
 * receivers. We assume that for the ROS field "err_time", we are requested to
 * provide the 2 sigma uncertainty on the clock bias estimate in square meters, not
 * the clock drift estimate (latter would be
 * "2*std::sqrt(static_cast<double>(last_velcovgeodetic_.Cov_DtDt))").
 * The "err_track" entry is calculated via the Gaussian error propagation formula
 * from the eastward and the northward velocities. For the formula's usage we have to
 * assume that the eastward and the northward velocities are independent variables.
 * Note that elevations and azimuths of visible satellites are taken from the
 * ChannelStatus block, which provides 1 degree precision, while the SatVisibility
 * block could provide hundredths of degrees precision. Change if imperative for your
 * application... Definition of "visible satellite" adopted here: We define a visible
 * satellite as being !up to! "in sync" mode with the receiver, which corresponds to
 * last_measepoch_.N (signal-to-noise ratios are thereby available for these), though
 * not last_channelstatus_.N, which also includes those "in search". In case certain
 * values appear unphysical, please consult the firmware, since those most likely
 * refer to Do-Not-Use values.
 */
gps_common::GPSFixPtr io_comm_rx::RxMessage::GPSFixCallback()
{
	gps_common::GPSFixPtr msg = boost::make_shared<gps_common::GPSFix>();
	msg->status.satellites_used = static_cast<uint16_t>(last_pvtgeodetic_.nr_sv);

	// MeasEpoch Processing
	std::vector<int32_t> cno_tracked;
	std::vector<int32_t> svid_in_sync;
	{
		uint8_t sb1_size = last_measepoch_.sb1_size;
		uint8_t sb2_size = last_measepoch_.sb2_size;
		uint8_t* sb_start = &last_measepoch_.data[0];
		int32_t index = sb_start - &last_measepoch_.block_header.sync_1;
		for (int32_t i = 0; i < static_cast<int32_t>(last_measepoch_.n); ++i)
		{
			// Define MeasEpochChannelType1 struct for the corresponding sub-block
			MeasEpochChannelType1* measepoch_channel_type1 =
				reinterpret_cast<MeasEpochChannelType1*>(
					&last_measepoch_.block_header.sync_1 + index);
			svid_in_sync.push_back(
				static_cast<int32_t>(measepoch_channel_type1->sv_id));
			uint8_t type_mask =
				15; // We extract the first four bits using this mask.
			if (((measepoch_channel_type1->type & type_mask) ==
				static_cast<uint8_t>(1)) ||
				((measepoch_channel_type1->type & type_mask) ==
				static_cast<uint8_t>(2)))
			{
				cno_tracked.push_back(
					static_cast<int32_t>(measepoch_channel_type1->cn0) / 4);
			} else
			{
				cno_tracked.push_back(
					static_cast<int32_t>(measepoch_channel_type1->cn0) / 4 +
					static_cast<int32_t>(10));
			}
			index += sb1_size;
			for (int32_t j = 0;
				j < static_cast<int32_t>(measepoch_channel_type1->n_type2); j++)
			{
				index += sb2_size;
			}
		}
	}

	// ChannelStatus Processing
	std::vector<int32_t> svid_in_sync_2;
	std::vector<int32_t> elevation_tracked;
	std::vector<int32_t> azimuth_tracked;
	std::vector<int32_t> svid_pvt;
	svid_pvt.clear();
	std::vector<int32_t> ordering;
	{
		uint8_t sb1_size = last_channelstatus_.sb1_size;
		uint8_t sb2_size = last_channelstatus_.sb2_size;
		uint8_t* sb_start = &last_channelstatus_.data[0];
		int32_t index = sb_start - &last_channelstatus_.block_header.sync_1;
		// ROS_DEBUG("index is %i", index); // yields 20, as expected

		uint16_t azimuth_mask = 511;
		for (int32_t i = 0; i < static_cast<int32_t>(last_channelstatus_.n); i++)
		{
			// Define ChannelSatInfo struct for the corresponding sub-block
			ChannelSatInfo* channel_sat_info = reinterpret_cast<ChannelSatInfo*>(
				&last_channelstatus_.block_header.sync_1 + index);
			bool to_be_added = false;
			for (int32_t j = 0; j < static_cast<int32_t>(svid_in_sync.size()); ++j)
			{
				if (svid_in_sync[j] == static_cast<int32_t>(channel_sat_info->sv_id))
				{
					ordering.push_back(j);
					to_be_added = true;
					break;
				}
			}
			if (to_be_added)
			{
				svid_in_sync_2.push_back(
					static_cast<int32_t>(channel_sat_info->sv_id));
				elevation_tracked.push_back(
					static_cast<int32_t>(channel_sat_info->elev));
				azimuth_tracked.push_back(static_cast<int32_t>(
					(channel_sat_info->az_rise_set & azimuth_mask)));
			}
			index += sb1_size;
			for (int32_t j = 0; j < static_cast<int32_t>(channel_sat_info->n2); j++)
			{
				// Define ChannelStateInfo struct for the corresponding sub-block
				ChannelStateInfo* channel_state_info =
					reinterpret_cast<ChannelStateInfo*>(
						&last_channelstatus_.block_header.sync_1 + index);
				bool pvt_status = false;
				uint16_t pvt_status_mask = std::pow(2, 15) + std::pow(2, 14);
				for (int k = 15; k != -1; k -= 2)
				{
					uint16_t pvt_status_value =
						(channel_state_info->pvt_status & pvt_status_mask) >> k - 1;
					if (pvt_status_value == 2)
					{
						pvt_status = true;
					}
					if (k > 1)
					{
						pvt_status_mask = pvt_status_mask - std::pow(2, k) -
										std::pow(2, k - 1) + std::pow(2, k - 2) +
										std::pow(2, k - 3);
					}
				}
				if (pvt_status)
				{
					svid_pvt.push_back(
						static_cast<int32_t>(channel_sat_info->sv_id));
				}
				index += sb2_size;
			}
		}
	}
	msg->status.satellite_used_prn =
		svid_pvt; // Entries such as int32[] in ROS messages are to be treated as
				// std::vectors.
	msg->status.satellites_visible = static_cast<uint16_t>(svid_in_sync.size());
	msg->status.satellite_visible_prn = svid_in_sync_2;
	msg->status.satellite_visible_z = elevation_tracked;
	msg->status.satellite_visible_azimuth = azimuth_tracked;

	// Reordering CNO vector to that of all previous arrays
	std::vector<int32_t> cno_tracked_reordered;
	if (static_cast<int32_t>(last_channelstatus_.n) != 0)
	{
		for (int32_t k = 0; k < static_cast<int32_t>(ordering.size()); ++k)
		{
			cno_tracked_reordered.push_back(cno_tracked[ordering[k]]);
		}
	}
	msg->status.satellite_visible_snr = cno_tracked_reordered;
	msg->err_time = 2 * std::sqrt(static_cast<double>(last_poscovgeodetic_.cov_bb));
	
	if (septentrio_receiver_type_ == "gnss")
    {
		
        // PVT Status Analysis
        uint16_t status_mask = 15; // We extract the first four bits using this mask.
        uint16_t type_of_pvt = ((uint16_t)(last_pvtgeodetic_.mode)) & status_mask;
        switch (type_of_pvt_map[type_of_pvt])
        {
        case evNoPVT:
        {
            msg->status.status = gps_common::GPSStatus::STATUS_NO_FIX;
            break;
        }
        case evStandAlone:
        case evFixed:
        {
            msg->status.status = gps_common::GPSStatus::STATUS_FIX;
            break;
        }
        case evDGPS:
        case evRTKFixed:
        case evRTKFloat:
        case evMovingBaseRTKFixed:
        case evMovingBaseRTKFloat:
        case evPPP:
        {
            msg->status.status = gps_common::GPSStatus::STATUS_GBAS_FIX;
            break;
        }
        case evSBAS:
        {
            uint16_t reference_id = last_pvtgeodetic_.reference_id;
            // Here come the PRNs of the 4 WAAS satellites..
            if (reference_id == 131 || reference_id == 133 || reference_id == 135 ||
                reference_id == 135)
            {
                msg->status.status = gps_common::GPSStatus::STATUS_WAAS_FIX;
            } else
            {
                msg->status.status = gps_common::GPSStatus::STATUS_SBAS_FIX;
            }
            break;
        }
        default:
        {
            throw std::runtime_error(
                "PVTGeodetic's Mode field contains an invalid type of PVT solution.");
        }
        }
        // Doppler is not used when calculating the velocities of, say, mosaic-x5, hence:
        msg->status.motion_source = gps_common::GPSStatus::SOURCE_POINTS;
        // Doppler is not used when calculating the orientation of, say, mosaic-x5,
        // hence:
        msg->status.orientation_source = gps_common::GPSStatus::SOURCE_POINTS;
        msg->status.position_source = gps_common::GPSStatus::SOURCE_GPS;
        msg->latitude = static_cast<double>(last_pvtgeodetic_.latitude) * 360 /
                        (2 * boost::math::constants::pi<double>());
        msg->longitude = static_cast<double>(last_pvtgeodetic_.longitude) * 360 /
                        (2 * boost::math::constants::pi<double>());
        msg->altitude = static_cast<double>(last_pvtgeodetic_.height);
        // Note that cog is of type float32 while track is of type float64.
        msg->track = static_cast<double>(last_pvtgeodetic_.cog);
        msg->speed = std::sqrt(std::pow(static_cast<double>(last_pvtgeodetic_.vn), 2) +
                            std::pow(static_cast<double>(last_pvtgeodetic_.ve), 2));
        msg->climb = static_cast<double>(last_pvtgeodetic_.vu);
        msg->pitch = static_cast<double>(last_atteuler_.pitch);
        msg->roll = static_cast<double>(last_atteuler_.roll);
        if (last_dop_.pdop == static_cast<uint16_t>(0) ||
            last_dop_.tdop == static_cast<uint16_t>(0))
        {
            msg->gdop = static_cast<double>(-1);
        } else
        {
            msg->gdop =
                std::sqrt(std::pow(static_cast<double>(last_dop_.pdop) / 100, 2) +
                        std::pow(static_cast<double>(last_dop_.tdop) / 100, 2));
        }
        if (last_dop_.pdop == static_cast<uint16_t>(0))
        {
            msg->pdop = static_cast<double>(-1);
        } else
        {
            msg->pdop = static_cast<double>(last_dop_.pdop) / 100;
        }
        if (last_dop_.hdop == static_cast<uint16_t>(0))
        {
            msg->hdop = static_cast<double>(-1);
        } else
        {
            msg->hdop = static_cast<double>(last_dop_.hdop) / 100;
        }
        if (last_dop_.vdop == static_cast<uint16_t>(0))
        {
            msg->vdop = static_cast<double>(-1);
        } else
        {
            msg->vdop = static_cast<double>(last_dop_.vdop) / 100;
        }
        if (last_dop_.tdop == static_cast<uint16_t>(0))
        {
            msg->tdop = static_cast<double>(-1);
        } else
        {
            msg->tdop = static_cast<double>(last_dop_.tdop) / 100;
        }
        msg->time = static_cast<double>(last_pvtgeodetic_.tow) / 1000 +
                    static_cast<double>(last_pvtgeodetic_.wnc * 7 * 24 * 60 * 60);
        msg->err = 2 * (std::sqrt(static_cast<double>(last_poscovgeodetic_.cov_latlat) +
                                static_cast<double>(last_poscovgeodetic_.cov_lonlon) +
                                static_cast<double>(last_poscovgeodetic_.cov_hgthgt)));
        msg->err_horz =
            2 * (std::sqrt(static_cast<double>(last_poscovgeodetic_.cov_latlat) +
                        static_cast<double>(last_poscovgeodetic_.cov_lonlon)));
        msg->err_vert =
            2 * std::sqrt(static_cast<double>(last_poscovgeodetic_.cov_hgthgt));
        msg->err_track =
            2 *
            (std::sqrt(
                std::pow(static_cast<double>(1) /
                            (static_cast<double>(last_pvtgeodetic_.vn) +
                            std::pow(static_cast<double>(last_pvtgeodetic_.ve), 2) /
                                static_cast<double>(last_pvtgeodetic_.vn)),
                        2) *
                    static_cast<double>(last_poscovgeodetic_.cov_lonlon) +
                std::pow((static_cast<double>(last_pvtgeodetic_.ve)) /
                            (std::pow(static_cast<double>(last_pvtgeodetic_.vn), 2) +
                            std::pow(static_cast<double>(last_pvtgeodetic_.ve), 2)),
                        2) *
                    static_cast<double>(last_poscovgeodetic_.cov_latlat)));
        msg->err_speed =
            2 * (std::sqrt(static_cast<double>(last_velcovgeodetic_.cov_vnvn) +
                        static_cast<double>(last_velcovgeodetic_.cov_veve)));
        msg->err_climb =
            2 * std::sqrt(static_cast<double>(last_velcovgeodetic_.cov_vuvu));
        msg->err_pitch =
            2 * std::sqrt(static_cast<double>(last_attcoveuler_.cov_pitchpitch));
        msg->err_roll =
            2 * std::sqrt(static_cast<double>(last_attcoveuler_.cov_rollroll));
        msg->position_covariance[0] =
            static_cast<double>(last_poscovgeodetic_.cov_lonlon);
        msg->position_covariance[1] =
            static_cast<double>(last_poscovgeodetic_.cov_latlon);
        msg->position_covariance[2] =
            static_cast<double>(last_poscovgeodetic_.cov_lonhgt);
        msg->position_covariance[3] =
            static_cast<double>(last_poscovgeodetic_.cov_latlon);
        msg->position_covariance[4] =
            static_cast<double>(last_poscovgeodetic_.cov_latlat);
        msg->position_covariance[5] =
            static_cast<double>(last_poscovgeodetic_.cov_lathgt);
        msg->position_covariance[6] =
            static_cast<double>(last_poscovgeodetic_.cov_lonhgt);
        msg->position_covariance[7] =
            static_cast<double>(last_poscovgeodetic_.cov_lathgt);
        msg->position_covariance[8] =
            static_cast<double>(last_poscovgeodetic_.cov_hgthgt);
        msg->position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_KNOWN;
		
    }
    
    if (septentrio_receiver_type_ == "ins")
    {
        int SBIdx = 0;

        // PVT Status Analysis
        uint16_t status_mask = 15; // We extract the first four bits using this mask.
        uint16_t type_of_pvt = ((uint16_t)(last_insnavgeod_.gnss_mode)) & status_mask;
        switch (type_of_pvt_map[type_of_pvt])
        {
        case evNoPVT:
        {
            msg->status.status = gps_common::GPSStatus::STATUS_NO_FIX;
            break;
        }
        case evStandAlone:
        case evFixed:
        {
            msg->status.status = gps_common::GPSStatus::STATUS_FIX;
            break;
        }
        case evDGPS:
        case evRTKFixed:
        case evRTKFloat:
        case evMovingBaseRTKFixed:
        case evMovingBaseRTKFloat:
        case evPPP:
        {
            msg->status.status = gps_common::GPSStatus::STATUS_GBAS_FIX;
            break;
        }
        case evSBAS:
        default:
        {
            throw std::runtime_error(
                "INSNavGeod's Mode field contains an invalid type of PVT solution.");
        }
        }
        // Doppler is not used when calculating the velocities of, say, mosaic-x5, hence:
        msg->status.motion_source = gps_common::GPSStatus::SOURCE_POINTS;
        // Doppler is not used when calculating the orientation of, say, mosaic-x5,
        // hence:
        msg->status.orientation_source = gps_common::GPSStatus::SOURCE_POINTS;
        msg->status.position_source = gps_common::GPSStatus::SOURCE_GPS;
        msg->latitude = static_cast<double>(last_insnavgeod_.latitude) * 360 /
                        (2 * boost::math::constants::pi<double>());
        msg->longitude = static_cast<double>(last_insnavgeod_.longitude) * 360 /
                        (2 * boost::math::constants::pi<double>());
        msg->altitude = static_cast<double>(last_insnavgeod_.height);
        // Note that cog is of type float32 while track is of type float64.
        if ((last_insnavgeod_.sb_list & 2) !=0)
        {
            msg->track = static_cast<double>(last_insnavgeod_.INSNavGeodData[SBIdx].Att.heading);
			msg->pitch = static_cast<double>(last_insnavgeod_.INSNavGeodData[SBIdx].Att.pitch);
			msg->roll = static_cast<double>(last_insnavgeod_.INSNavGeodData[SBIdx].Att.roll);
        }
        if ((last_insnavgeod_.sb_list & 8) !=0)
        {
            msg->speed = std::sqrt(std::pow(static_cast<double>(last_insnavgeod_.INSNavGeodData[SBIdx].Vel.vn),2) + 
                                    std::pow(static_cast<double>(last_insnavgeod_.INSNavGeodData[SBIdx].Vel.ve),2));

			msg->climb = static_cast<double>(last_insnavgeod_.INSNavGeodData[SBIdx].Vel.vu);
        }
        if (last_dop_.pdop == static_cast<uint16_t>(0) ||
            last_dop_.tdop == static_cast<uint16_t>(0))
        {
            msg->gdop = static_cast<double>(-1);
        } else
        {
            msg->gdop =
                std::sqrt(std::pow(static_cast<double>(last_dop_.pdop) / 100, 2) +
                        std::pow(static_cast<double>(last_dop_.tdop) / 100, 2));
        }
        if (last_dop_.pdop == static_cast<uint16_t>(0))
        {
            msg->pdop = static_cast<double>(-1);
        } else
        {
            msg->pdop = static_cast<double>(last_dop_.pdop) / 100;
        }
        if (last_dop_.hdop == static_cast<uint16_t>(0))
        {
            msg->hdop = static_cast<double>(-1);
        } else
        {
            msg->hdop = static_cast<double>(last_dop_.hdop) / 100;
        }
        if (last_dop_.vdop == static_cast<uint16_t>(0))
        {
            msg->vdop = static_cast<double>(-1);
        } else
        {
            msg->vdop = static_cast<double>(last_dop_.vdop) / 100;
        }
        if (last_dop_.tdop == static_cast<uint16_t>(0))
        {
            msg->tdop = static_cast<double>(-1);
        } else
        {
            msg->tdop = static_cast<double>(last_dop_.tdop) / 100;
        }
        msg->time = static_cast<double>(last_insnavgeod_.tow) / 1000 +
                    static_cast<double>(last_insnavgeod_.wnc * 7 * 24 * 60 * 60);
        if((last_insnavgeod_.sb_list & 1) !=0)
        {
            msg->err = 2*(std::sqrt(std::pow(static_cast<double>(last_insnavgeod_.INSNavGeodData[SBIdx].PosStdDev.latitude_std_dev),2) + 
                        std::pow(static_cast<double>(last_insnavgeod_.INSNavGeodData[SBIdx].PosStdDev.longitude_std_dev),2)+ 
                        std::pow(static_cast<double>(last_insnavgeod_.INSNavGeodData[SBIdx].PosStdDev.height_std_dev),2)));
			msg->err_horz = 2*(std::sqrt(std::pow(static_cast<double>(last_insnavgeod_.INSNavGeodData[SBIdx].PosStdDev.latitude_std_dev),2) + 
                        std::pow(static_cast<double>(last_insnavgeod_.INSNavGeodData[SBIdx].PosStdDev.longitude_std_dev),2)));
			msg->err_vert = 2*(std::sqrt(std::pow(static_cast<double>(last_insnavgeod_.INSNavGeodData[SBIdx].PosStdDev.height_std_dev),2)));
        }
        if (((last_insnavgeod_.sb_list & 8) !=0) || ((last_insnavgeod_.sb_list & 1) !=0))
        {
            msg->err_track = 
                2 * 
                (std::sqrt(
                    std::pow(static_cast<double>(1) /
                                (static_cast<double>(last_insnavgeod_.INSNavGeodData[SBIdx].Vel.vn) + 
                                std::pow(static_cast<double>(last_insnavgeod_.INSNavGeodData[SBIdx].Vel.ve),2) / 
                                    static_cast<double>(last_insnavgeod_.INSNavGeodData[SBIdx].Vel.vn)),
                            2) * 
                        std::pow(static_cast<double>(last_insnavgeod_.INSNavGeodData[SBIdx].PosStdDev.longitude_std_dev),2) +
                        std::pow((static_cast<double>(last_insnavgeod_.INSNavGeodData[SBIdx].Vel.ve)) /
                                (std::pow(static_cast<double>(last_insnavgeod_.INSNavGeodData[SBIdx].Vel.vn),2) +
                                std::pow(static_cast<double>(last_insnavgeod_.INSNavGeodData[SBIdx].Vel.ve),2)),
                            2) *
                        std::pow(static_cast<double>(last_insnavgeod_.INSNavGeodData[SBIdx].PosStdDev.latitude_std_dev),2)));   
        }
        if ((last_insnavgeod_.sb_list & 8) !=0)
        {
            msg->err_speed = 2 * (std::sqrt(std::pow(static_cast<double>(last_insnavgeod_.INSNavGeodData[SBIdx].Vel.vn),2) + 
                                std::pow(static_cast<double>(last_insnavgeod_.INSNavGeodData[SBIdx].Vel.ve),2)));
			msg->err_climb = 2 * std::sqrt(std::pow(static_cast<double>(last_insnavgeod_.INSNavGeodData[SBIdx].Vel.vn),2));
        }
        if ((last_insnavgeod_.sb_list & 2) !=0)
        {
             msg->err_pitch = 2 * std::sqrt(std::pow(
                 static_cast<double>(last_insnavgeod_.INSNavGeodData[SBIdx].Att.pitch),2));
        }
        if ((last_insnavgeod_.sb_list & 2) !=0)
        {
             msg->err_pitch = 2 * std::sqrt(std::pow(
                 static_cast<double>(last_insnavgeod_.INSNavGeodData[SBIdx].Att.roll),2));
        }
        if((last_insnavgeod_.sb_list & 1) !=0)
        {
            msg->position_covariance[0] = std::pow(static_cast<double>(last_insnavgeod_.
                                            INSNavGeodData[SBIdx].PosStdDev.longitude_std_dev),2);
			msg->position_covariance[4] = std::pow(static_cast<double>(last_insnavgeod_.
                                            INSNavGeodData[SBIdx].PosStdDev.latitude_std_dev),2);
			msg->position_covariance[8] = std::pow(static_cast<double>(last_insnavgeod_.
                                            INSNavGeodData[SBIdx].PosStdDev.height_std_dev),2);
        }
        SBIdx++;
        if((last_insnavgeod_.sb_list & 32) !=0)
        {
            msg->position_covariance[1] = static_cast<double>(last_insnavgeod_.
                                            INSNavGeodData[SBIdx].PosCov.latitude_longitude_cov);
			msg->position_covariance[2] = static_cast<double>(last_insnavgeod_.
                                            INSNavGeodData[SBIdx].PosCov.longitude_height_cov);
			msg->position_covariance[3] = static_cast<double>(last_insnavgeod_.
                                            INSNavGeodData[SBIdx].PosCov.latitude_longitude_cov);
			msg->position_covariance[5] = static_cast<double>(last_insnavgeod_.
                                            INSNavGeodData[SBIdx].PosCov.latitude_height_cov);
			msg->position_covariance[6] = static_cast<double>(last_insnavgeod_.
                                            INSNavGeodData[SBIdx].PosCov.longitude_height_cov);
			msg->position_covariance[7] = static_cast<double>(last_insnavgeod_.
                                            INSNavGeodData[SBIdx].PosCov.latitude_height_cov);
        }
        msg->position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
    }
	return msg;
};

/// If the current time shall be employed, it is calculated via the time(NULL)
/// function found in the \<ctime\> library At the time of writing the code (2020),
/// the GPS time was ahead of UTC time by 18 (leap) seconds. Adapt the g_leap_seconds
/// ROSaic parameter accordingly as soon as the next leap second is inserted into the
/// UTC time.
ros::Time io_comm_rx::timestampSBF(uint32_t tow, uint16_t wnc, bool use_gnss)
{
	ros::Time time_obj;
	if (use_gnss)
	{
		// conversion from GPS time of week and week number to UTC taking leap seconds into account
		static uint64_t secToNSec    = 1000000000;
		static uint64_t mSec2NSec    = 1000000;
		static uint64_t nsOfGpsStart = 315964800 * secToNSec; // GPS week counter starts at 1980-01-06 which is 315964800 seconds since Unix epoch (1970-01-01 UTC)
		static uint64_t nsecPerWeek  = 7 * 24 * 60 * 60 * secToNSec;
		uint64_t        UTC          = nsOfGpsStart + tow * mSec2NSec + wnc * nsecPerWeek - g_leap_seconds * secToNSec;
		
		time_obj.fromNSec(UTC);
	}
	else
    {
        time_obj = ros::Time::now();
    }
	return time_obj;
}

bool io_comm_rx::RxMessage::found()
{
    if (found_)
        return true;

    // Verify header bytes
    if (!this->isSBF() && !this->isNMEA() && !this->isResponse() &&
        !(g_read_cd && this->isConnectionDescriptor()))
    {
        return false;
    }

    found_ = true;
    return true;
}

const uint8_t* io_comm_rx::RxMessage::search()
{
    if (found_)
    {
        next();
    }
    // Search for message or a response header
    for (; count_ > 0; --count_, ++data_)
    {
        if (this->isSBF() || this->isNMEA() || this->isResponse() ||
            (g_read_cd && this->isConnectionDescriptor()))
        {
            break;
        }
    }
    found_ = true;
    return data_;
}

std::size_t io_comm_rx::RxMessage::messageSize()
{
    uint16_t pos = 0;
    message_size_ = 0;
    std::size_t count_copy = count_;
    if (this->isResponse())
    {
        do
        {
            ++message_size_;
            ++pos;
            --count_copy;
            if (count_copy == 0)
                break;
        } while (!((data_[pos] == CARRIAGE_RETURN && data_[pos + 1] == LINE_FEED)) ||
                 (data_[pos] == CARRIAGE_RETURN && data_[pos + 1] == LINE_FEED &&
                  data_[pos + 2] == 0x20 && data_[pos + 3] == 0x20 &&
                  data_[pos + 4] == 0x4E) ||
                 (data_[pos] == CARRIAGE_RETURN && data_[pos + 1] == LINE_FEED &&
                  data_[pos + 2] == 0x20 && data_[pos + 3] == 0x20 &&
                  data_[pos + 4] == 0x53) ||
                 (data_[pos] == CARRIAGE_RETURN && data_[pos + 1] == LINE_FEED &&
                  data_[pos + 2] == 0x20 && data_[pos + 3] == 0x20 &&
                  data_[pos + 4] == 0x52));
    } else
    {
        do
        {
            ++message_size_;
            ++pos;
            --count_copy;
            if (count_copy == 0)
                break;
        } while (!((data_[pos] == CARRIAGE_RETURN && data_[pos + 1] == LINE_FEED) ||
                   data_[pos] == CARRIAGE_RETURN || data_[pos] == LINE_FEED));
    }
    return message_size_;
}

bool io_comm_rx::RxMessage::isMessage(const uint16_t id)
{
    if (this->isSBF())
    {
         if (parsing_utilities::getId(data_) == static_cast<const uint16_t>(id))
        // Caution: reinterpret_cast is the most dangerous cast. It's used primarily
        // for particularly weird conversions and bit manipulations, like turning a
        // raw data stream into actual data.
        {
            return true;
        } else
        {
            return false;
        }
    } else
    {
        return false;
    }
}

bool io_comm_rx::RxMessage::isMessage(std::string id)
{
    if (this->isNMEA())
    {
        boost::char_separator<char> sep(",");
        typedef boost::tokenizer<boost::char_separator<char>> tokenizer;
        std::size_t nmea_size = this->messageSize();
        std::string block_in_string(reinterpret_cast<const char*>(data_), nmea_size);
        tokenizer tokens(block_in_string, sep);
        if (*tokens.begin() == id)
        {
            return true;
        } else
        {
            return false;
        }
    } else
    {
        return false;
    }
}

bool io_comm_rx::RxMessage::isSBF()
{
    if (count_ >= 2)
    {
        if (data_[0] == SBF_SYNC_BYTE_1 && data_[1] == SBF_SYNC_BYTE_2)
        {
            return true;
        } else
        {
            return false;
        }
    } else
    {
        return false;
    }
}

bool io_comm_rx::RxMessage::isNMEA()
{
    if (count_ >= 2)
    {
        if ((data_[0] == NMEA_SYNC_BYTE_1 && data_[1] == NMEA_SYNC_BYTE_2_1) ||
            (data_[0] == NMEA_SYNC_BYTE_1 && data_[1] == NMEA_SYNC_BYTE_2_2))
        {
            return true;
        } else
        {
            return false;
        }
    } else
    {
        return false;
    }
}

bool io_comm_rx::RxMessage::isResponse()
{
    if (count_ >= 2)
    {
        if (data_[0] == RESPONSE_SYNC_BYTE_1 && data_[1] == RESPONSE_SYNC_BYTE_2)
        {
            return true;
        } else
        {
            return false;
        }
    } else
    {
        return false;
    }
}

bool io_comm_rx::RxMessage::isConnectionDescriptor()
{
    if (count_ >= 2)
    {
        if (data_[0] == CONNECTION_DESCRIPTOR_BYTE_1 &&
            data_[1] == CONNECTION_DESCRIPTOR_BYTE_2)
        {
            return true;
        } else
        {
            return false;
        }
    } else
    {
        return false;
    }
}

bool io_comm_rx::RxMessage::isErrorMessage()
{
    if (count_ >= 3)
    {
        if (data_[0] == RESPONSE_SYNC_BYTE_1 && data_[1] == RESPONSE_SYNC_BYTE_2 &&
            data_[2] == RESPONSE_SYNC_BYTE_3)
        {
            return true;
        } else
        {
            return false;
        }
    } else
    {
        return false;
    }
}

std::string io_comm_rx::RxMessage::messageID()
{
    if (this->isSBF())
    {
        uint16_t value = parsing_utilities::getId(data_);
        std::stringstream ss;
        ss << value;
        return ss.str();
    }
    if (this->isNMEA())
    {
        boost::char_separator<char> sep(",");
        typedef boost::tokenizer<boost::char_separator<char>> tokenizer;
        std::size_t nmea_size = this->messageSize();
        std::string block_in_string(reinterpret_cast<const char*>(data_), nmea_size);
        tokenizer tokens(block_in_string, sep);
        return *tokens.begin();
    }
    return std::string(); // less CPU work than return "";
}

const uint8_t* io_comm_rx::RxMessage::getPosBuffer() { return data_; }

const uint8_t* io_comm_rx::RxMessage::getEndBuffer() { return data_ + count_; }

uint16_t io_comm_rx::RxMessage::getBlockLength()
{
    if (this->isSBF())
    {
        uint16_t block_length;
        // Note that static_cast<uint16_t>(data_[6]) would just take the one byte
        // "data_[6]" and cast it as requested, !neglecting! the byte "data_[7]".
        block_length = *(reinterpret_cast<const uint16_t*>(data_ + 6));
        return block_length;
    } else
    {
        return 0;
    }
}

/**
 * This method won't make data_ jump to the next message if the current one is an
 * NMEA message or a command reply. In that case, search() will check the bytes one
 * by one for the new message's sync bytes ($P, $G or $R).
 */
void io_comm_rx::RxMessage::next()
{
    std::size_t jump_size;
    if (found())
    {
        if (this->isNMEA() || this->isResponse() ||
            (g_read_cd && this->isConnectionDescriptor()))
        {
            if (g_read_cd && this->isConnectionDescriptor() && g_cd_count == 2)
            {
                g_read_cd = false;
            }
            jump_size = static_cast<uint32_t>(1);
        }
        if (this->isSBF())
        {
            if (crc_check_)
            {
                jump_size = static_cast<std::size_t>(this->getBlockLength());
                // Some corrupted messages that survive the CRC check (this happened)
                // could tell ROSaic their size is 0, which would lead to an endless
                // while loop in the ReadCallback() method of the CallbackHandlers
                // class.
                if (jump_size == 0)
                    jump_size = static_cast<std::size_t>(1);
            } else
            {
                jump_size = static_cast<std::size_t>(1);
            }
        }
    }
    found_ = false;
    data_ += jump_size;
    count_ -= jump_size;
    // ROS_DEBUG("Jump about to happen with jump size %li and count after jump being
    // %li.", jump_size, count_);
    return; // For readability
}

/**
 * Note that putting the default in the definition's argument list instead of the
 * declaration's is an added extra that is not available for function templates,
 * hence no search = false here. Also note that the SBF block header part of the
 * SBF-echoing ROS messages have ID fields that only show the block number as found
 * in the firmware (e.g. 4007 for PVTGeodetic), without the revision number. NMEA
 * 0183 messages are at most 82 characters long in principle, but most Septentrio Rxs
 * by default increase precision on lat/lon s.t. the maximum allowed e.g. for GGA
 * seems to be 89 on a mosaic-x5. Luckily, when parsing we do not care since we just
 * search for \<LF\>\<CR\>.
 */
bool io_comm_rx::RxMessage::read(std::string message_key, bool search)
{
    if (search)
        this->search();
    if (!found())
        return false;
    if (this->isSBF())
    {
        // If the CRC check is unsuccessful, throw an error message.
        crc_check_ = isValid(data_);
        if (!crc_check_)
        {
            throw std::runtime_error(
                "CRC Check returned False. Not a valid data block. Retrieving full SBF block..");
        }
    }
    switch (rx_id_map[message_key])
    {
		case evPVTCartesian: // Position and velocity in XYZ
		{   // The curly bracket here is crucial: Declarations inside a block remain
		    // inside, and will die at
			// the end of the block. Otherwise variable overloading etc.
			septentrio_gnss_driver::PVTCartesianPtr msg =
				boost::make_shared<septentrio_gnss_driver::PVTCartesian>();
			PVTCartesian pvtcartesian;
			memcpy(&pvtcartesian, data_, sizeof(pvtcartesian));
			msg = PVTCartesianCallback(pvtcartesian);
			msg->header.frame_id = g_frame_id;
			uint32_t tow = parsing_utilities::getTow(data_);
			uint16_t wnc = parsing_utilities::getWnc(data_);
			ros::Time time_obj;
			time_obj = timestampSBF(tow, wnc, g_use_gnss_time);
			msg->header.stamp.sec = time_obj.sec;
			msg->header.stamp.nsec = time_obj.nsec;
			msg->block_header.id = 4006;
			static ros::Publisher publisher =
				pNh_->advertise<septentrio_gnss_driver::PVTCartesian>("/pvtcartesian",
																	  g_ROS_QUEUE_SIZE);
			// Wait as long as necessary (only when reading from SBF/PCAP file)
			if (g_read_from_sbf_log || g_read_from_pcap)
			{
				wait(time_obj);
			}
			publisher.publish(*msg);
			break;
		}
		case evPVTGeodetic: // Position and velocity in geodetic coordinate frame (ENU
							// frame)
		{
			septentrio_gnss_driver::PVTGeodeticPtr msg =
				boost::make_shared<septentrio_gnss_driver::PVTGeodetic>();
			memcpy(&last_pvtgeodetic_, data_, sizeof(last_pvtgeodetic_));
			msg = PVTGeodeticCallback(last_pvtgeodetic_);
			msg->header.frame_id = g_frame_id;
			uint32_t tow = parsing_utilities::getTow(data_);
			uint16_t wnc = parsing_utilities::getWnc(data_);
			ros::Time time_obj;
			time_obj = timestampSBF(tow, wnc, g_use_gnss_time);
			msg->header.stamp.sec = time_obj.sec;
			msg->header.stamp.nsec = time_obj.nsec;
			msg->block_header.id = 4007;
			pvtgeodetic_has_arrived_gpsfix_ = true;
			pvtgeodetic_has_arrived_navsatfix_ = true;
			pvtgeodetic_has_arrived_pose_ = true;
			static ros::Publisher publisher =
				pNh_->advertise<septentrio_gnss_driver::PVTGeodetic>("/pvtgeodetic",
																	 g_ROS_QUEUE_SIZE);
			// Wait as long as necessary (only when reading from SBF/PCAP file)
			if (g_read_from_sbf_log || g_read_from_pcap)
			{
				wait(time_obj);
			}
			publisher.publish(*msg);
			break;
		}
		case evPosCovCartesian:
		{
			septentrio_gnss_driver::PosCovCartesianPtr msg =
				boost::make_shared<septentrio_gnss_driver::PosCovCartesian>();
			PosCovCartesian poscovcartesian;
			memcpy(&poscovcartesian, data_, sizeof(poscovcartesian));
			msg = PosCovCartesianCallback(poscovcartesian);
			msg->header.frame_id = g_frame_id;
			uint32_t tow = parsing_utilities::getTow(data_);
			uint16_t wnc = parsing_utilities::getWnc(data_);
			ros::Time time_obj;
			time_obj = timestampSBF(tow, wnc, g_use_gnss_time);
			msg->header.stamp.sec = time_obj.sec;
			msg->header.stamp.nsec = time_obj.nsec;
			msg->block_header.id = 5905;
			static ros::Publisher publisher =
				pNh_->advertise<septentrio_gnss_driver::PosCovCartesian>(
					"/poscovcartesian", g_ROS_QUEUE_SIZE);
			// Wait as long as necessary (only when reading from SBF/PCAP file)
			if (g_read_from_sbf_log || g_read_from_pcap)
			{
				wait(time_obj);
			}
			publisher.publish(*msg);
			break;
		}
		case evPosCovGeodetic:
		{
			septentrio_gnss_driver::PosCovGeodeticPtr msg =
				boost::make_shared<septentrio_gnss_driver::PosCovGeodetic>();
			memcpy(&last_poscovgeodetic_, data_, sizeof(last_poscovgeodetic_));
			msg = PosCovGeodeticCallback(last_poscovgeodetic_);
			msg->header.frame_id = g_frame_id;
			uint32_t tow = parsing_utilities::getTow(data_);
			uint16_t wnc = parsing_utilities::getWnc(data_);
			ros::Time time_obj;
			time_obj = timestampSBF(tow, wnc, g_use_gnss_time);
			msg->header.stamp.sec = time_obj.sec;
			msg->header.stamp.nsec = time_obj.nsec;
			msg->block_header.id = 5906;
			poscovgeodetic_has_arrived_gpsfix_ = true;
			poscovgeodetic_has_arrived_navsatfix_ = true;
			poscovgeodetic_has_arrived_pose_ = true;
			static ros::Publisher publisher =
				pNh_->advertise<septentrio_gnss_driver::PosCovGeodetic>(
					"/poscovgeodetic", g_ROS_QUEUE_SIZE);
			// Wait as long as necessary (only when reading from SBF/PCAP file)
			if (g_read_from_sbf_log || g_read_from_pcap)
			{
				wait(time_obj);
			}
			publisher.publish(*msg);
			break;
		}
		case evAttEuler:
		{
			septentrio_gnss_driver::AttEulerPtr msg =
				boost::make_shared<septentrio_gnss_driver::AttEuler>();
			memcpy(&last_atteuler_, data_, sizeof(last_atteuler_));
			msg = AttEulerCallback(last_atteuler_);
			msg->header.frame_id = g_frame_id;
			uint32_t tow = parsing_utilities::getTow(data_);
			uint16_t wnc = parsing_utilities::getWnc(data_);
			ros::Time time_obj;
			time_obj = timestampSBF(tow, wnc, g_use_gnss_time);
			msg->header.stamp.sec = time_obj.sec;
			msg->header.stamp.nsec = time_obj.nsec;
			msg->block_header.id = 5938;
			atteuler_has_arrived_gpsfix_ = true;
			atteuler_has_arrived_pose_ = true;
			static ros::Publisher publisher =
				pNh_->advertise<septentrio_gnss_driver::AttEuler>("/atteuler",
																  g_ROS_QUEUE_SIZE);
			// Wait as long as necessary (only when reading from SBF/PCAP file)
			if (g_read_from_sbf_log || g_read_from_pcap)
			{
				wait(time_obj);
			}
			publisher.publish(*msg);
			break;
		}
		case evAttCovEuler:
		{
			septentrio_gnss_driver::AttCovEulerPtr msg =
				boost::make_shared<septentrio_gnss_driver::AttCovEuler>();
			memcpy(&last_attcoveuler_, data_, sizeof(last_attcoveuler_));
			msg = AttCovEulerCallback(last_attcoveuler_);
			msg->header.frame_id = g_frame_id;
			uint32_t tow = parsing_utilities::getTow(data_);
			uint16_t wnc = parsing_utilities::getWnc(data_);
			ros::Time time_obj;
			time_obj = timestampSBF(tow, wnc, g_use_gnss_time);
			msg->header.stamp.sec = time_obj.sec;
			msg->header.stamp.nsec = time_obj.nsec;
			msg->block_header.id = 5939;
			attcoveuler_has_arrived_gpsfix_ = true;
			attcoveuler_has_arrived_pose_ = true;
			static ros::Publisher publisher =
				pNh_->advertise<septentrio_gnss_driver::AttCovEuler>("/attcoveuler",
																	 g_ROS_QUEUE_SIZE);
			// Wait as long as necessary (only when reading from SBF/PCAP file)
			if (g_read_from_sbf_log || g_read_from_pcap)
			{
				wait(time_obj);
			}
			publisher.publish(*msg);
			break;
		}
		case evINSNavCart: // Position, velocity and orientation in cartesian coordinate frame (ENU
							// frame)
		{
			septentrio_gnss_driver::INSNavCartPtr msg =
				boost::make_shared<septentrio_gnss_driver::INSNavCart>();
			INSNavCart insnavcart;
			memcpy(&insnavcart, data_, sizeof(insnavcart));
			msg = INSNavCartCallback(insnavcart);
			msg->header.frame_id = g_frame_id;
			uint32_t tow = parsing_utilities::getTow(data_);
			uint16_t wnc = parsing_utilities::getWnc(data_);
			ros::Time time_obj;
			time_obj = timestampSBF(tow, wnc, g_use_gnss_time);
			msg->header.stamp.sec = time_obj.sec;
			msg->header.stamp.nsec = time_obj.nsec;
			msg->block_header.id = 4225;
			static ros::Publisher publisher = 
				pNh_->advertise<septentrio_gnss_driver::INSNavCart>(
					"/insnavcart", g_ROS_QUEUE_SIZE);
			// Wait as long as necessary (only when reading from SBF/PCAP file)
			if (g_read_from_sbf_log || g_read_from_pcap)
			{
				wait(time_obj);
			}
			publisher.publish(*msg);
			break;
		}
		case evINSNavGeod: // Position, velocity and orientation in geodetic coordinate frame (ENU
							// frame)
		{
			septentrio_gnss_driver::INSNavGeodPtr msg =
				boost::make_shared<septentrio_gnss_driver::INSNavGeod>();
			memcpy(&last_insnavgeod_, data_, sizeof(last_insnavgeod_));
			msg = INSNavGeodCallback(last_insnavgeod_);
			msg->header.frame_id = g_frame_id;
			uint32_t tow = parsing_utilities::getTow(data_);
			uint16_t wnc = parsing_utilities::getWnc(data_);
			ros::Time time_obj;
			time_obj = timestampSBF(tow, wnc, g_use_gnss_time);
			msg->header.stamp.sec = time_obj.sec;
			msg->header.stamp.nsec = time_obj.nsec;
			msg->block_header.id = 4226;
			insnavgeod_has_arrived_gpsfix_ = true;
			insnavgeod_has_arrived_navsatfix_ = true;
			insnavgeod_has_arrived_pose_ = true;
			static ros::Publisher publisher =
				pNh_->advertise<septentrio_gnss_driver::INSNavGeod>("/insnavgeod",
																	 g_ROS_QUEUE_SIZE);
			// Wait as long as necessary (only when reading from SBF/PCAP file)
			if (g_read_from_sbf_log || g_read_from_pcap)
			{
				wait(time_obj);
			}
			publisher.publish(*msg);
			break;
		}

		case evIMUSetup: // IMU orientation and lever arm 
		{
			septentrio_gnss_driver::IMUSetupPtr msg =
				boost::make_shared<septentrio_gnss_driver::IMUSetup>();
			IMUSetup imusetup;
			memcpy(&imusetup, data_, sizeof(imusetup));
			msg = IMUSetupCallback(imusetup);
			msg->header.frame_id = g_frame_id;
			uint32_t tow = parsing_utilities::getTow(data_);
			uint16_t wnc = parsing_utilities::getWnc(data_);
			ros::Time time_obj;
			time_obj = timestampSBF(tow, wnc, g_use_gnss_time);
			msg->header.stamp.sec = time_obj.sec;
			msg->header.stamp.nsec = time_obj.nsec;
			msg->block_header.id = 4224;
			static ros::Publisher publisher = 
				pNh_->advertise<septentrio_gnss_driver::IMUSetup>("/imusetup", 
																g_ROS_QUEUE_SIZE);
			// Wait as long as necessary (only when reading from SBF/PCAP file)
			if (g_read_from_sbf_log || g_read_from_pcap)
			{
				wait(time_obj);
			}
			publisher.publish(*msg);
			break;
		}

		case evVelSensorSetup: // Velocity sensor lever arm
		{
			septentrio_gnss_driver::VelSensorSetupPtr msg =
				boost::make_shared<septentrio_gnss_driver::VelSensorSetup>();
			VelSensorSetup velsensorsetup;
			memcpy(&velsensorsetup, data_, sizeof(velsensorsetup));
			msg = VelSensorSetupCallback(velsensorsetup);
			msg->header.frame_id = g_frame_id;
			uint32_t tow = parsing_utilities::getTow(data_);
			uint16_t wnc = parsing_utilities::getWnc(data_);
			ros::Time time_obj;
			time_obj = timestampSBF(tow, wnc, g_use_gnss_time);
			msg->header.stamp.sec = time_obj.sec;
			msg->header.stamp.nsec = time_obj.nsec;
			msg->block_header.id = 4244;
			static ros::Publisher publisher = 
				pNh_->advertise<septentrio_gnss_driver::VelSensorSetup>("/velsensorsetup", 
																g_ROS_QUEUE_SIZE);
			// Wait as long as necessary (only when reading from SBF/PCAP file)
			if (g_read_from_sbf_log || g_read_from_pcap)
			{
				wait(time_obj);
			}
			publisher.publish(*msg);
			break;
		}

		case evExtEventINSNavCart: // Position, velocity and orientation in cartesian coordinate frame (ENU
							// frame)
		{
			septentrio_gnss_driver::ExtEventINSNavCartPtr msg =
				boost::make_shared<septentrio_gnss_driver::ExtEventINSNavCart>();
			ExtEventINSNavCart exteventinsnavcart;
			memcpy(&exteventinsnavcart, data_, sizeof(exteventinsnavcart));
			msg = ExtEventINSNavCartCallback(exteventinsnavcart);
			msg->header.frame_id = g_frame_id;
			uint32_t tow = parsing_utilities::getTow(data_);
			uint16_t wnc = parsing_utilities::getWnc(data_);
			ros::Time time_obj;
			time_obj = timestampSBF(tow, wnc, g_use_gnss_time);
			msg->header.stamp.sec = time_obj.sec;
			msg->header.stamp.nsec = time_obj.nsec;
			msg->block_header.id = 4229;
			static ros::Publisher publisher = 
				pNh_->advertise<septentrio_gnss_driver::ExtEventINSNavCart>(
					"/exteventinsnavcart", g_ROS_QUEUE_SIZE);
			// Wait as long as necessary (only when reading from SBF/PCAP file)
			if (g_read_from_sbf_log || g_read_from_pcap)
			{
				wait(time_obj);
			}
			publisher.publish(*msg);
			break;
		}

		case evExtEventINSNavGeod:
		{
			septentrio_gnss_driver::ExtEventINSNavGeodPtr msg =
				boost::make_shared<septentrio_gnss_driver::ExtEventINSNavGeod>();
			ExtEventINSNavGeod exteventinsnavgeod;
			memcpy(&exteventinsnavgeod, data_, sizeof(exteventinsnavgeod));
			msg = ExtEventINSNavGeodCallback(exteventinsnavgeod);
			msg->header.frame_id = g_frame_id;
			uint32_t tow = parsing_utilities::getTow(data_);
			uint16_t wnc = parsing_utilities::getWnc(data_);
			ros::Time time_obj;
			time_obj = timestampSBF(tow, wnc, g_use_gnss_time);
			msg->header.stamp.sec = time_obj.sec;
			msg->header.stamp.nsec = time_obj.nsec;
			msg->block_header.id = 4230;
			static ros::Publisher publisher =
				pNh_->advertise<septentrio_gnss_driver::ExtEventINSNavGeod>("/exteventinsnavgeod",
																	  g_ROS_QUEUE_SIZE);
			// Wait as long as necessary (only when reading from SBF/PCAP file)
			if (g_read_from_sbf_log || g_read_from_pcap)
			{
				wait(time_obj);
			}
			publisher.publish(*msg);
			break;
		}

		case evExtSensorMeas:
		{
			septentrio_gnss_driver::ExtSensorMeasPtr msg =
				boost::make_shared<septentrio_gnss_driver::ExtSensorMeas>();
			ExtSensorMeas extsensormeas;
			memcpy(&extsensormeas, data_, sizeof(extsensormeas));
			msg = ExtSensorMeasCallback(extsensormeas);
			msg->header.frame_id = g_frame_id;
			uint32_t tow = parsing_utilities::getTow(data_);
			uint16_t wnc = parsing_utilities::getWnc(data_);
			ros::Time time_obj;
			time_obj = timestampSBF(tow, wnc, g_use_gnss_time);
			msg->header.stamp.sec = time_obj.sec;
			msg->header.stamp.nsec = time_obj.nsec;
			msg->block_header.id = 4050;
			static ros::Publisher publisher =
				pNh_->advertise<septentrio_gnss_driver::ExtSensorMeas>("/extsensormeas",
																	  g_ROS_QUEUE_SIZE);
			// Wait as long as necessary (only when reading from SBF/PCAP file)
			if (g_read_from_sbf_log || g_read_from_pcap)
			{
				wait(time_obj);
			}
			publisher.publish(*msg);
			break;
		}

		case evGPST:
		{
			sensor_msgs::TimeReferencePtr msg =
				boost::make_shared<sensor_msgs::TimeReference>();
			uint32_t tow = parsing_utilities::getTow(data_);
			uint16_t wnc = parsing_utilities::getWnc(data_);
			ros::Time time_obj;
			time_obj = timestampSBF(tow, wnc, true); // We need the GPS time, hence true
			msg->time_ref.sec = time_obj.sec;
			msg->time_ref.nsec = time_obj.nsec;
			msg->source = "GPST";
			static ros::Publisher publisher =
				pNh_->advertise<sensor_msgs::TimeReference>("/gpst", g_ROS_QUEUE_SIZE);
			// Wait as long as necessary (only when reading from SBF/PCAP file)
			if (g_read_from_sbf_log || g_read_from_pcap)
			{
				wait(time_obj);
			}
			publisher.publish(*msg);
			break;
		}
		case evGPGGA:
		{
			boost::char_separator<char> sep("\r");
			typedef boost::tokenizer<boost::char_separator<char>> tokenizer;
			std::size_t nmea_size = this->messageSize();
			std::string block_in_string(reinterpret_cast<const char*>(data_), nmea_size);
			tokenizer tokens(block_in_string, sep);

			std::string id = this->messageID();
			std::string one_message = *tokens.begin();
			// No kept delimiters, hence "". Also, we specify that empty tokens should
			// show up in the output when two delimiters are next to each other. Hence we
			// also append the checksum part of the GGA message to "body" below, though
			// it is not parsed.
			boost::char_separator<char> sep_2(",*", "", boost::keep_empty_tokens);
			tokenizer tokens_2(one_message, sep_2);
			std::vector<std::string> body;
			for (tokenizer::iterator tok_iter = tokens_2.begin();
				tok_iter != tokens_2.end(); ++tok_iter)
			{
				body.push_back(*tok_iter);
			}
			// Create NmeaSentence struct to pass to GpggaParser::parseASCII
			NMEASentence gga_message(id, body);
			septentrio_gnss_driver::GpggaPtr msg =
				boost::make_shared<septentrio_gnss_driver::Gpgga>();
			GpggaParser parser_obj;
			try
			{
				msg = parser_obj.parseASCII(gga_message);
			} catch (ParseException& e)
			{
				throw std::runtime_error(e.what());
			}
			static ros::Publisher publisher =
				pNh_->advertise<septentrio_gnss_driver::Gpgga>("/gpgga",
															g_ROS_QUEUE_SIZE);
			// Wait as long as necessary (only when reading from SBF/PCAP file)
			if (g_read_from_sbf_log || g_read_from_pcap)
			{
				ros::Time time_obj(msg->header.stamp.sec, msg->header.stamp.nsec);
				wait(time_obj);
			}
			publisher.publish(*msg);
			break;
		}
		case evGPRMC:
		{
			boost::char_separator<char> sep("\r");
			typedef boost::tokenizer<boost::char_separator<char>> tokenizer;
			std::size_t nmea_size = this->messageSize();
			std::string block_in_string(reinterpret_cast<const char*>(data_), nmea_size);
			tokenizer tokens(block_in_string, sep);

			std::string id = this->messageID();
			std::string one_message = *tokens.begin();
			boost::char_separator<char> sep_2(",*", "", boost::keep_empty_tokens);
			tokenizer tokens_2(one_message, sep_2);
			std::vector<std::string> body;
			for (tokenizer::iterator tok_iter = tokens_2.begin();
				tok_iter != tokens_2.end(); ++tok_iter)
			{
				body.push_back(*tok_iter);
			}
			// Create NmeaSentence struct to pass to GprmcParser::parseASCII
			NMEASentence rmc_message(id, body);
			septentrio_gnss_driver::GprmcPtr msg =
				boost::make_shared<septentrio_gnss_driver::Gprmc>();
			GprmcParser parser_obj;
			try
			{
				msg = parser_obj.parseASCII(rmc_message);
			} catch (ParseException& e)
			{
				throw std::runtime_error(e.what());
			}
			static ros::Publisher publisher =
				pNh_->advertise<septentrio_gnss_driver::Gprmc>("/gprmc",
															g_ROS_QUEUE_SIZE);
			// Wait as long as necessary (only when reading from SBF/PCAP file)
			if (g_read_from_sbf_log || g_read_from_pcap)
			{
				ros::Time time_obj(msg->header.stamp.sec, msg->header.stamp.nsec);
				wait(time_obj);
			}
			publisher.publish(*msg);
			break;
		}
		case evGPGSA:
		{
			boost::char_separator<char> sep("\r");
			typedef boost::tokenizer<boost::char_separator<char>> tokenizer;
			std::size_t nmea_size = this->messageSize();
			std::string block_in_string(reinterpret_cast<const char*>(data_), nmea_size);
			tokenizer tokens(block_in_string, sep);

			std::string id = this->messageID();
			std::string one_message = *tokens.begin();
			boost::char_separator<char> sep_2(",*", "", boost::keep_empty_tokens);
			tokenizer tokens_2(one_message, sep_2);
			std::vector<std::string> body;
			for (tokenizer::iterator tok_iter = tokens_2.begin();
				tok_iter != tokens_2.end(); ++tok_iter)
			{
				body.push_back(*tok_iter);
			}
			// Create NmeaSentence struct to pass to GpgsaParser::parseASCII
			NMEASentence gsa_message(id, body);
			septentrio_gnss_driver::GpgsaPtr msg =
				boost::make_shared<septentrio_gnss_driver::Gpgsa>();
			GpgsaParser parser_obj;
			try
			{
				msg = parser_obj.parseASCII(gsa_message);
			} catch (ParseException& e)
			{
				throw std::runtime_error(e.what());
			}
			if (septentrio_receiver_type_ == "gnss")
			{
				ros::Time time_obj;
				time_obj = timestampSBF(last_pvtgeodetic_.tow, last_pvtgeodetic_.wnc, g_use_gnss_time);
				msg->header.stamp.sec = time_obj.sec;
				msg->header.stamp.nsec = time_obj.nsec;
			}
			if (septentrio_receiver_type_ == "ins")
			{
				ros::Time time_obj;
				time_obj = timestampSBF(last_insnavgeod_.tow, last_insnavgeod_.wnc, g_use_gnss_time);
				msg->header.stamp.sec = time_obj.sec;
				msg->header.stamp.nsec = time_obj.nsec;
			}
			static ros::Publisher publisher =
				pNh_->advertise<septentrio_gnss_driver::Gpgsa>("/gpgsa",
															g_ROS_QUEUE_SIZE);
			// Wait as long as necessary (only when reading from SBF/PCAP file)
			if (g_read_from_sbf_log || g_read_from_pcap)
			{
				ros::Time time_obj(msg->header.stamp.sec, msg->header.stamp.nsec);
				wait(time_obj);
			}
			publisher.publish(*msg);
			break;
		}
		case evGPGSV:
		case evGLGSV:
		case evGAGSV:
		{
			boost::char_separator<char> sep("\r");
			typedef boost::tokenizer<boost::char_separator<char>> tokenizer;
			std::size_t nmea_size = this->messageSize();
			std::string block_in_string(reinterpret_cast<const char*>(data_), nmea_size);
			tokenizer tokens(block_in_string, sep);

			std::string id = this->messageID();
			std::string one_message = *tokens.begin();
			boost::char_separator<char> sep_2(",*", "", boost::keep_empty_tokens);
			tokenizer tokens_2(one_message, sep_2);
			std::vector<std::string> body;
			for (tokenizer::iterator tok_iter = tokens_2.begin();
				tok_iter != tokens_2.end(); ++tok_iter)
			{
				body.push_back(*tok_iter);
			}
			// Create NmeaSentence struct to pass to GpgsvParser::parseASCII
			NMEASentence gsv_message(id, body);
			septentrio_gnss_driver::GpgsvPtr msg =
				boost::make_shared<septentrio_gnss_driver::Gpgsv>();
			GpgsvParser parser_obj;
			try
			{
				msg = parser_obj.parseASCII(gsv_message);
			} catch (ParseException& e)
			{
				throw std::runtime_error(e.what());
			}
			if (septentrio_receiver_type_ == "gnss")
			{
				ros::Time time_obj;
				time_obj = timestampSBF(last_pvtgeodetic_.tow, last_pvtgeodetic_.wnc, g_use_gnss_time);
				msg->header.stamp.sec = time_obj.sec;
				msg->header.stamp.nsec = time_obj.nsec;
			}
			if (septentrio_receiver_type_ == "ins")
			{
				ros::Time time_obj;
				time_obj = timestampSBF(last_insnavgeod_.tow, last_insnavgeod_.wnc, g_use_gnss_time);
				msg->header.stamp.sec = time_obj.sec;
				msg->header.stamp.nsec = time_obj.nsec;
			}
			static ros::Publisher publisher =
				pNh_->advertise<septentrio_gnss_driver::Gpgsv>("/gpgsv",
															g_ROS_QUEUE_SIZE);
			// Wait as long as necessary (only when reading from SBF/PCAP file)
			if (g_read_from_sbf_log || g_read_from_pcap)
			{
				ros::Time time_obj(msg->header.stamp.sec, msg->header.stamp.nsec);
				wait(time_obj);
			}
			publisher.publish(*msg);
			break;
		}
		
		if (septentrio_receiver_type_ == "gnss")
		{
			case evNavSatFix:
			{
				sensor_msgs::NavSatFixPtr msg = boost::make_shared<sensor_msgs::NavSatFix>();
				try
				{
					msg = NavSatFixCallback();
				} catch (std::runtime_error& e)
				{
					throw std::runtime_error(e.what());
				}
				msg->header.frame_id = g_frame_id;
				uint32_t tow = parsing_utilities::getTow(data_);
				uint16_t wnc = parsing_utilities::getWnc(data_);
				ros::Time time_obj;
				time_obj = timestampSBF(tow, wnc, g_use_gnss_time);
				msg->header.stamp.sec = time_obj.sec;
				msg->header.stamp.nsec = time_obj.nsec;
				pvtgeodetic_has_arrived_navsatfix_ = false;
				poscovgeodetic_has_arrived_navsatfix_ = false;
				static ros::Publisher publisher =
					pNh_->advertise<sensor_msgs::NavSatFix>("/navsatfix", g_ROS_QUEUE_SIZE);
				// Wait as long as necessary (only when reading from SBF/PCAP file)
				if (g_read_from_sbf_log || g_read_from_pcap)
				{
					wait(time_obj);
				}
				publisher.publish(*msg);
				break;
			}
		}
		if (septentrio_receiver_type_ == "ins")
		{
			case evINSNavSatFix:
			{
				sensor_msgs::NavSatFixPtr msg = boost::make_shared<sensor_msgs::NavSatFix>();
				try
				{
					msg = NavSatFixCallback();
				} catch (std::runtime_error& e)
				{
					throw std::runtime_error(e.what());
				}
				msg->header.frame_id = g_frame_id;
				uint32_t tow = parsing_utilities::getTow(data_);
				uint16_t wnc = parsing_utilities::getWnc(data_);
				ros::Time time_obj;
				time_obj = timestampSBF(tow, wnc, g_use_gnss_time);
				msg->header.stamp.sec = time_obj.sec;
				msg->header.stamp.nsec = time_obj.nsec;
				insnavgeod_has_arrived_navsatfix_ = false;
				static ros::Publisher publisher =
					pNh_->advertise<sensor_msgs::NavSatFix>("/navsatfix", g_ROS_QUEUE_SIZE);
				// Wait as long as necessary (only when reading from SBF/PCAP file)
				if (g_read_from_sbf_log || g_read_from_pcap)
				{
					wait(time_obj);
				}
				publisher.publish(*msg);
				break;
			}
		}

		if (septentrio_receiver_type_ == "gnss")
		{
			case evGPSFix:
			{
				gps_common::GPSFixPtr msg = boost::make_shared<gps_common::GPSFix>();
				try
				{
					msg = GPSFixCallback();
				} catch (std::runtime_error& e)
				{
					throw std::runtime_error(e.what());
				}
				msg->status.header.seq = count_gpsfix_;
				msg->header.frame_id = g_frame_id;
				msg->status.header.frame_id = g_frame_id;
				uint32_t tow = parsing_utilities::getTow(data_);
				uint16_t wnc = parsing_utilities::getWnc(data_);
				ros::Time time_obj;
				time_obj = timestampSBF(tow, wnc, g_use_gnss_time);
				msg->header.stamp.sec = time_obj.sec;
				msg->status.header.stamp.sec = time_obj.sec;
				msg->header.stamp.nsec = time_obj.nsec;
				msg->status.header.stamp.nsec = time_obj.nsec;
				++count_gpsfix_;
				channelstatus_has_arrived_gpsfix_ = false;
				measepoch_has_arrived_gpsfix_ = false;
				dop_has_arrived_gpsfix_ = false;
				pvtgeodetic_has_arrived_gpsfix_ = false;
				poscovgeodetic_has_arrived_gpsfix_ = false;
				velcovgeodetic_has_arrived_gpsfix_ = false;
				atteuler_has_arrived_gpsfix_ = false;
				attcoveuler_has_arrived_gpsfix_ = false;
				static ros::Publisher publisher =
					pNh_->advertise<gps_common::GPSFix>("/gpsfix", g_ROS_QUEUE_SIZE);
				// Wait as long as necessary (only when reading from SBF/PCAP file)
				if (g_read_from_sbf_log || g_read_from_pcap)
				{
					wait(time_obj);
				}
				publisher.publish(*msg);
				break;
			}
		}
		if (septentrio_receiver_type_ == "ins")
		{
			case evINSGPSFix:
			{
				gps_common::GPSFixPtr msg = boost::make_shared<gps_common::GPSFix>();
				try
				{
					msg = GPSFixCallback();
				} catch (std::runtime_error& e)
				{
					throw std::runtime_error(e.what());
				}
				msg->status.header.seq = count_gpsfix_;
				msg->header.frame_id = g_frame_id;
				msg->status.header.frame_id = g_frame_id;
				uint32_t tow = parsing_utilities::getTow(data_);
				uint16_t wnc = parsing_utilities::getWnc(data_);
				ros::Time time_obj;
				time_obj = timestampSBF(tow, wnc, g_use_gnss_time);
				msg->header.stamp.sec = time_obj.sec;
				msg->status.header.stamp.sec = time_obj.sec;
				msg->header.stamp.nsec = time_obj.nsec;
				msg->status.header.stamp.nsec = time_obj.nsec;
				++count_gpsfix_;
				channelstatus_has_arrived_gpsfix_ = false;
				measepoch_has_arrived_gpsfix_ = false;
				dop_has_arrived_gpsfix_ = false;
				insnavgeod_has_arrived_gpsfix_ = false;
				static ros::Publisher publisher =
					pNh_->advertise<gps_common::GPSFix>("/gpsfix", g_ROS_QUEUE_SIZE);
				// Wait as long as necessary (only when reading from SBF/PCAP file)
				if (g_read_from_sbf_log || g_read_from_pcap)
				{
					wait(time_obj);
				}
				publisher.publish(*msg);
				break;
			}
		}
		if (septentrio_receiver_type_ == "gnss")
		{
			case evPoseWithCovarianceStamped:
			{
				geometry_msgs::PoseWithCovarianceStampedPtr msg =
					boost::make_shared<geometry_msgs::PoseWithCovarianceStamped>();
				try
				{
					msg = PoseWithCovarianceStampedCallback();
				} catch (std::runtime_error& e)
				{
					throw std::runtime_error(e.what());
				}
				msg->header.frame_id = g_frame_id;
				uint32_t tow = parsing_utilities::getTow(data_);
				uint16_t wnc = parsing_utilities::getWnc(data_);
				ros::Time time_obj;
				time_obj = timestampSBF(tow, wnc, g_use_gnss_time);
				msg->header.stamp.sec = time_obj.sec;
				msg->header.stamp.nsec = time_obj.nsec;
				pvtgeodetic_has_arrived_pose_ = false;
				poscovgeodetic_has_arrived_pose_ = false;
				atteuler_has_arrived_pose_ = false;
				attcoveuler_has_arrived_pose_ = false;
				static ros::Publisher publisher =
					pNh_->advertise<geometry_msgs::PoseWithCovarianceStamped>(
						"/pose", g_ROS_QUEUE_SIZE);
				// Wait as long as necessary (only when reading from SBF/PCAP file)
				if (g_read_from_sbf_log || g_read_from_pcap)
				{
					wait(time_obj);
				}
				publisher.publish(*msg);
				break;
			}
		}
		if (septentrio_receiver_type_ == "ins")
		{
			case evINSPoseWithCovarianceStamped:
			{
				geometry_msgs::PoseWithCovarianceStampedPtr msg =
					boost::make_shared<geometry_msgs::PoseWithCovarianceStamped>();
				try
				{
					msg = PoseWithCovarianceStampedCallback();
				} catch (std::runtime_error& e)
				{
					throw std::runtime_error(e.what());
				}
				msg->header.frame_id = g_frame_id;
				uint32_t tow = parsing_utilities::getTow(data_);
				uint16_t wnc = parsing_utilities::getWnc(data_);
				ros::Time time_obj;
				time_obj = timestampSBF(tow, wnc, g_use_gnss_time);
				msg->header.stamp.sec = time_obj.sec;
				msg->header.stamp.nsec = time_obj.nsec;
				insnavgeod_has_arrived_pose_ = false;
				static ros::Publisher publisher =
					pNh_->advertise<geometry_msgs::PoseWithCovarianceStamped>(
						"/pose", g_ROS_QUEUE_SIZE);
				// Wait as long as necessary (only when reading from SBF/PCAP file)
				if (g_read_from_sbf_log || g_read_from_pcap)
				{
					wait(time_obj);
				}
				publisher.publish(*msg);
				break;
			}
			
		}
		case evChannelStatus:
		{
			memcpy(&last_channelstatus_, data_, sizeof(last_channelstatus_));
			channelstatus_has_arrived_gpsfix_ = true;
			break;
		}
		case evMeasEpoch:
		{
			memcpy(&last_measepoch_, data_, sizeof(last_measepoch_));
			measepoch_has_arrived_gpsfix_ = true;
			break;
		}
		case evDOP:
		{
			memcpy(&last_dop_, data_, sizeof(last_dop_));
			dop_has_arrived_gpsfix_ = true;
			break;
		}
		case evVelCovGeodetic:
		{
			septentrio_gnss_driver::VelCovGeodeticPtr msg =
				boost::make_shared<septentrio_gnss_driver::VelCovGeodetic>();
			memcpy(&last_velcovgeodetic_, data_, sizeof(last_velcovgeodetic_));
			msg = VelCovGeodeticCallback(last_velcovgeodetic_);
			msg->header.frame_id = g_frame_id;
			uint32_t tow = parsing_utilities::getTow(data_);
			uint16_t wnc = parsing_utilities::getWnc(data_);
			ros::Time time_obj;
			time_obj = timestampSBF(tow, wnc, g_use_gnss_time);
			msg->header.stamp.sec = time_obj.sec;
			msg->header.stamp.nsec = time_obj.nsec;
			msg->block_header.id = 5908;
			velcovgeodetic_has_arrived_gpsfix_ = true;
			static ros::Publisher publisher =
				pNh_->advertise<septentrio_gnss_driver::VelCovGeodetic>(
					"/velcovgeodetic", g_ROS_QUEUE_SIZE);
			// Wait as long as necessary (only when reading from SBF/PCAP file)
			if (g_read_from_sbf_log || g_read_from_pcap)
			{
				wait(time_obj);
			}
			publisher.publish(*msg);
			break;
		}
		case evDiagnosticArray:
		{
			diagnostic_msgs::DiagnosticArrayPtr msg =
			boost::make_shared<diagnostic_msgs::DiagnosticArray>();
			try
			{
				msg = DiagnosticArrayCallback();
			} catch (std::runtime_error& e)
			{
				throw std::runtime_error(e.what());
			}
			if (septentrio_receiver_type_ == "gnss")
			{
				msg->header.frame_id = g_frame_id;
			}
			if (septentrio_receiver_type_ == "ins")
			{
				msg->header.frame_id = g_frame_id;
			}
			uint32_t tow = parsing_utilities::getTow(data_);
			uint16_t wnc = parsing_utilities::getWnc(data_);
			ros::Time time_obj;
			time_obj = timestampSBF(tow, wnc, g_use_gnss_time);
			msg->header.stamp.sec = time_obj.sec;
			msg->header.stamp.nsec = time_obj.nsec;
			receiverstatus_has_arrived_diagnostics_ = false;
			qualityind_has_arrived_diagnostics_ = false;
			static ros::Publisher publisher =
				pNh_->advertise<diagnostic_msgs::DiagnosticArray>("/diagnostics",
																g_ROS_QUEUE_SIZE);
			// Wait as long as necessary (only when reading from SBF/PCAP file)
			if (g_read_from_sbf_log || g_read_from_pcap)
			{
				wait(time_obj);
			}
			publisher.publish(*msg);
			break; 
		}
		case evReceiverStatus:
		{
			memcpy(&last_receiverstatus_, data_, sizeof(last_receiverstatus_));
			receiverstatus_has_arrived_diagnostics_ = true;
			break;
		}
		case evQualityInd:
		{
			memcpy(&last_qualityind_, data_, sizeof(last_qualityind_));
			qualityind_has_arrived_diagnostics_ = true;
			break;
		}
		case evReceiverSetup:
		{
			memcpy(&last_receiversetup_, data_, sizeof(last_receiversetup_));
			break;
		}
		
		// Many more to be implemented...
    }
    return true;
}

void io_comm_rx::RxMessage::wait(const ros::Time& time_obj)
{
	ros::Time unix_old = unix_time_;
	unix_time_ = time_obj;
	if (!(unix_old.sec == 0 && unix_old.nsec == 0) &&
		(unix_time_.sec != unix_old.sec ||
			unix_time_.nsec != unix_old.nsec))
	{
		std::stringstream ss;
		ss << "Waiting for " << unix_time_.sec - unix_old.sec
			<< " seconds and "
			<< abs(int((unix_time_.nsec - unix_old.nsec) / 1000))
			<< " microseconds";
		ROS_DEBUG("%s", ss.str().c_str());
		sleep((unsigned int)(unix_time_.sec - unix_old.sec));
		usleep(static_cast<uint32_t>(
			abs(int((unix_time_.nsec - unix_old.nsec) / 1000))));
	}
}

bool io_comm_rx::RxMessage::gnss_gpsfix_complete(uint32_t id)
{
	std::vector<bool> gpsfix_vec = {
	channelstatus_has_arrived_gpsfix_,
	measepoch_has_arrived_gpsfix_,
	dop_has_arrived_gpsfix_,
	pvtgeodetic_has_arrived_gpsfix_,
	poscovgeodetic_has_arrived_gpsfix_,
	velcovgeodetic_has_arrived_gpsfix_,
	atteuler_has_arrived_gpsfix_,
	attcoveuler_has_arrived_gpsfix_};
	return allTrue(gpsfix_vec, id);
}

bool io_comm_rx::RxMessage::ins_gpsfix_complete(uint32_t id)
{
	std::vector<bool> gpsfix_vec = {
	channelstatus_has_arrived_gpsfix_,
	measepoch_has_arrived_gpsfix_,
	dop_has_arrived_gpsfix_,
	insnavgeod_has_arrived_gpsfix_};
	return allTrue(gpsfix_vec, id);
}

bool io_comm_rx::RxMessage::gnss_navsatfix_complete(uint32_t id)
{
	std::vector<bool> navsatfix_vec = {
		pvtgeodetic_has_arrived_navsatfix_,
		poscovgeodetic_has_arrived_navsatfix_};
	return allTrue(navsatfix_vec, id);
}

bool io_comm_rx::RxMessage::ins_navsatfix_complete(uint32_t id)
{
	std::vector<bool> navsatfix_vec = {
	insnavgeod_has_arrived_navsatfix_};
	return allTrue(navsatfix_vec, id);
}

bool io_comm_rx::RxMessage::gnss_pose_complete(uint32_t id)
{
	std::vector<bool> pose_vec = {pvtgeodetic_has_arrived_pose_,
								poscovgeodetic_has_arrived_pose_,
								atteuler_has_arrived_pose_,
								attcoveuler_has_arrived_pose_};
	return allTrue(pose_vec, id);
}

bool io_comm_rx::RxMessage::ins_pose_complete(uint32_t id)
{
	 std::vector<bool> pose_vec = {insnavgeod_has_arrived_pose_};
	return allTrue(pose_vec, id);
}

bool io_comm_rx::RxMessage::diagnostics_complete(uint32_t id)
{
	std::vector<bool> diagnostics_vec = {
		receiverstatus_has_arrived_diagnostics_,
		qualityind_has_arrived_diagnostics_};
	return allTrue(diagnostics_vec, id);
}

bool io_comm_rx::RxMessage::allTrue(std::vector<bool>& vec, uint32_t id)
{
	vec.erase(vec.begin() + id);
	// Checks whether all entries in vec are true
	return (std::all_of(vec.begin(), vec.end(),
						[](bool v) { return v; }) == true);
}