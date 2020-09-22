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

#include <rosaic/communication/mosaicMessage.hpp>

/**
 * @file mosaicMessage.cpp
 * @date 20/08/20
 * @brief Defines a class that can deal with a buffer of size bytes_transferred that is handed over from async_read_some
 */
 
//! Number of times the "read" method of the mosaicMessage class has been called
uint32_t io_comm_mosaic::mosaicMessage::count_pvtgeodetic_ = 0;
uint32_t io_comm_mosaic::mosaicMessage::count_pvtcartesian_ = 0;
uint32_t io_comm_mosaic::mosaicMessage::count_poscovgeodetic_ = 0;
uint32_t io_comm_mosaic::mosaicMessage::count_atteuler_ = 0;
uint32_t io_comm_mosaic::mosaicMessage::count_attcoveuler_ = 0;
uint32_t io_comm_mosaic::mosaicMessage::count_navsatfix_ = 0;
uint32_t io_comm_mosaic::mosaicMessage::count_gpsfix_ = 0;
uint32_t io_comm_mosaic::mosaicMessage::count_gpgga_ = 0;
PVTGeodetic io_comm_mosaic::mosaicMessage::last_pvtgeodetic_ = PVTGeodetic();
PosCovGeodetic io_comm_mosaic::mosaicMessage::last_poscovgeodetic_ = PosCovGeodetic();
AttEuler io_comm_mosaic::mosaicMessage::last_atteuler_ = AttEuler();
AttCovEuler io_comm_mosaic::mosaicMessage::last_attcoveuler_ = AttCovEuler();
ChannelStatus io_comm_mosaic::mosaicMessage::last_channelstatus_ = ChannelStatus();
MeasEpoch io_comm_mosaic::mosaicMessage::last_measepoch_ = MeasEpoch();
DOP io_comm_mosaic::mosaicMessage::last_dop_ = DOP();
VelCovGeodetic io_comm_mosaic::mosaicMessage::last_velcovgeodetic_ = VelCovGeodetic();

rosaic::PVTGeodeticPtr io_comm_mosaic::mosaicMessage::PVTGeodeticCallback(PVTGeodetic& data)
{
	rosaic::PVTGeodeticPtr msg = boost::make_shared<rosaic::PVTGeodetic>();
	msg->Block_Header.SYNC1 = data.Block_Header.SYNC1;
	msg->Block_Header.SYNC2 = data.Block_Header.SYNC2;
	msg->Block_Header.CRC = data.Block_Header.CRC;
	msg->Block_Header.ID = data.Block_Header.ID;
	msg->Block_Header.Length = data.Block_Header.Length;
	msg->Block_Header.TOW = data.TOW;
	msg->Block_Header.WNc = data.WNc;
	msg->Mode = data.Mode;
	msg->Error = data.Error;
	msg->Latitude = data.Latitude;
	msg->Longitude = data.Longitude;
	msg->Height = data.Height;
	msg->Undulation = data.Undulation;
	msg->Vn = data.Vn;
	msg->Ve = data.Ve;
	msg->Vu = data.Vu;
	msg->COG = data.COG;
	msg->RxClkBias = data.RxClkBias;
	msg->RxClkDrift = data.RxClkDrift;
	msg->TimeSystem = data.TimeSystem;
	msg->Datum = data.Datum;
	msg->NrSV = data.NrSV;
	msg->WACorrInfo = data.WACorrInfo;
	msg->ReferenceID = data.ReferenceID;
	msg->MeanCorrAge = data.MeanCorrAge;
	msg->SignalInfo = data.SignalInfo;
	msg->AlertFlag = data.AlertFlag;
	msg->NrBases = data.NrBases;
	msg->PPPInfo = data.PPPInfo;
	msg->Latency = data.Latency;
	msg->HAccuracy = data.HAccuracy;
	msg->VAccuracy = data.VAccuracy;
	msg->Misc = data.Misc;
	return msg;
}


rosaic::PVTCartesianPtr io_comm_mosaic::mosaicMessage::PVTCartesianCallback(PVTCartesian& data)
{
	rosaic::PVTCartesianPtr msg = boost::make_shared<rosaic::PVTCartesian>();
	msg->Block_Header.SYNC1 = data.Block_Header.SYNC1;
	msg->Block_Header.SYNC2 = data.Block_Header.SYNC2;
	msg->Block_Header.CRC = data.Block_Header.CRC;
	msg->Block_Header.ID = data.Block_Header.ID;
	msg->Block_Header.Length = data.Block_Header.Length;
	msg->Block_Header.TOW = data.TOW;
	msg->Block_Header.WNc = data.WNc;
	msg->Mode = data.Mode;
	msg->Error = data.Error;
	msg->X = data.X;
	msg->Y = data.Y;
	msg->Z = data.Z;
	msg->Undulation = data.Undulation;
	msg->Vx = data.Vx;
	msg->Vy = data.Vy;
	msg->Vz = data.Vz;
	msg->COG = data.COG;
	msg->RxClkBias = data.RxClkBias;
	msg->RxClkDrift = data.RxClkDrift;
	msg->TimeSystem = data.TimeSystem;
	msg->Datum = data.Datum;
	msg->NrSV = data.NrSV;
	msg->WACorrInfo = data.WACorrInfo;
	msg->ReferenceID = data.ReferenceID;
	msg->MeanCorrAge = data.MeanCorrAge;
	msg->SignalInfo = data.SignalInfo;
	msg->AlertFlag = data.AlertFlag;
	msg->NrBases = data.NrBases;
	msg->PPPInfo = data.PPPInfo;
	msg->Latency = data.Latency;
	msg->HAccuracy = data.HAccuracy;
	msg->VAccuracy = data.VAccuracy;
	msg->Misc = data.Misc;
	return msg;
}

rosaic::PosCovGeodeticPtr io_comm_mosaic::mosaicMessage::PosCovGeodeticCallback(PosCovGeodetic& data)
{
	rosaic::PosCovGeodeticPtr msg = boost::make_shared<rosaic::PosCovGeodetic>();
	msg->Block_Header.SYNC1 = data.Block_Header.SYNC1;
	msg->Block_Header.SYNC2 = data.Block_Header.SYNC2;
	msg->Block_Header.CRC = data.Block_Header.CRC;
	msg->Block_Header.ID = data.Block_Header.ID;
	msg->Block_Header.Length = data.Block_Header.Length;
	msg->Block_Header.TOW = data.TOW;
	msg->Block_Header.WNc = data.WNc;
	msg->Mode = data.Mode;
	msg->Error = data.Error;
	msg->Cov_latlat = data.Cov_latlat;
	msg->Cov_lonlon = data.Cov_lonlon;
	msg->Cov_hgthgt = data.Cov_hgthgt;
	msg->Cov_bb = data.Cov_bb;
	msg->Cov_latlon = data.Cov_latlon;
	msg->Cov_lathgt = data.Cov_lathgt;
	msg->Cov_latb = data.Cov_latb;
	msg->Cov_lonhgt = data.Cov_lonhgt;
	msg->Cov_lonb = data.Cov_lonb;
	msg->Cov_hb = data.Cov_hb;
	return msg;
}

rosaic::AttEulerPtr io_comm_mosaic::mosaicMessage::AttEulerCallback(AttEuler& data)
{
	rosaic::AttEulerPtr msg = boost::make_shared<rosaic::AttEuler>();
	msg->Block_Header.SYNC1 = data.Block_Header.SYNC1;
	msg->Block_Header.SYNC2 = data.Block_Header.SYNC2;
	msg->Block_Header.CRC = data.Block_Header.CRC;
	msg->Block_Header.ID = data.Block_Header.ID;
	msg->Block_Header.Length = data.Block_Header.Length;
	msg->Block_Header.TOW = data.TOW;
	msg->Block_Header.WNc = data.WNc;
	msg->NrSV = data.NrSV;
	msg->Error = data.Error;
	msg->Mode = data.Mode;
	msg->Heading = data.Heading;
	msg->Pitch = data.Pitch;
	msg->Roll = data.Roll;
	msg->PitchDot = data.PitchDot;
	msg->RollDot = data.RollDot;
	msg->HeadingDot = data.HeadingDot;
	return msg;
};

rosaic::AttCovEulerPtr io_comm_mosaic::mosaicMessage::AttCovEulerCallback(AttCovEuler& data)
{
	rosaic::AttCovEulerPtr msg = boost::make_shared<rosaic::AttCovEuler>();
	msg->Block_Header.SYNC1 = data.Block_Header.SYNC1;
	msg->Block_Header.SYNC2 = data.Block_Header.SYNC2;
	msg->Block_Header.CRC = data.Block_Header.CRC;
	msg->Block_Header.ID = data.Block_Header.ID;
	msg->Block_Header.Length = data.Block_Header.Length;
	msg->Block_Header.TOW = data.TOW;
	msg->Block_Header.WNc = data.WNc;
	msg->Error = data.Error;
	msg->Cov_HeadHead = data.Cov_HeadHead;
	msg->Cov_PitchPitch = data.Cov_PitchPitch;
	msg->Cov_RollRoll = data.Cov_RollRoll;
	msg->Cov_HeadPitch = data.Cov_HeadPitch;
	msg->Cov_HeadRoll = data.Cov_HeadRoll;
	msg->Cov_PitchRoll = data.Cov_PitchRoll;
	return msg;
};

/**
 * The position_covariance array is populated in row-major order, where the basis of the corresponding matrix is ENU (so Cov_lonlon is in location 11 of the matrix).
 * The B2b signal type of BeiDou is not checked for usage, since the SignalInfo field of the PVTGeodetic block does not disclose it. For that, one would need to
 * go to the ObsInfo field of the MeasEpochChannelType1 sub-block.
 */
sensor_msgs::NavSatFixPtr io_comm_mosaic::mosaicMessage::NavSatFixCallback()
{
	sensor_msgs::NavSatFixPtr msg = boost::make_shared<sensor_msgs::NavSatFix>();
	uint16_t mask = 15; // We extract the first four bits using this mask.
	uint16_t type_of_pvt = ((uint16_t) (last_pvtgeodetic_.Mode)) & mask;
	switch(type_of_pvt)
	{
		case 0:
		{
			msg->status.status = sensor_msgs::NavSatStatus::STATUS_NO_FIX;
			break;
		}
		case 1: case 3:
		{
			msg->status.status = sensor_msgs::NavSatStatus::STATUS_FIX;
			break;
		}
		case 2: case 4: case 5: case 7: case 8: case 10:
		{
			msg->status.status = sensor_msgs::NavSatStatus::STATUS_GBAS_FIX;
			break;
		}
		case 6:
		{
			msg->status.status = sensor_msgs::NavSatStatus::STATUS_SBAS_FIX;
			break;
		}
		default:
		{
			throw std::runtime_error("PVTGeodetic's Mode field contains an invalid type of PVT solution.");
		}
	}
	bool gps_in_pvt = false;
	bool glo_in_pvt = false;
	bool com_in_pvt = false;
	bool gal_in_pvt = false;
	uint32_t mask_2 = 1;
	for(int bit = 0; bit != 31; ++bit)
	{
		bool in_use = last_pvtgeodetic_.SignalInfo & mask_2;
		if (bit <= 5 && in_use) 
		{
			gps_in_pvt = true;
		}
		if (8   <= bit && bit <= 12 && in_use) glo_in_pvt = true;
		if (((13 <= bit && bit <= 14) || (28 <= bit && bit <= 30)) && in_use) com_in_pvt = true;
		if ((bit == 17 || (19 <= bit && bit <= 22)) && in_use) gal_in_pvt = true;
		mask_2 *= 2;
	}
	//ROS_DEBUG("GPS is in use: %s, GLO is in use: %s, COM is in use: %s, GAL is in use: %s", gps_in_pvt ? "true" : "false", glo_in_pvt ? "true" : "false", com_in_pvt ? "true" : "false", gal_in_pvt ? "true" : "false");
	uint16_t service = gps_in_pvt*1+glo_in_pvt*2+com_in_pvt*4+gal_in_pvt*8; // booleans will be promoted to integers automatically
	msg->status.service = service;
	msg->latitude = last_pvtgeodetic_.Latitude*360/(2*boost::math::constants::pi<double>());
	msg->longitude = last_pvtgeodetic_.Longitude*360/(2*boost::math::constants::pi<double>());
	msg->altitude = last_pvtgeodetic_.Height;
	msg->position_covariance[0] = last_poscovgeodetic_.Cov_lonlon;
	msg->position_covariance[1] = last_poscovgeodetic_.Cov_latlon;
	msg->position_covariance[2] = last_poscovgeodetic_.Cov_lonhgt;
	msg->position_covariance[3] = last_poscovgeodetic_.Cov_latlon;
	msg->position_covariance[4] = last_poscovgeodetic_.Cov_latlat;
	msg->position_covariance[5] = last_poscovgeodetic_.Cov_lathgt;
	msg->position_covariance[6] = last_poscovgeodetic_.Cov_lonhgt;
	msg->position_covariance[7] = last_poscovgeodetic_.Cov_lathgt;
	msg->position_covariance[8] = last_poscovgeodetic_.Cov_hgthgt;
	msg->position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_KNOWN;
	return msg;
}

/**
 * For some unknown reason, the first 2 entries of the GPSStatus field's arrays are not shown properly when published. 
 * Please consult section 4.1.9 of the firmware to understand the meaning of the satellite identifiers used in the arrays of the GPSStatus field.
 * Note that the field "dip" denotes the local magnetic inclination in degrees (positive when the magnetic field points downwards (into the Earth)). 
 * This quantity cannot be calculated by mosaic.
 * We assume that for the ROS field "err_time", we are requested to provide the 2 sigma uncertainty on the clock bias estimate in square meters, 
 * not the clock drift estimate (latter would be "2*std::sqrt(static_cast<double>(last_velcovgeodetic_.Cov_DtDt))").
 * The "err_track" entry is calculated via the Gaussian error propagation formula. For its usage we have to assume that the eastward and 
 * the northward velocities are independent variables.
 * Note that elevations and azimuths of visible satellites are taken from the ChannelStatus block, which provides 1 degree precision, while the 
 * SatVisibility block could provide hundredths of degrees precision. Change if imperative for your application..
 * Definition of "visible satellite" adopted: Here, we define a visible satellite as being up to "in sync" mode with the receiver, which corresponds 
 * to last_measepoch_.N, though not last_channelstatus_.N (also includes those "in search").
 */
gps_common::GPSFixPtr io_comm_mosaic::mosaicMessage::GPSFixCallback()
{
	gps_common::GPSFixPtr msg = boost::make_shared<gps_common::GPSFix>();
	
	msg->status.satellites_used = static_cast<uint16_t>(last_pvtgeodetic_.NrSV);
	
	// MeasEpoch Processing
	std::vector<int32_t> cno_tracked;
	std::vector<int32_t> svid_in_sync;
	{
		uint8_t sb1_size = last_measepoch_.SB1Size;
		uint8_t sb2_size = last_measepoch_.SB2Size;
		uint8_t *sb_start = &last_measepoch_.Data[0];
		int32_t index = sb_start - &last_measepoch_.Block_Header.SYNC1;
		for (int32_t i = 0; i < static_cast<int32_t>(last_measepoch_.N); ++i)
		{
			// Define MeasEpochChannelType1 struct for the corresponding sub-block
			MeasEpochChannelType1 *measepoch_channel_type1  = reinterpret_cast<MeasEpochChannelType1*>(&last_measepoch_.Block_Header.SYNC1 + index);
			svid_in_sync.push_back(static_cast<int32_t>(measepoch_channel_type1->SVID));
			uint8_t type_mask = 15; // We extract the first four bits using this mask.
			if (measepoch_channel_type1->Type & type_mask == static_cast<uint8_t>(1) || measepoch_channel_type1->Type & type_mask == static_cast<uint8_t>(2))
			{
				cno_tracked.push_back(static_cast<int32_t>(measepoch_channel_type1->CN0)/4);
			}
			else
			{
				cno_tracked.push_back(static_cast<int32_t>(measepoch_channel_type1->CN0)/4+static_cast<int32_t>(10));
			}
			//ROS_DEBUG("static_cast<int32_t>(measepoch_channel_type1->SVID)) is %i", static_cast<int32_t>(measepoch_channel_type1->SVID));
			index += sb1_size;
			for (int32_t j = 0; j < static_cast<int32_t>(measepoch_channel_type1->N_Type2); j++)
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
	std::vector<int32_t> ordering;
	{
		uint8_t sb1_size = last_channelstatus_.SB1Size;
		uint8_t sb2_size = last_channelstatus_.SB2Size;
		uint8_t *sb_start = &last_channelstatus_.Data[0];
		int32_t index = sb_start - &last_channelstatus_.Block_Header.SYNC1; //What about endianness?
		//ROS_DEBUG("index is %i", index); // yields 20, as expected
		
		uint16_t azimuth_mask = 511;
		for (int32_t i = 0; i < static_cast<int32_t>(last_channelstatus_.N); i++)
		{
			// Define ChannelSatInfo struct for the corresponding sub-block
			ChannelSatInfo *channel_sat_info  = reinterpret_cast<ChannelSatInfo*>(&last_channelstatus_.Block_Header.SYNC1 + index);
			bool to_be_added = false;
			for (int32_t j = 0; j < static_cast<int32_t>(svid_in_sync.size()); ++j)
			{
				if (svid_in_sync[j] == static_cast<int32_t>(channel_sat_info->SVID))
				{
					ordering.push_back(j);
					to_be_added = true;
					break;
				}
			}
			if (to_be_added)
			{
				svid_in_sync_2.push_back(static_cast<int32_t>(channel_sat_info->SVID));
				elevation_tracked.push_back(static_cast<int32_t>(channel_sat_info->Elev));
				azimuth_tracked.push_back(static_cast<int32_t>((channel_sat_info->Az_RiseSet & azimuth_mask)));
			}
			index += sb1_size;
			for (int32_t j = 0; j < static_cast<int32_t>(channel_sat_info->N2); j++)
			{
				// Define ChannelStateInfo struct for the corresponding sub-block
				ChannelStateInfo *channel_state_info  = reinterpret_cast<ChannelStateInfo*>(&last_channelstatus_.Block_Header.SYNC1 + index);
				bool pvt_status = false;
				uint16_t pvt_status_mask = std::pow(2,15)+std::pow(2,14);
				for(int k = 15; k != -1; k -= 2)
				{
					uint16_t pvt_status_value = (channel_state_info->PVTStatus & pvt_status_mask) >> k-1;
					//ROS_DEBUG("value is %u and channel_state_info->PVTStatus is %u", pvt_status_value, channel_state_info->PVTStatus);
					if (pvt_status_value == 2)
					{
						pvt_status = true;
					}
					if (k > 1)
					{
						pvt_status_mask = pvt_status_mask - std::pow(2,k) - std::pow(2,k-1) + std::pow(2,k-2) + std::pow(2,k-3);
					}
				}
				if (pvt_status)
				{
					svid_pvt.push_back(static_cast<int32_t>(channel_sat_info->SVID));
				}
				index += sb2_size;
			}
		}
	}
	msg->status.satellite_used_prn = svid_pvt; // Entries such as int32[] in ROS messages are to be treated as std::vectors.
	msg->status.satellites_visible = static_cast<uint16_t>(svid_in_sync.size());
	msg->status.satellite_visible_prn = svid_in_sync_2;
	msg->status.satellite_visible_z = elevation_tracked;
	msg->status.satellite_visible_azimuth = azimuth_tracked;

	// Reordering CNO vector to that of all previous arrays
	std::vector<int32_t> cno_tracked_reordered;
	//ROS_DEBUG("size svid_in_sync is %i, size of cno_tracked is %i, size of svid_in_sync_2 is %i", static_cast<int32_t>(svid_in_sync.size()), static_cast<int32_t>(cno_tracked.size()), static_cast<int32_t>(svid_in_sync_2.size()));
	if (static_cast<int32_t>(last_channelstatus_.N) != 0)
	{
		for (int32_t k = 0; k < static_cast<int32_t>(ordering.size()); ++k)
		{
			cno_tracked_reordered.push_back(cno_tracked[ordering[k]]);
		}
	}
	msg->status.satellite_visible_snr = cno_tracked_reordered;
	
	// PVT Status Analysis
	uint16_t status_mask = 15; // We extract the first four bits using this mask.
	uint16_t type_of_pvt = ((uint16_t) (last_pvtgeodetic_.Mode)) & status_mask;
	switch(type_of_pvt)
	{
		case 0:
		{
			msg->status.status = gps_common::GPSStatus::STATUS_NO_FIX;
			break;
		}
		case 1: case 3:
		{
			msg->status.status = gps_common::GPSStatus::STATUS_FIX;
			break;
		}
		case 2: case 4: case 5: case 7: case 8: case 10:
		{
			msg->status.status = gps_common::GPSStatus::STATUS_GBAS_FIX;
			break;
		}
		case 6:
		{
			uint16_t reference_id = last_pvtgeodetic_.ReferenceID;
			if (reference_id == 131 || reference_id == 133 || reference_id == 135 || reference_id == 135) // PRNs of the 4 WAAS satellites 
			{
				msg->status.status = gps_common::GPSStatus::STATUS_WAAS_FIX;
			}
			else
			{
				msg->status.status = gps_common::GPSStatus::STATUS_SBAS_FIX;
			}
			break;
		}
		default:
		{
			throw std::runtime_error("PVTGeodetic's Mode field contains an invalid type of PVT solution.");
		}
	}
	msg->status.motion_source = gps_common::GPSStatus::SOURCE_POINTS; // Doppler is not used when calculating the velocities of mosaic.
	msg->status.orientation_source = gps_common::GPSStatus::SOURCE_POINTS; // Doppler is not used when calculating the orientation of mosaic.
	msg->status.position_source = gps_common::GPSStatus::SOURCE_GPS;
	msg->latitude = last_pvtgeodetic_.Latitude*360/(2*boost::math::constants::pi<double>());
	msg->longitude = last_pvtgeodetic_.Longitude*360/(2*boost::math::constants::pi<double>());
	msg->altitude = last_pvtgeodetic_.Height;
	msg->track = static_cast<double>(last_pvtgeodetic_.COG); // Note that COG is of type float32 while track is of type float64.
	msg->speed = std::sqrt(std::pow(static_cast<double>(last_pvtgeodetic_.Vn), 2) + std::pow(static_cast<double>(last_pvtgeodetic_.Ve), 2));
	msg->climb = static_cast<double>(last_pvtgeodetic_.Vu);
	msg->pitch = last_atteuler_.Pitch;
	msg->roll = last_atteuler_.Roll;
	// Heading was already given in the "track" field.
	if (last_dop_.PDOP == static_cast<uint16_t>(0) || last_dop_.TDOP == static_cast<uint16_t>(0))
	{
		msg->gdop = static_cast<double>(-1);
	}
	else
	{
		msg->gdop = std::sqrt(std::pow(static_cast<double>(last_dop_.PDOP)/100, 2) + std::pow(static_cast<double>(last_dop_.TDOP)/100, 2));
	}
	if (last_dop_.PDOP == static_cast<uint16_t>(0))
	{
		msg->pdop = static_cast<double>(-1);
	}
	else
	{
		msg->pdop = static_cast<double>(last_dop_.PDOP)/100;
	}
	if (last_dop_.HDOP == static_cast<uint16_t>(0))
	{
		msg->hdop = static_cast<double>(-1);
	}
	else
	{
		msg->hdop = static_cast<double>(last_dop_.HDOP)/100;
	}
	if (last_dop_.VDOP == static_cast<uint16_t>(0))
	{
		msg->vdop = static_cast<double>(-1);
	}
	else
	{
		msg->vdop = static_cast<double>(last_dop_.VDOP)/100;
	}
	if (last_dop_.TDOP == static_cast<uint16_t>(0))
	{
		msg->tdop = static_cast<double>(-1);
	}
	else
	{
		msg->tdop = static_cast<double>(last_dop_.TDOP)/100;
	}
	msg->time = static_cast<double>(last_pvtgeodetic_.TOW)/1000 + static_cast<double>(last_pvtgeodetic_.WNc*7*24*60*60);
	msg->err = 2*(std::sqrt(static_cast<double>(last_poscovgeodetic_.Cov_latlat) + static_cast<double>(last_poscovgeodetic_.Cov_lonlon) + static_cast<double>(last_poscovgeodetic_.Cov_hgthgt)));
	msg->err_horz = 2*(std::sqrt(static_cast<double>(last_poscovgeodetic_.Cov_latlat) + static_cast<double>(last_poscovgeodetic_.Cov_lonlon)));
	msg->err_vert = 2*std::sqrt(static_cast<double>(last_poscovgeodetic_.Cov_hgthgt));
	msg->err_track = 2*(std::sqrt(std::pow(static_cast<double>(1)/(static_cast<double>(last_pvtgeodetic_.Vn)+std::pow(static_cast<double>(last_pvtgeodetic_.Ve),2)/static_cast<double>(last_pvtgeodetic_.Vn)),2)*static_cast<double>(last_poscovgeodetic_.Cov_lonlon)+std::pow((static_cast<double>(last_pvtgeodetic_.Ve))/(std::pow(static_cast<double>(last_pvtgeodetic_.Vn),2)+std::pow(static_cast<double>(last_pvtgeodetic_.Ve),2)),2)*static_cast<double>(last_poscovgeodetic_.Cov_latlat)));
	msg->err_speed = 2*(std::sqrt(static_cast<double>(last_velcovgeodetic_.Cov_VnVn) + static_cast<double>(last_velcovgeodetic_.Cov_VeVe)));
	msg->err_climb = 2*std::sqrt(static_cast<double>(last_velcovgeodetic_.Cov_VuVu));
	msg->err_time = 2*std::sqrt(static_cast<double>(last_poscovgeodetic_.Cov_bb));
	msg->err_pitch = 2*std::sqrt(static_cast<double>(last_attcoveuler_.Cov_PitchPitch));
	msg->err_roll = 2*std::sqrt(static_cast<double>(last_attcoveuler_.Cov_RollRoll));
	msg->position_covariance[0] = last_poscovgeodetic_.Cov_lonlon;
	msg->position_covariance[1] = last_poscovgeodetic_.Cov_latlon;
	msg->position_covariance[2] = last_poscovgeodetic_.Cov_lonhgt;
	msg->position_covariance[3] = last_poscovgeodetic_.Cov_latlon;
	msg->position_covariance[4] = last_poscovgeodetic_.Cov_latlat;
	msg->position_covariance[5] = last_poscovgeodetic_.Cov_lathgt;
	msg->position_covariance[6] = last_poscovgeodetic_.Cov_lonhgt;
	msg->position_covariance[7] = last_poscovgeodetic_.Cov_lathgt;
	msg->position_covariance[8] = last_poscovgeodetic_.Cov_hgthgt;
	msg->position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_KNOWN;
	
	return msg;
}
	
/// If the current time shall be employed, it is calculated via the time(NULL) function found in the <ctime> library
/// At the time of writing the code (2020), the GPS time was ahead of UTC time by 18 (leap) seconds. Adapt 
/// the leap_seconds parameter accordingly as soon as the next leap second is inserted into the UTC time.
ros::Time io_comm_mosaic::TimestampSBF(uint32_t TOW, bool use_GNSS)
{
	if (use_GNSS)
	{
		uint16_t hh = (TOW%(1000*60*60*24))/(60*60*1000);
		uint16_t mm = ((TOW%(1000*60*60*24))-hh*(60*60*1000))/(60*1000);
		uint16_t ss = ((TOW%(1000*60*60*24))-hh*(60*60*1000)-mm*(60*1000))/(1000);
		uint16_t hs = ((TOW%(1000*60*60*24))-hh*(60*60*1000)-mm*(60*1000)-ss*1000)/10; // hundredths of a second
		if (ss >= leap_seconds)
		{
			ss = ss - leap_seconds;
		}
		else
		{	
			if (mm >= 1)
			{
				--mm;
				ss = 60 - (leap_seconds - ss);
			}
			else
			{
				if (hh >= 1)
				{
					--hh;
					mm = 59;
					ss = 60 - (leap_seconds - ss);
				}
				else
				{
					hh = 23;
					mm = 59;
					ss = 60 - (leap_seconds - ss);
				}
			}
		}
		boost::format fmt = boost::format("%02u%02u%02u.%02u") % hh % mm % ss % hs; 
		std::string utc_string = fmt.str();
		//ROS_DEBUG("UTC string is %s", utc_string.c_str());
		double utc_double;
		string_utilities::ToDouble(utc_string, utc_double);
		time_t unix_time_seconds = rosaic_driver::UTCtoUnix(utc_double);
		uint32_t unix_time_nanoseconds = (static_cast<uint32_t>(utc_double*100)%100)*10000000; 	// works since there are two digits after the decimal point in the utc_double
		ros::Time time_obj(unix_time_seconds, unix_time_nanoseconds);
		return time_obj;
	}
	else
	{
		time_t time_now = time(NULL);
		ros::Time time_obj((uint32_t) time_now, 0);
		return time_obj;
	}
}

bool io_comm_mosaic::mosaicMessage::Found()
{
	if (found_) return true;
	
	// Verify header bytes
	if (!this->IsSBF() && !this->IsNMEA() && !this->IsResponse())
	{
		return false;
	}
	
	found_ = true;
	return true;
}
  
const uint8_t* io_comm_mosaic::mosaicMessage::Search()
{
	if (found_) 
	{	
		Next(); 
	}
	// Search for message or a response header
	for( ; count_ > 0; --count_, ++data_) 
	{
		if (this->IsSBF() || this->IsNMEA() || this->IsResponse())
		{
			break;
		}
	}
	found_ = true;
	return data_;
}

std::size_t io_comm_mosaic::mosaicMessage::SegmentEnd()
{
	uint16_t pos = 0;
	segment_size_ = 0;
	if (this->IsResponse())
	{
		do
		{
			++segment_size_;
			++pos;
		} while(!((data_[pos] == CARRIAGE_RETURN && data_[pos+1] == LINE_FEED)) || (data_[pos] == CARRIAGE_RETURN && data_[pos+1] == LINE_FEED && data_[pos+2] == 0x20 && data_[pos+3] == 0x20 && data_[pos+4] == 0x4E) || (data_[pos] == CARRIAGE_RETURN && data_[pos+1] == LINE_FEED && data_[pos+2] == 0x20 && data_[pos+3] == 0x20 && data_[pos+4] == 0x53) || (data_[pos] == CARRIAGE_RETURN && data_[pos+1] == LINE_FEED && data_[pos+2] == 0x20 && data_[pos+3] == 0x20 && data_[pos+4] == 0x52));
	}
	else
	{
		do
		{
			++segment_size_;
			++pos;
		} while(!((data_[pos] == CARRIAGE_RETURN && data_[pos+1] == LINE_FEED)));
	}
	return segment_size_;
}
bool io_comm_mosaic::mosaicMessage::IsMessage(const uint16_t ID)
{
	if (this->IsSBF())
	{
		uint16_t mask = 8191;
		if (*(reinterpret_cast<const uint16_t *>(data_ + 4)) & mask== static_cast<const uint16_t>(ID))
		// Caution: reinterpret_cast is the most dangerous cast, It's used primarily for particularly weird conversions and bit manipulations, like turning a raw data stream into actual data
		{
			return true;
		}
		else
		{
			return false;
		}
	}
	else
	{
		return false;
	}
}


bool io_comm_mosaic::mosaicMessage::IsMessage(std::string ID)
{
	if (this->IsNMEA())
	{
		boost::char_separator<char> sep(",");
		typedef boost::tokenizer<boost::char_separator<char> > tokenizer;
		std::size_t end_point = std::min(static_cast<std::size_t>(this->End() - data_), static_cast<std::size_t>(82));
		std::string block_in_string(reinterpret_cast<const char*>(data_), end_point);
		tokenizer tokens(block_in_string,sep);
		if (*tokens.begin() == ID) 
		{
			return true;
		}
		else
		{
			return false;
		}
	}
	else
	{
		return false;
	}
}

bool io_comm_mosaic::mosaicMessage::IsSBF()
{
	if (data_[0] == SBF_SYNC_BYTE_1 && data_[1] == SBF_SYNC_BYTE_2)
	{
		return true;
	}
	else
	{
		return false;
	}
}

bool io_comm_mosaic::mosaicMessage::IsNMEA()
{
	if ((data_[0] == NMEA_SYNC_BYTE_1 && data_[1] == NMEA_SYNC_BYTE_2_1) || (data_[0] == NMEA_SYNC_BYTE_1 && data_[1] == NMEA_SYNC_BYTE_2_2))
	{
		return true;
	}
	else
	{
		return false;
	}
}

bool io_comm_mosaic::mosaicMessage::IsResponse()
{
	if (data_[0] == RESPONSE_SYNC_BYTE_1 && data_[1] == RESPONSE_SYNC_BYTE_2)
	{
		return true;
	}
	else
	{
		return false;
	}
}

bool io_comm_mosaic::mosaicMessage::IsErrorMessage()
{
	if (data_[0] == RESPONSE_SYNC_BYTE_1 && data_[1] == RESPONSE_SYNC_BYTE_2 && data_[2] == RESPONSE_SYNC_BYTE_3)
	{
		return true;
	}
	else
	{
		return false;
	}
}

std::string io_comm_mosaic::mosaicMessage::MessageID()
{
	if (this->IsSBF())
	{
		// Define bit mask:
		uint16_t mask = 8191; // It is not as stated in the firmware: !first! three bits are for revision (not last 3), and rest for block number
		uint16_t value = (*(reinterpret_cast<const uint16_t*>(data_+4))) & mask; // Bitwise AND gives us all but first 3 bits set to zero, rest unchanged
		std::stringstream ss;
		ss << value;
		return ss.str();
	}
	if (this->IsNMEA())
	{
		boost::char_separator<char> sep(",");
		typedef boost::tokenizer<boost::char_separator<char> > tokenizer;
		std::size_t end_point = std::min(static_cast<std::size_t>(this->End() - data_), static_cast<std::size_t>(82));
		std::string block_in_string(reinterpret_cast<const char*>(data_), end_point);
		tokenizer tokens(block_in_string,sep);
		return *tokens.begin();
	}
	return std::string(); // less CPU work than return "";
}



const uint8_t* io_comm_mosaic::mosaicMessage::Pos()
{
	return data_;
}

const uint8_t* io_comm_mosaic::mosaicMessage::End()
{
	return data_ + count_;
}

uint16_t io_comm_mosaic::mosaicMessage::BlockLength()
{
	if (this->IsSBF())
	{
		uint16_t block_length;
		// Note that static_cast<uint16_t>(data_[6]) would just take the one byte "data_[6]" and cast it as requested, !neglecting! the byte "data_[7]".
		block_length = *(reinterpret_cast<const uint16_t*>(data_ + 6));
		return block_length;
	}
	else
	{
		return 0;
	}
}

/**
 * Warning: Won't jump to next message if current one is an NMEA message. search() will then check bytes one by one for the new message's sync bytes ($P or $G).
 */
const uint8_t* io_comm_mosaic::mosaicMessage::Next()
{
	if (Found()) 
	{
		if (this->IsNMEA() || this->IsResponse())
		{
			--count_;
			++data_;
			return data_;
		}
		if (this->IsSBF())
		{
			uint32_t jump_size;
			if (CRCcheck_)
			{
				jump_size = this->BlockLength();
				if (jump_size == 0) jump_size = 1; 	// Some corrupted messages that survive the CRC check (this happened) could tell ROSaic their size is 0,
													// which would lead to an endless while loop in the ReadCallback() method of the CallbackHandlers class. 
			}
			else
			{
				jump_size = 1;
			}
			ROS_DEBUG("Jump about to happen with jump size %u", jump_size);
			data_ += jump_size; count_ -= jump_size;
		}
	}
	found_ = false;
	return data_;
}
