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
uint32_t io_comm_mosaic::mosaicMessage::read_count_pvtgeodetic_ = 0;
uint32_t io_comm_mosaic::mosaicMessage::read_count_pvtcartesian_ = 0;
uint32_t io_comm_mosaic::mosaicMessage::read_count_poscovgeodetic_ = 0;
uint32_t io_comm_mosaic::mosaicMessage::read_count_atteuler_ = 0;
uint32_t io_comm_mosaic::mosaicMessage::read_count_attcoveuler_ = 0;
uint32_t io_comm_mosaic::mosaicMessage::read_count_navsatfix_ = 0;
uint32_t io_comm_mosaic::mosaicMessage::read_count_gpgga_ = 0;

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
 */
sensor_msgs::NavSatFixPtr io_comm_mosaic::mosaicMessage::NavSatFixCallback()
{
	sensor_msgs::NavSatFixPtr msg = boost::make_shared<sensor_msgs::NavSatFix>();
	uint16_t mask = 15; // We extract the first four bytes using this mask.
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

/// If the current time shall be employed, it is calculated via the time(NULL) function found in the <ctime> library
/// At the time of writing the code (2020), the GPS time was ahead of UTC time by 18 (leap) seconds. Adapt 
/// accordingly as soon as the next leap second is inserted into the UTC time.
ros::Time io_comm_mosaic::TimestampSBF(uint32_t TOW, bool use_GNSS)
{
	if (use_GNSS)
	{
		uint16_t hh = (TOW%(1000*60*60*24))/(60*60*1000);
		uint16_t mm = ((TOW%(1000*60*60*24))-hh*(60*60*1000))/(60*1000);
		uint16_t ss = ((TOW%(1000*60*60*24))-hh*(60*60*1000)-mm*(60*1000))/(1000);
		uint16_t hs = ((TOW%(1000*60*60*24))-hh*(60*60*1000)-mm*(60*1000)-ss*1000)/10; // hundredths of a second
		if (ss >= 18)
		{
			ss = ss - 18;
		}
		else
		{	
			if (mm >= 1)
			{
				--mm;
				ss = 60 - (18 - ss);
			}
			else
			{
				if (hh >= 1)
				{
					--hh;
					mm = 59;
					ss = 60 - (18 - ss);
				}
				else
				{
					hh = 23;
					mm = 59;
					ss = 60 - (18 - ss);
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
	// Search for a message header
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
	do
	{
		++segment_size_;
		++pos;
	} while(!(data_[pos] == CARRIAGE_RETURN && data_[pos+1] == LINE_FEED));
	if (this->IsResponse())
	{
		do
		{
			++segment_size_;
			++pos;
		} while(!(data_[pos] == CARRIAGE_RETURN && data_[pos+1] == LINE_FEED));
	}
	return segment_size_;
}
bool io_comm_mosaic::mosaicMessage::IsMessage(const uint16_t ID)
{
	if (this->IsSBF())
	{
		if (*reinterpret_cast<const uint16_t *>(data_ + 4) == static_cast<const uint16_t>(ID))
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

std::string io_comm_mosaic::mosaicMessage::MessageID()
{
	if (this->IsSBF())
	{
		// Define bit mask:
		uint16_t mask = 8191; // It is not as stated in the firmware: !first! three bits are for revision (not last 3), and rest for block number
		uint16_t value = (*(reinterpret_cast<const uint16_t*>(data_+4))) & mask; // Bitwise AND gives us first 3 bits set to zero, rest unchanged
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
		return static_cast<uint16_t>(data_[6]);
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
