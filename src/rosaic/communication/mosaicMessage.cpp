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
 
rosaic::PVTGeodeticPtr io_comm_mosaic::PVTGeodeticCallback(PVTGeodetic& data)
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


rosaic::PVTCartesianPtr io_comm_mosaic::PVTCartesianCallback(PVTCartesian& data)
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


/// If the current time shall be employed, it is calculated via BOOST, using the type boost::gregorian::date.
/// At the time of writing the code (2020), the GPS time was ahead of UTC time by 18 (leap) seconds. Adapt 
/// accordingly as soon as the next leap second is inserted into the UTC time.
ros::Time io_comm_mosaic::timestampSBF(uint32_t TOW, bool use_GNSS)
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
		//ROS_DEBUG("UTC string (still need to subtract 18 seconds) is %s", utc_string.c_str());
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

bool io_comm_mosaic::mosaicMessage::found()
{
	if (found_) return true;
	
	// Verify header bytes
	if (!((data_[0] == SEP_SYNC_BYTE_1 && data_[1] == SEP_SYNC_BYTE_2) || (data_[0] == SEP_SYNC_BYTE_1 && data_[1] == SEP_SYNC_BYTE_3) || (data_[0] == SEP_SYNC_BYTE_1 && data_[1] == SEP_SYNC_BYTE_4)))
	{
		return false;
	}
	
	found_ = true;
	return true;
}
  
const uint8_t* io_comm_mosaic::mosaicMessage::search()
{
	if (found_) 
	{	
		next(); 
	}
	// Search for a message header
	for( ; count_ > 0; --count_, ++data_) 
	{
		if ((data_[0] == SEP_SYNC_BYTE_1 && data_[1] == SEP_SYNC_BYTE_2) || (data_[0] == SEP_SYNC_BYTE_1 && data_[1] == SEP_SYNC_BYTE_3) || (data_[0] == SEP_SYNC_BYTE_1 && data_[1] == SEP_SYNC_BYTE_4)) 
		{
			break;
		}
	}
	found_ = true;
	return data_;
}


bool io_comm_mosaic::mosaicMessage::checksum()
{
	if ((data_[0] == SEP_SYNC_BYTE_1 && data_[1] == SEP_SYNC_BYTE_3) || (data_[0] == SEP_SYNC_BYTE_1 && data_[1] == SEP_SYNC_BYTE_4))
	{
		return false;
	}
	if (data_[0] == SEP_SYNC_BYTE_1 && data_[1] == SEP_SYNC_BYTE_2)
	{
		return *reinterpret_cast<const uint16_t *>(data_ + 2); //data[2] would only catch first half of CRC
	}
}

bool io_comm_mosaic::mosaicMessage::isMessage(const uint16_t ID)
{
	if (data_[0] == SEP_SYNC_BYTE_1 && data_[1] == SEP_SYNC_BYTE_2)
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
		

bool io_comm_mosaic::mosaicMessage::isMessage(std::string ID)
{
	if ((data_[0] == SEP_SYNC_BYTE_1 && data_[1] == SEP_SYNC_BYTE_3) || (data_[0] == SEP_SYNC_BYTE_1 && data_[1] == SEP_SYNC_BYTE_4))
	{
		boost::char_separator<char> sep(",");
		typedef boost::tokenizer<boost::char_separator<char> > tokenizer;
		std::size_t end_point = std::min(static_cast<std::size_t>(this->end() - data_), static_cast<std::size_t>(82));
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

std::string io_comm_mosaic::mosaicMessage::MessageID()
{
	if (data_[0] == SEP_SYNC_BYTE_1 && data_[1] == SEP_SYNC_BYTE_2)
	{
		//ROS_DEBUG("MessageID if clause");
		// Construct bit mask:
		uint16_t mask = 8191; // It is not as stated in the firmware: !first! three bits are for revision (not last 3), and rest for block number
		uint16_t value = (*(reinterpret_cast<const uint16_t*>(data_+4))) & mask; // Bitwise AND gives us first 3 bits set to zero, rest unchanged
		std::stringstream ss;
		ss << value;
		return ss.str();
	}
	if ((data_[0] == SEP_SYNC_BYTE_1 && data_[1] == SEP_SYNC_BYTE_3) || (data_[0] == SEP_SYNC_BYTE_1 && data_[1] == SEP_SYNC_BYTE_4))
	{
		boost::char_separator<char> sep(",");
		typedef boost::tokenizer<boost::char_separator<char> > tokenizer;
		std::size_t end_point = std::min(static_cast<std::size_t>(this->end() - data_), static_cast<std::size_t>(82));
		std::string block_in_string(reinterpret_cast<const char*>(data_), end_point);
		tokenizer tokens(block_in_string,sep);
		return *tokens.begin();
	}
	return std::string(); // less work than return "";
}



const uint8_t* io_comm_mosaic::mosaicMessage::pos()
{
	return data_;
}

const uint8_t* io_comm_mosaic::mosaicMessage::end()
{
	return data_ + count_;
}

uint16_t io_comm_mosaic::mosaicMessage::block_length()
{
	if (data_[0] == SEP_SYNC_BYTE_1 && data_[1] == SEP_SYNC_BYTE_2)
	{
		return static_cast<uint16_t>(data_[6]);
	}
	else
	{
		return 0;
	}
}

/**
 * Warning: Won't jump to next message if current one is an NMEA message. search() will then check bytes one by one for the new message start.
 */
const uint8_t* io_comm_mosaic::mosaicMessage::next()
{
	if (found()) 
	{
		if ((data_[0] == SEP_SYNC_BYTE_1 && data_[1] == SEP_SYNC_BYTE_3) || (data_[0] == SEP_SYNC_BYTE_1 && data_[1] == SEP_SYNC_BYTE_4))
		{
			--count_;
			++data_;
			return data_;
		}
		if (data_[0] == SEP_SYNC_BYTE_1 && data_[1] == SEP_SYNC_BYTE_2)
		{
			//ROS_DEBUG("Jump about to happen with jump size %u", block_length());
			uint32_t size = 1;
			data_ += size; count_ -= size;
		}
	}
	found_ = false;
	return data_;
}
