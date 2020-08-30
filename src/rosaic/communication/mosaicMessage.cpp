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
 

bool io_comm_mosaic::mosaicMessage::found()
{
	if (found_) return true;
	
	// Verify header bytes
	if (!((data_[0] == SEP_SYNC_BYTE_1 && data_[1] == SEP_SYNC_BYTE_2) || (data_[0] == SEP_SYNC_BYTE_1 && data_[1] == SEP_SYNC_BYTE_3) || (data_[0] == SEP_SYNC_BYTE_1 && data_[1] == SEP_SYNC_BYTE_4)))
		return false;

	found_ = true;
	return true;
}
  
const uint8_t* io_comm_mosaic::mosaicMessage::search()
{
	//ROS_DEBUG("found_ is %s at the moment", found_ ? "true" : "false");
	if (found_) next(); // Without this, you would have to check every byte, takes time. This jump only works for SBF of course, since for NMEA, message lengths are variable.
	// Search for a message header
	for( ; count_ > 0; --count_, ++data_) {
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
	if (data_[1] == SEP_SYNC_BYTE_2)
	{
		return false;
	}
	else
	{
		return *reinterpret_cast<const uint16_t *>(data_ + 2); //data[2] would only catch first half of CRC
	}
}

bool io_comm_mosaic::mosaicMessage::isMessage(const uint16_t ID)
{
	if (data_[1] == SEP_SYNC_BYTE_2)
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
	if (data_[1] == SEP_SYNC_BYTE_3 || data_[1] == SEP_SYNC_BYTE_4)
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
	if (data_[1] == SEP_SYNC_BYTE_2)
	{
		// Construct bit mask:
		uint16_t mask = 0;
		uint16_t value = (*reinterpret_cast<const uint16_t *>(data_ + 4)) & (~mask << 3); // Bitwise AND gives us first 13 bits unchanged, rest set to zero
		std::stringstream ss;
		ss << value;
		return ss.str();
	}
	else
	{
		boost::char_separator<char> sep(",");
		typedef boost::tokenizer<boost::char_separator<char> > tokenizer;
		std::size_t end_point = std::min(static_cast<std::size_t>(this->end() - data_), static_cast<std::size_t>(82));
		std::string block_in_string(reinterpret_cast<const char*>(data_), end_point);
		tokenizer tokens(block_in_string,sep);
		return *tokens.begin();
	}
}



const uint8_t* io_comm_mosaic::mosaicMessage::pos()
{
	return data_;
}

const uint8_t* io_comm_mosaic::mosaicMessage::end()
{
	return data_ + count_;
}

uint32_t io_comm_mosaic::mosaicMessage::block_length()
{
	if (data_[1] == SEP_SYNC_BYTE_2)
	{
		return static_cast<uint32_t>(static_cast<uint16_t>(data_[6]));
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
		if (data_[1] == SEP_SYNC_BYTE_3 || data_[1] == SEP_SYNC_BYTE_4)
		{
			found_ = false;
			--count_;
			++data_;
			return data_;
		}
		else
		{
			uint32_t size = block_length();
			data_ += size; count_ -= size;
		}
	}
	found_ = false;
	return data_;
}
