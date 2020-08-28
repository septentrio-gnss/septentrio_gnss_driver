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


#ifndef NMEA_MSGS_GPGGA_HPP
#define NMEA_MSGS_GPGGA_HPP

#include <MinROS/parsers/parser_base_class.hpp>
#include <nmea_msgs/Gpgga.h>

/**
 * @file gpgga.hpp
 * @brief Derived class for parsing GGA messages
 * @date 17/08/20 
 */

extern std::string frame_id;
namespace minros_driver
{
	/**
	 * @class GpggaParser
	 * @brief Derived class for parsing GGA messages
	 *
	 * Here, the access-specifier is set to "public". Recall this type of inheritance: 
	 * In public access mode, public members of the base class will become public in the derived 
	 * class and protected members of the base class will become protected in the derived class. 
	 * @date 13/08/20
	 */
	class GpggaParser : public BaseParser<nmea_msgs::GpggaPtr>
	{
	public:
		/**
		 * @brief Constructor of the class GpggaParser
		 * 
		 * Here, we make use of the concept of member initialization lists:
		 * Instead of writing all the initialization specifications into {}, 
		 * we can list all of these, delimited by commas, before the opening and closing bracket {}.
		 */
		GpggaParser(): BaseParser<nmea_msgs::GpggaPtr>(), was_last_gpgga_valid_(false)
		{}

		/**
		 * @brief Returns the ASCII message ID, here "$GPGGA"
		 * @return The message ID
		 */
		const std::string GetMessageID() const override; 	
		
		/**
		 * @brief Parses one GGA message 
		 * @param[in] sentence The GGA message to be parsed
		 * @return A ROS message pointer of ROS type nmea_msgs::GpggaPtr
		 */
		nmea_msgs::GpggaPtr ParseASCII(const NMEASentence& sentence) noexcept(false) override;

		/**
		 * @brief Tells us whether the last GGA message was valid or not 
		 * @return True if last GGA message was valid, false if not
		 */
		bool WasLastGPGGAValid() const;

		/**
		 * @brief Declares the string MESSAGE_ID
		 */
		static const std::string MESSAGE_ID;
		
	private:
		/**
		 * @brief Declares a boolean representing whether or not the last GPGGA message was valid
		 */
		bool was_last_gpgga_valid_;
	};
}

#endif //NMEA_MSGS_GPGGA_HPP