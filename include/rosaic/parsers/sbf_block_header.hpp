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

#ifndef SBF_BLOCK_HEADER_HPP
#define SBF_BLOCK_HEADER_HPP

//! The cstdint header was originally in the C standard library as <stdint.h>. 
#include <cstdint> 
#include <rosaic/parsers/parsing_utilities.hpp>

/**
 * @file sbf_block_header.hpp
 * @brief Defines structs SBFBlockHeader and TimeHeader
 * @date 14/08/20 
*/

namespace rosaic_driver
{
	/**
	 * @brief Represents the header of an SBF block, excluding the timestamp
	 */
	struct SBFBlockHeader
	// In C++ the only difference between a class and a struct is that members and base classes are private by default in classes, whereas they are public by default in structs. 
	{
		uint16_t       Sync;
		uint16_t       CRC;
		uint16_t       ID;
		uint16_t       Length;
	};
	
	/**
	 * @brief Represents the header of an SBF block, including the timestamp
	 */
	struct TimeHeader
	{
		SBFBlockHeader Header;

		uint32_t       TOW;
		uint16_t       WNc;
	};
}
#endif //SBF_BLOCK_HEADER_HPP
