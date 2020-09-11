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

#include <rosaic/crc/crc.h>

/**
 * @file crc.c
 * @brief Defines the CRC table and the functions to compute and validate the CRC of a buffer
 * @date 17/08/20 
 */

/**
 * Note that a void pointer is a pointer that has no associated data type with it. A void pointer can hold address of any type and can be typcasted to any type.
 */
uint16_t CRC_compute16CCITT (const void *buf, size_t buf_length) // the CRC we choose is 2 bytes, remember, hence uint16_t..
{
	uint32_t  i;  // since i might become rather large, after all..
	uint16_t  crc = 0; // seed is 0, as suggested by ref guide. computed in the forward direction
	const uint8_t  *buf8 = (const uint8_t *) buf; // Convert the type to access by byte. RHS is pointer to constant type uint8_t, pointing to the address buf
												// later in the code this address buf8 is used to write/read an actual uint8_t value to/from there.
												// uint8_t x = address_at_eeprom_location[3]: This would read the 3rd uint8_t starting from address 10 (so at address 13, since addresses work byte-wise) in ram into the variable x
	for (i=0; i<buf_length; i++) {
		crc = (crc << 8) ^ CRCLookUp[ (crc >> 8) ^ buf8[i] ]; 
	// The ^ (bitwise XOR) in C or C++ takes two numbers as operands and does XOR on every bit of two numbers. The result of XOR is 1 if the two bits are different. 
	// The << (left shift) in C or C++ takes two numbers, left shifts the bits of the first operand, the second operand decides the number of places to shift. 
	// The >> (right shift) in C or C++ takes two numbers, right shifts the bits of the first operand, the second operand decides the number of places to shift; you can just loose the smallest values if big-endian
	// The left shift and right shift operators should not be used for negative numbers
	// The left-shift and right-shift operators are equivalent to multiplication and division by 2 respectively, hence only rightshift is non-exact (remainder is not retained)
	// CRC_SBF_LookUp contains is constructed from truncated polynomial (divisor)
	// The above implements a kind of CRC 32 algorithm: efficient, fast
	}

	return crc;
}

bool CRCIsValid(const void *Block)
{
	const VoidBlock_t * Block2 = (const VoidBlock_t *) Block; // Convert to access the generic header fields. Otherwise Block2->ID would not work.
	uint16_t crc;
	crc = CRC_compute16CCITT ( &(Block2->ID), Block2->Length-2*sizeof(uint16_t) ); // need address of ID, then all the message except for first 4 bytes (Sync and CRC)
	return (crc ==  Block2->CRC) ? true:false;
}
