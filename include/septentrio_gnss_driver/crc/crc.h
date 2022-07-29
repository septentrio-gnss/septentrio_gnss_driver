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

#ifndef CRC_H
#define CRC_H

// ROSaic includes
// The following imports structs into which SBF blocks can be unpacked then shipped
// to handler functions
#include <septentrio_gnss_driver/packed_structs/sbf_structs.hpp>
// C++ libary includes
#include <cstdint>
#include <stdbool.h>
#include <stddef.h>

/**
 * @file crc.h
 * @brief Declares the functions to compute and validate the CRC of a buffer
 * @date 17/08/20
 */

/**
 * @brief This function computes the CRC-8-CCITT (Cyclic Redundancy Check) of a
 * buffer "buf" of "buf_length" bytes
 * @param[in] buf The buffer at hand
 * @param[in] buf_length Number of bytes in "buf"
 * @return The calculated CRC
 */
uint16_t compute16CCITT(const uint8_t* buf, size_t buf_length);

/**
 * @brief Validates whether the calculated CRC of the SBF block at hand matches the
 * CRC field of the streamed SBF block
 * @param block The SBF block that we are interested in
 * @return True if the CRC check of the SBFBlock has passed, false otherwise
 */
bool isValid(const uint8_t* block);

#endif // CRC_H
