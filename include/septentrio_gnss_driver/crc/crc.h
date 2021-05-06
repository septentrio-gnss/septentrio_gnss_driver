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
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// By wrapping the C code with extern "C" the C++ compiler will not mangle the C
// code's names (official term: name mangling, allows C++ function overloading),
// Therefore, !all! generic C style header files (stdio.h for e.g. printf(...),
// string.h, .. etc) have their declarations in extern “C” block, do the same for
// your C style header files!! extern "C" doesn't really change the way that the
// compiler reads the code. If your code is in a .c file, it will be compiled as C,
// if it is in a .cpp file, it will be compiled as C++ (unless you do something
// strange to your configuration). What extern "C" does is affect linkage. C++
// functions, when compiled, have their names mangled -- this is what makes
// overloading possible. The function name gets modified based on the types and
// number of parameters, so that two functions with the same name will have different
// symbol names. Code inside an extern "C" is still C++ code. There are limitations
// on what you can do in an extern "C" block, but they're all about linkage. You
// can't define any new symbols that can't be built with C linkage. That means no
// classes or templates, for example.

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
uint16_t FW_EXPORT compute16CCITT(const void* buf, size_t buf_length);

/**
 * @brief Validates whether the calculated CRC of the SBF block at hand matches the
 * CRC field of the streamed SBF block
 * @param block The SBF block that we are interested in
 * @return True if the CRC check of the SBFBlock has passed, false otherwise
 */
bool FW_EXPORT isValid(const void* block);

#ifdef __cplusplus
}
#endif

#endif // CRC_H
