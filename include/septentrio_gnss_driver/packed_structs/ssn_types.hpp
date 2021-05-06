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

#ifndef SSNTYPES_HPP
#define SSNTYPES_HPP

/**
 * @file ssn_types.hpp
 * @brief Aims at making the C++ code as portable as possible, by dealing with all
 * compilers except for MS compilers
 *
 * MS compilers are disregarded since ROS Windows is not supported.
 * Here we declare Septentrio types and implement common C/C++ types which are not
 * implemented by every compiler. If your compiler does not support the standard C99
 * types from \p stdint.h and \p stdbool.h, please define them for your platform.
 * @date 17/08/20
 */

#ifdef SSN_DLL
#ifdef _WIN32
#ifdef FW_MAKE_DLL
#define FW_EXPORT __declspec(dllexport)
#else
#define FW_EXPORT __declspec(dllimport)
#endif
#else
#ifdef FW_MAKE_DLL
#define FW_EXPORT __attribute__((visibility("default")))
#else
#define FW_EXPORT
#endif
#endif
#else
#define FW_EXPORT
#endif

#if (defined(__STDC_VERSION__) && (__STDC_VERSION__ >= 199901L)) ||                 \
    defined(__GNUC__) || defined(__ARMCC__)
#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h> // see comment in preamble
#endif

#endif // SSNTYPES_HPP