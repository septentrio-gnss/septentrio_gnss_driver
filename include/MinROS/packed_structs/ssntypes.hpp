/*
 * ssntypes.h: 
 *  Declarations of Septentrio types and implementation of common C types which
 *  are not implemented by every compiler.
 *
 *  If your compiler does not support the standard C99 types from \p stdint.h
 *  and \p stdbool.h, please define them for your platform. \n
 *  This is already done for the Microsoft C/C++ compiler.
 *
 *
 * Septentrio grants permission to use, copy, modify, and/or distribute
 * this software for any purpose with or without fee.  
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND SEPTENTRIO DISCLAIMS ALL
 * WARRANTIES WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL
 * SEPTENTRIO BE LIABLE FOR ANY SPECIAL, DIRECT, INDIRECT, OR
 * CONSEQUENTIAL DAMAGES OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS
 * OF USE, DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT,
 * NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN
 * CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

#ifndef SSNTYPES_HPP
#define SSNTYPES_HPP /**< To avoid multiple inclusions. */

/**
 * @file ssntypes.hpp
 * @brief Aims at making the C++ code as portable as possible, by dealing with all compilers except for MS compilers (since ROS Windows is not supported)
 * @date 17/08/20 
*/

#ifdef SSN_DLL
# ifdef _WIN32
#  ifdef FW_MAKE_DLL
#   define FW_EXPORT __declspec(dllexport)
#  else
#   define FW_EXPORT __declspec(dllimport)
#  endif
# else
#  ifdef FW_MAKE_DLL
#   define FW_EXPORT __attribute__((visibility("default")))
#  else
#   define FW_EXPORT
#  endif
# endif
#else
# define FW_EXPORT
#endif

#if (defined(__STDC_VERSION__) && (__STDC_VERSION__ >= 199901L)) || defined(__GNUC__) || defined(__ARMCC__)
#  include <stdint.h> // see comment in preamble
#  include <stdbool.h>
#  include <inttypes.h>
// #else
// #  ifdef _MSC_VER  // only for MS compilers: _MSC_VER is always defined whenever you are using our Windows based (icl) compiler. It is set to whatever version of the Microsoft compiler is in your PATH. 
// #    include "mscssntypes.h"
// #    ifndef _USE_MATH_DEFINES /* used to define M_PI and so*/
// #      define _USE_MATH_DEFINES
// #    endif
// #  endif
#endif

#endif
/* End of "ifndef SSNTYPES_HPP" */