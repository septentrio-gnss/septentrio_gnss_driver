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

#pragma once

#include <cstdint>
#include <type_traits>

template <class T, class = void>
struct has_block_header : std::false_type
{
};

template <class T>
struct has_block_header<T, std::void_t<decltype(T::block_header)>> : std::true_type
{
};

/**
 * validValue
 * @brief Check if value is not set to Do-Not-Use
 */
template <typename T>
[[nodiscard]] bool validValue(T s)
{
    static_assert(std::is_same<uint16_t, T>::value ||
                  std::is_same<uint32_t, T>::value ||
                  std::is_same<float, T>::value || std::is_same<double, T>::value);
    if (std::is_same<uint16_t, T>::value)
    {
        return (s != static_cast<uint16_t>(65535));
    } else if (std::is_same<uint32_t, T>::value)
    {
        return (s != 4294967295u);
    } else if (std::is_same<float, T>::value)
    {
        return (!std::isnan(s) && (s != -2e10f));
    } else if (std::is_same<double, T>::value)
    {
        return (!std::isnan(s) && (s != -2e10));
    }
}