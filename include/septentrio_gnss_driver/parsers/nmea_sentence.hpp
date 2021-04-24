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

#ifndef NMEA_SENTENCE_HPP
#define NMEA_SENTENCE_HPP

// C++ library includes
#include <string>
#include <vector>

/**
 * @file nmea_sentence.hpp
 * @brief Defines a struct NMEASentence, into which NMEA sentences - both
 * standardized and proprietary ones - should be mapped
 * @date 13/08/20
 */

/**
 * @brief Struct to split an NMEA sentence into its ID and its body, the latter
 * tokenized into a vector of strings.
 *
 * By ID, we mean either a standardized ID, e.g. "$GPGGA", or proprietary ID such as
 * "$PSSN,HRP". The STL Container Vector can be used to dynamically allocate arrays
 * (C++ feature). Also note that the ID of !all! (not just those defined by
 * Septentrio) proprietary NMEA messages starts with "$P". The body_ member variable
 * shall exclude the NMEA checksum (also hinted at in files that implement the
 * parsing).
 */
class NMEASentence
{
public:
    NMEASentence(std::string id, std::vector<std::string> body) :
        id_(id), body_(body)
    {
    }
    std::vector<std::string> get_body() const { return body_; }

protected:
    std::string id_;
    std::vector<std::string> body_;
};

#endif // NMEA_SENTENCE_HPP
