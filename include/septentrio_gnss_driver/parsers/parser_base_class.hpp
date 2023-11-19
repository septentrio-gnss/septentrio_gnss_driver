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

// ROSaic includes
#include "nmea_sentence.hpp"
#include "parse_exception.hpp"
#include "parsing_utilities.hpp"

/**
 * @file parser_base_class.hpp
 * @brief Declares a base class for parsing NMEA messages and SBF blocks
 * @date 13/08/20
 */

/**
 * @class BaseParser
 * @brief Base class for parsing NMEA messages and SBF blocks
 *
 * Subclasses that parse NMEA messages should implement
 * ParseASCII(const NMEASentence&); The base class is implemented
 * as a template, which is a simple and yet very powerful tool in C++. The
 * simple idea is to pass data type as a parameter so that we don’t need to
 * write the same code for different data types. Like function templates, class
 * templates are useful when a class defines something that is independent of
 * the data type, as here the notion of parsing.
 *
 * @tparam T The ROS message pointer type that the parser should produce, e.g.
 * nmea_msgs::GpggaPtr.
 */
template <typename T>
class BaseParser
{
public:
    /**
     * @brief Default constructor of the class BaseParser
     *
     * Adding the "default" keyword to the constructor declaration is a new feature
     * since C++11 for creating a default constructor (a constructor which can be
     * called with no arguments). Strictly speaking, it is not the same as when the
     * keyword is omitted, but the differences are miniscule.
     *
     * Also note that in C++, the constructor cannot be virtual, because when a
     * constructor of a class is executed, there is no virtual table in the memory,
     * i.e. no virtual pointer defined yet.
     *
     */
    BaseParser() = default;

    /**
     * @brief Default destructor of the class BaseParser
     *
     * As opposed to the constructor, a destructor can be virtual, as here.
     */
    virtual ~BaseParser() = default;

    /**
     * @brief Returns the ASCII message name
     *
     * GetMessageID() is a pure virtual function, i.e. a function
     * for which writing a function declaration suffices. It is declared by
     * assigning the value 0 in the declaration. Since we now have at least
     * 1 pure virtual function, our class BaseParser thus becomes an
     * "abstract" one.
     *
     * @return The ASCII message name.
     */
    virtual const std::string getMessageID() const = 0;

    /**
     * @brief Converts an NMEA sentence - both standardized and proprietary ones -
     * into a ROS message pointer (e.g. nmea_msgs::GpggaPtr) and returns it
     *
     * The returned value should not be NULL. ParseException will be thrown
     * if there are any issues parsing the message.
     * @param[in] sentence The standardized NMEA sentence to convert, of type
     * NMEASentence
     * @return A valid ROS message pointer
     */
    virtual T parseASCII(const NMEASentence& sentence, const std::string& frame_id,
                         bool use_gnss_time, Timestamp time_obj) noexcept(false)
    {
        throw ParseException("ParseASCII not implemented.");
    };
};