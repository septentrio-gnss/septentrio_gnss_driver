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

#ifndef PARSE_EXCEPTION_HPP
#define PARSE_EXCEPTION_HPP

// C++ library includes
#include <stdexcept>
// The C++ Standard library provides a base class specifically designed to declare
// objects to be thrown as exceptions. It is called std::exception and is defined in
// the <exception> header. This class has a virtual member function called what that
// returns a null-terminated character sequence (of type char *) and that can be
// overwritten in derived classes to contain some sort of description of the
// exception. Example Code for using base class exception:
/*
class myexception: public exception
{
    virtual const char* what() const throw()
    {
        return "My exception happened";
    }
} myex;

int main () {
    try
    {
        throw myex;
    }
    catch (exception& e)
    {
        cout << e.what() << '\n';
    }
    return 0;
}
*/

/**
 * @file parse_exception.hpp
 * @brief Declares a derived class of the class "std::runtime_error" for throwing
 * error messages when parsing NMEA/SBF
 * @date 17/08/20
 */

/**
 * @class ParseException
 * @date 17/08/20
 * @brief Class to declare error message format when parsing, derived from the public
 * class "std::runtime_error"
 *
 * Such error messages shall be thrown whenever a parser class has an unrecoverable
 * issue parsing a message.. Note that "std::runtime_error" is already a class
 * derived from the base class "exception". Note on "explicit" keyword: When a class
 * has a constructor which can be called with a single argument (since arguments
 * might be set to some values by default), then this constructor becomes a
 * conversion constructor, since it allows the !implicit! conversion of the single
 * argument to the full class. We can avoid such implicit conversions as these may
 * lead to unexpected results by making the constructor explicit with the help of the
 * "explicit" keyword.
 */
class ParseException : public std::runtime_error
{
public:
    explicit ParseException(const std::string& error) : std::runtime_error(error) {}
};

#endif // PARSE_EXCEPTION_HPP
