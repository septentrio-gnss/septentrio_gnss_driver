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

// ROS includes
#include <septentrio_gnss_driver/abstraction/typedefs.hpp>

// C++ library includes
#include <algorithm>
#include <cstdint>

#ifndef CIRCULAR_BUFFER_HPP
#define CIRCULAR_BUFFER_HPP

/**
 * @file circular_buffer.hpp
 * @brief Declares a class for creating, writing to and reading from a circular
 * bufffer
 * @date 25/09/20
 */

/**
 * @class CircularBuffer
 * @brief Class for creating, writing to and reading from a circular buffer
 */
class CircularBuffer
{
public:
    //! Constructor of CircularBuffer
    explicit CircularBuffer(ROSaicNodeBase* node, std::size_t capacity);
    //! Destructor of CircularBuffer
    ~CircularBuffer();
    //! Returns size_
    std::size_t size() const { return size_; }
    //! Returns capacity_
    std::size_t capacity() const { return capacity_; }
    //! Returns number of bytes written.
    std::size_t write(const uint8_t* data, std::size_t bytes);
    //! Returns number of bytes read.
    std::size_t read(uint8_t* data, std::size_t bytes);

private:
    //! Pointer to the node
    ROSaicNodeBase* node_;
    //! Specifies where we start writing
    std::size_t head_;
    //! Specifies where we start reading
    std::size_t tail_;
    //! Number of bytes that have been written but not yet read
    std::size_t size_;
    //! Capacity of the circular buffer
    std::size_t capacity_;
    //! Pointer that always points to the same memory address, hence could be const
    //! pointer
    uint8_t* data_;
};

#endif // for CIRCULAR_BUFFER_HPP