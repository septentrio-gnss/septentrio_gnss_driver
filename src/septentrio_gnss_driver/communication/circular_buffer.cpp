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

#include <septentrio_gnss_driver/communication/circular_buffer.hpp>

/**
 * @file circular_buffer.cpp
 * @brief Defines a class for creating, writing and reading from a circular bufffer
 * @date 25/09/20
 */

CircularBuffer::CircularBuffer(ROSaicNodeBase* node, std::size_t capacity) :
    node_(node), head_(0), tail_(0), size_(0), capacity_(capacity)
{
    data_ = new uint8_t[capacity];
}

//! The destructor frees memory (first line) and points the dangling pointer to NULL
//! (second line).
CircularBuffer::~CircularBuffer()
{
    delete[] data_;
    data_ = NULL;
}

std::size_t CircularBuffer::write(const uint8_t* data, std::size_t bytes)
{
    if (bytes == 0)
        return 0;

    std::size_t capacity = capacity_;
    std::size_t bytes_to_write = std::min(bytes, capacity - size_);
    if (bytes_to_write != bytes)
    {
        node_->log(
            LogLevel::ERROR,
            "You are trying to overwrite parts of the circular buffer that have not yet been read!");
    }

    // Writes in a single step
    if (bytes_to_write <= capacity - head_)
    {

        memcpy(data_ + head_, data, bytes_to_write);
        head_ += bytes_to_write;
        if (head_ == capacity)
            head_ = 0;
    }
    // Writes in two steps. Here the circular nature comes to the surface
    else
    {
        std::size_t size_1 = capacity - head_;
        memcpy(data_ + head_, data, size_1);
        std::size_t size_2 = bytes_to_write - size_1;
        memcpy(data_, data + size_1, size_2);
        head_ =
            size_2; // Hence setting head_ = 0 three lines above was not necessary.
    }
    size_ += bytes_to_write;
    return bytes_to_write;
}

std::size_t CircularBuffer::read(uint8_t* data, std::size_t bytes)
{
    if (bytes == 0)
        return 0;
    std::size_t capacity = capacity_;
    std::size_t bytes_to_read = std::min(bytes, size_);
    if (bytes_to_read != bytes)
    {
        node_->log(
            LogLevel::ERROR,
            "You are trying to read parts of the circular buffer that have not yet been written!");
    }

    // Read in a single step
    if (bytes_to_read <=
        capacity - tail_) // Note that it is not size_ - tail_:
                          // If write() hasn't written something into all of capacity
                          // yet (first round of writing), we would still read those
                          // unknown bytes..
    {
        memcpy(data, data_ + tail_, bytes_to_read);
        tail_ += bytes_to_read;
        if (tail_ == capacity)
            tail_ = 0; // Same here?
    }

    // Read in two steps
    else
    {
        std::size_t size_1 = capacity - tail_;
        memcpy(data, data_ + tail_, size_1);
        std::size_t size_2 = bytes_to_read - size_1;
        memcpy(data + size_1, data_, size_2);
        tail_ = size_2;
    }

    size_ -= bytes_to_read;
    return bytes_to_read;
}