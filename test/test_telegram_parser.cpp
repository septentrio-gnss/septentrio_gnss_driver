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

#include <gtest/gtest.h>

#include <septentrio_gnss_driver/communication/telegram_parser.hpp>

TEST(ClassifySyncTest, second_byte_table)
{
    EXPECT_EQ(telegram_parser::classifySync2('@'), telegram_type::SBF);
    EXPECT_EQ(telegram_parser::classifySync2('G'), telegram_type::NMEA);
    EXPECT_EQ(telegram_parser::classifySync2('I'), telegram_type::NMEA_INS);
    EXPECT_EQ(telegram_parser::classifySync2('R'), telegram_type::RESPONSE);
    EXPECT_EQ(telegram_parser::classifySync2('$'), telegram_type::UNKNOWN);
    EXPECT_EQ(telegram_parser::classifySync2('g'), telegram_type::UNKNOWN);
    EXPECT_EQ(telegram_parser::classifySync2(0x00), telegram_type::UNKNOWN);
}

TEST(ClassifySyncTest, third_byte_sets)
{
    for (uint8_t byte3 : {'P', 'N', 'A', 'B', 'L'})
        EXPECT_TRUE(telegram_parser::isNmeaSync3(byte3));
    EXPECT_FALSE(telegram_parser::isNmeaSync3(':'));
    EXPECT_FALSE(telegram_parser::isNmeaSync3('Q'));

    EXPECT_TRUE(telegram_parser::isNmeaInsSync3('N'));
    EXPECT_FALSE(telegram_parser::isNmeaInsSync3('P'));

    EXPECT_TRUE(telegram_parser::isResponseSync3(':'));
    EXPECT_TRUE(telegram_parser::isResponseSync3('!'));
    EXPECT_FALSE(telegram_parser::isResponseSync3('?'));

    EXPECT_TRUE(telegram_parser::isErrorSync3('?'));
    EXPECT_FALSE(telegram_parser::isErrorSync3(':'));
}

TEST(SbfLengthTest, extraction_and_validity)
{
    uint8_t header[8] = {'$', '@', 0x00, 0x00, 0x00, 0x00, 0x0C, 0x00};
    EXPECT_EQ(telegram_parser::getSbfLength(header), 12);
    header[6] = 0xFF;
    header[7] = 0xFF;
    EXPECT_EQ(telegram_parser::getSbfLength(header), 65535);

    EXPECT_FALSE(telegram_parser::isValidSbfLength(4));
    EXPECT_TRUE(telegram_parser::isValidSbfLength(8));
    EXPECT_TRUE(telegram_parser::isValidSbfLength(12));
    EXPECT_FALSE(telegram_parser::isValidSbfLength(13));
    EXPECT_TRUE(telegram_parser::isValidSbfLength(65532));
    EXPECT_FALSE(telegram_parser::isValidSbfLength(65535));
}

TEST(NmeaEndTest, crlf)
{
    EXPECT_TRUE(telegram_parser::isNmeaEnd(CR, LF));
    EXPECT_FALSE(telegram_parser::isNmeaEnd(LF, LF));
    EXPECT_FALSE(telegram_parser::isNmeaEnd(CR, CR));
    EXPECT_FALSE(telegram_parser::isNmeaEnd('A', LF));
}
