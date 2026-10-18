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

#include <clocale>
#include <iostream>
#include <locale>

#include <gtest/gtest.h>
#include <septentrio_gnss_driver/parsers/string_utilities.hpp>

TEST(TrimTest, string_trimming)
{
    {
        double val = 0.333333333;
        auto str = string_utilities::trimDecimalPlaces(val);

        EXPECT_EQ(str.size(), 5);
    }
    {

        double val = 0.0;
        auto str = string_utilities::trimDecimalPlaces(val);

        EXPECT_EQ(str.size(), 5);
    }
    {

        double val = 100.333333333;
        auto str = string_utilities::trimDecimalPlaces(val);

        EXPECT_EQ(str.size(), 7);
    }
    {
        double val = 100.0;
        auto str = string_utilities::trimDecimalPlaces(val);

        EXPECT_EQ(str.size(), 7);
    }
}

namespace {
    class CommaNumpunct : public std::numpunct<char>
    {
    protected:
        char do_decimal_point() const override { return ','; }
    };
} // namespace

TEST(LocaleTest, comma_decimal_locale_independence)
{
    std::locale previousGlobal =
        std::locale::global(std::locale(std::locale(), new CommaNumpunct()));
    char* previousCLocale = std::setlocale(LC_NUMERIC, nullptr);
    std::string previousCLocaleName =
        previousCLocale ? previousCLocale : std::string("C");
    bool hasCommaCLocale =
        (std::setlocale(LC_NUMERIC, "de_DE.UTF-8") != nullptr) ||
        (std::setlocale(LC_NUMERIC, "de_DE.utf8") != nullptr);

    EXPECT_EQ(string_utilities::trimDecimalPlaces(0.5), "0.500");

    double d = 0.0;
    EXPECT_TRUE(string_utilities::toDouble("4717.11399", d));
    EXPECT_DOUBLE_EQ(d, 4717.11399);
    EXPECT_FALSE(string_utilities::toDouble("4717,11399", d));

    float f = 0.0f;
    EXPECT_TRUE(string_utilities::toFloat("45.5", f));
    EXPECT_FLOAT_EQ(f, 45.5f);

    if (!hasCommaCLocale)
        std::cout << "de_DE locale not installed, C locale part not exercised"
                  << std::endl;

    std::setlocale(LC_NUMERIC, previousCLocaleName.c_str());
    std::locale::global(previousGlobal);
}
TEST(ToUInt32Test, range_and_sign)
{
    uint32_t val = 0;
    EXPECT_TRUE(string_utilities::toUInt32("4294967295", val));
    EXPECT_EQ(val, 4294967295u);
    EXPECT_TRUE(string_utilities::toUInt32("2147483648", val));
    EXPECT_EQ(val, 2147483648u);
    EXPECT_FALSE(string_utilities::toUInt32("4294967296", val));
    EXPECT_FALSE(string_utilities::toUInt32("-1", val));
    EXPECT_FALSE(string_utilities::toUInt32("12ab", val));
    EXPECT_FALSE(string_utilities::toUInt32("", val));
}
