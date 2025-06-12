// BSD 3-Clause License
// Copyright (c) 2023, GeneSys Elektronik
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// 1. Redistributions of source code must retain the above copyright notice, this
//    list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from
//    this software without specific prior written permission.
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include <array>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <stdexcept>
#include <type_traits>

#include <rclcpp/rclcpp.hpp>
#include <adma_core_lib/parser/data_structs.hpp>
#include <adma_ros_driver_msgs/msg/admanet_header.hpp>

#pragma once

const double PI = 3.1415926535897932384626433832795028841971;

bool getbit(unsigned char byte, int position);

uint8_t getBits(unsigned char byte, int bitOffset, int length);

double getScaledValue(int32_t raw_value, double lsb_factor);

template<typename T>
inline T deg2Rad(T deg)
{
  return T(deg * PI / 180.0);
}

template<typename T>
inline T rad2Deg(T rad)
{
  return T(rad * 180.0 / PI);
}

bool isLittleEndian();

template<typename T_source>
T_source extractValue(const std::array<char, 856> & buffer, size_t offset, size_t lengthBytes)
{
  if (lengthBytes > sizeof(T_source)) {
    RCLCPP_ERROR(rclcpp::get_logger("adma_core_lib::parser_utils"), "Invalid datatype");
  }
  if (offset + lengthBytes > buffer.size()) {
    throw std::out_of_range("Offset + Size is out of buffer range!");
  }

  T_source raw_value{};
  std::memcpy(&raw_value, buffer.data() + offset, lengthBytes);
  return raw_value;
}

void extractAdmanetHeader(
  adma_ros_driver_msgs::msg::AdmanetHeader & headerMsg, std::array<char, 856> & buffer);
