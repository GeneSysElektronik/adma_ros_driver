#pragma once

#include <iostream>
#include <array>
#include <cstring>
#include <cstdint>
#include <type_traits>
#include <stdexcept>
#include <rclcpp/rclcpp.hpp>
#include <adma_core_lib/parser/data_structs.hpp>
#include <adma_ros_driver_msgs/msg/admanet_header.hpp>

const double PI = 3.1415926535897932384626433832795028841971;

bool getbit(unsigned char byte, int position);

uint8_t getBits(unsigned char byte, int bitOffset, int length);

double getScaledValue(int32_t raw_value, double lsb_factor);

template <typename T>
inline T deg2Rad(T deg){ return T(deg * PI / 180.0); }

template <typename T>
inline T rad2Deg(T rad){ return T(rad * 180.0 / PI); }

bool isLittleEndian();

template<typename T_source>
T_source extractValue(const std::array<char, 856>& buffer, size_t offset, int lengthBytes)
{
        if (lengthBytes > sizeof(T_source))
        {
                RCLCPP_ERROR(rclcpp::get_logger("adma_core_lib::parser_utils"), "Invalid datatype");
        }
        if (offset + lengthBytes > buffer.size()) {
            throw std::out_of_range("Offset + Size is out of buffer range!");
        }

        T_source raw_value{};
        std::memcpy(&raw_value, buffer.data() + offset, lengthBytes);
        return raw_value;
}

void extractAdmanetHeader(adma_ros_driver_msgs::msg::AdmanetHeader &headerMsg, std::array<char, 856>& buffer);
