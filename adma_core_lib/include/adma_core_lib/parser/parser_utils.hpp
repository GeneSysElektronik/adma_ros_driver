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
      
template<typename T_source>
T_source extractValue(const std::array<char, 856>& buffer, size_t offset, int lengthBytes, bool sourceLittleEndian = true)
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

// template<typename T_dest>
// T_dest extractValue(const std::array<char, 856>& buffer, genesys::DataChannel dataChannel)
// {
//         // first check if ROS datatype fits for current data channel
//         if (dataChannel.lengthBits > sizeof(T_dest))
//         {
//                 RCLCPP_ERROR(rclcpp::get_logger("adma_core_lib::parser_utils"), "Invalid datatype for Channel %s", dataChannel.name.c_str());
//         }

//         switch (dataChannel.dataType)
//         {
//         case genesys::ADMADataType::UINT:
//                 if(dataChannel.minRange < 0)
//                 {
//                         RCLCPP_ERROR(rclcpp::get_logger("adma_core_lib::parser_utils"), "Datatype of %s is UINT but Min-Range is lower 0..", dataChannel.name.c_str());
//                 }
//                 if(dataChannel.lengthBits == 8) return static_cast<T_dest>(extractValue<uint8_t>(buffer, dataChannel, dataChannel.scale != 1.0));
//                 if(dataChannel.lengthBits == 16) return static_cast<T_dest>(extractValue<uint16_t>(buffer, dataChannel, dataChannel.scale != 1.0));
//                 if(dataChannel.lengthBits == 32) return static_cast<T_dest>(extractValue<uint32_t>(buffer, dataChannel, dataChannel.scale != 1.0));
//                 RCLCPP_ERROR(rclcpp::get_logger("adma_core_lib::parser_utils"), "unhandled case for %s", dataChannel.name.c_str());
//                 break;
//         case genesys::ADMADataType::INT:
//                 if(dataChannel.minRange > 0)
//                 {
//                         RCLCPP_ERROR(rclcpp::get_logger("adma_core_lib::parser_utils"), "Datatype of %s is INT but Min-Range is limited to 0..", dataChannel.name.c_str());
//                 }
//                 if(dataChannel.lengthBits == 8) return static_cast<T_dest>(extractValue<int8_t>(buffer, dataChannel, dataChannel.scale != 1.0));
//                 if(dataChannel.lengthBits == 16) return static_cast<T_dest>(extractValue<int16_t>(buffer, dataChannel, dataChannel.scale != 1.0));
//                 if(dataChannel.lengthBits == 32) return static_cast<T_dest>(extractValue<int32_t>(buffer, dataChannel, dataChannel.scale != 1.0));
//                 RCLCPP_ERROR(rclcpp::get_logger("adma_core_lib::parser_utils"), "unhandled case for %s", dataChannel.name.c_str());
//                 break;
//         default:
//                 RCLCPP_ERROR(rclcpp::get_logger("adma_core_lib::parser_utils"), "unhandled case for %s", dataChannel.name.c_str());
//                 break;
//         }
// }

// template<typename T_source>
// T_source extractValue(const std::array<char, 856>& buffer, genesys::DataChannel dataChannel, bool scale)
// {
//         if (dataChannel.byteOffset + dataChannel.lengthBits > buffer.size()) {
//             throw std::out_of_range("Offset + Size is out of buffer range!");
//         }
    
//         T_source raw_value{};
//         std::memcpy(&raw_value, buffer.data() + dataChannel.byteOffset, dataChannel.lengthBits);
//         if(scale)
//         {
//                 ret
//         }    
//         return raw_value;
// }

void extractAdmanetHeader(adma_ros_driver_msgs::msg::AdmanetHeader &headerMsg, std::array<char, 856>& buffer);