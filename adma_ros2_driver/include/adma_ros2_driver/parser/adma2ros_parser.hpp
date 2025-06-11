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

#pragma once

#include <iostream>
#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float64.hpp>

#include "adma_ros2_driver/data/adma_data_v32.hpp"
#include "adma_ros2_driver/parser/adma2ros_parser_v32.hpp"
#include "adma_ros_driver_msgs/msg/adma_data.hpp"
#include "adma_ros_driver_msgs/msg/adma_data_scaled.hpp"
#include "adma_ros_driver_msgs/msg/adma_status.hpp"
#include "adma_core_lib/parser/mapping.hpp"

class ADMA2ROSParser
{
public:
  explicit ADMA2ROSParser(u_int16_t version);
  ~ADMA2ROSParser() {}
  // void findMappingFiles(std::string & protocolFileName, std::string & glossarFileName);
  void mapAdmaMessageToROS(
    adma_ros_driver_msgs::msg::AdmaData & ros_msg, std::array<char, 856> & recv_data);
  void extractNavSatFix(
    adma_ros_driver_msgs::msg::AdmaData & ros_msg, sensor_msgs::msg::NavSatFix & nav_ros_msg);
  void extractNavSatFix(
    adma_ros_driver_msgs::msg::AdmaDataScaled & ros_msg, sensor_msgs::msg::NavSatFix & nav_ros_msg,
    std::array<adma_ros_driver_msgs::msg::POI, 8> & pois, uint8_t desiredSource);
  void extractIMU(
    adma_ros_driver_msgs::msg::AdmaData & ros_msg, sensor_msgs::msg::Imu & imu_ros_msg);
  void extractIMU(
    adma_ros_driver_msgs::msg::AdmaDataScaled & ros_msg, sensor_msgs::msg::Imu & imu_ros_msg,
    std::array<adma_ros_driver_msgs::msg::POI, 8> & pois, uint8_t desiredSource);
  void extractOdometry(
    adma_ros_driver_msgs::msg::AdmaDataScaled & ros_msg,
    nav_msgs::msg::Odometry & odometry_msg, double yawOffset,
    std::array<adma_ros_driver_msgs::msg::POI, 8> & pois, uint8_t desiredSource);
  void parseScaledData(adma_ros_driver_msgs::msg::AdmaData & ros_msg);
  ADMA2ROSParserV32 * parserV32_;
  genesys::parser::Mapping * mapping_;
  void extractHeading(std_msgs::msg::Float64 & headingMsg, std::array<char, 856> & recv_data);
  void extractAdmaStatus(
    adma_ros_driver_msgs::msg::AdmaStatus & statusMsg, std::array<char,
    856> & recv_data);
  void extractAdmaDataScaled(
    adma_ros_driver_msgs::msg::AdmaDataScaled & admaScaledMsg,
    std::array<char, 856> & recv_data);
  void extractPOIs(
    adma_ros_driver_msgs::msg::AdmaDataScaled & admaScaledMsg, std::array<char,
    856> & recv_data);

private:
  void getStatusGPS(adma_ros_driver_msgs::msg::AdmaData & ros_msg, unsigned char gps_status);
  void getStatusTrigger(
    adma_ros_driver_msgs::msg::AdmaData & ros_msg, unsigned char gps_trigger_status);
  void getEVKStatus(adma_ros_driver_msgs::msg::AdmaData & ros_msg, unsigned char evk_status);
  void getErrorandWarning(
    adma_ros_driver_msgs::msg::AdmaData & ros_msg, unsigned char adma_data[4]);

  uint16_t protocolVersion_;
};
