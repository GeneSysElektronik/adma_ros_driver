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

#include <string>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rosgraph_msgs/msg/clock.hpp>

#include <adma_ros_driver_msgs/msg/adma_data.hpp>
#include <adma_ros_driver_msgs/msg/adma_data_raw.hpp>
#include <adma_ros_driver_msgs/msg/adma_data_scaled.hpp>
#include <adma_ros_driver_msgs/msg/adma_status.hpp>
#include <adma_core_lib/network/udp_socket.hpp>

#include "adma_ros2_driver/parser/adma2ros_parser.hpp"

#pragma once

namespace genesys
{
class ADMADriver : public rclcpp::Node
{
public:
  explicit ADMADriver(const rclcpp::NodeOptions & options);
  virtual ~ADMADriver();

private:
  void updateLoop();
  void parseData(std::array<char, 856> recv_buf);
  void rawDataCallback(adma_ros_driver_msgs::msg::AdmaDataRaw::SharedPtr rawDataMsg);

  genesys::core::UDPSocket * socket_;
  size_t len_ = 0;
  /** \brief Check the timings */
  bool performance_check_ = true;
  bool setupDone = false;

  // subscriber
  rclcpp::Subscription<adma_ros_driver_msgs::msg::AdmaDataRaw>::SharedPtr subRawData_;

  // publisher
  rclcpp::Publisher<adma_ros_driver_msgs::msg::AdmaData>::SharedPtr pub_adma_data_;
  rclcpp::Publisher<adma_ros_driver_msgs::msg::AdmaDataRaw>::SharedPtr pub_adma_data_raw_;
  rclcpp::Publisher<adma_ros_driver_msgs::msg::AdmaDataScaled>::SharedPtr pub_adma_data_scaled_;
  rclcpp::Publisher<adma_ros_driver_msgs::msg::AdmaStatus>::SharedPtr pub_adma_status_;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr pub_navsat_fix_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_heading_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_velocity_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odometry_;
  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr pub_clock_;

  // frame_ids for the ros msgs
  std::string gnss_frame_;
  std::string imu_frame_;
  std::string adma_frame_;
  std::string adma_status_frame_;
  std::string raw_data_frame_;
  std::string odometry_pose_frame_;
  std::string odometry_child_frame_;

  ADMA2ROSParser * parser_;

  // yaw offset angle if the odometry should be rotated by a fixed angle
  double odometry_yaw_offset_;

  // desired data sources per topic (POI_x or MRP)
  uint8_t navsatfix_id_;
  uint8_t imu_id_;
  uint8_t velocity_id_;
  uint8_t odometry_id_;
  std::array<adma_ros_driver_msgs::msg::POI, 8> pois;

  // parameters for mode and time source
  uint8_t mode_;
  uint8_t time_mode_;
  bool publish_clock_;
};
}  // namespace genesys
