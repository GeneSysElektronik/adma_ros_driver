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

#include <adma_ros_driver_msgs/msg/delta1170_raw.hpp>
#include <adma_ros_driver_msgs/msg/delta1170_scaled.hpp>
#include <adma_core_lib/network/udp_socket.hpp>
#include <adma_core_lib/parser/mapping.hpp>

#pragma once

namespace genesys
{
class ADMADeltaDriver : public rclcpp::Node
{
public:
  explicit ADMADeltaDriver(const rclcpp::NodeOptions & options);
  virtual ~ADMADeltaDriver();

private:
  void updateLoop();
  double convertCoordinates(double rawValue);

  std::string protocol_version_;
  size_t len_ = 0;
  genesys::core::UDPSocket * socket_;

  genesys::parser::Mapping * mapping_;

  rclcpp::Publisher<adma_ros_driver_msgs::msg::Delta1170Raw>::SharedPtr pub_delta_raw_;
  rclcpp::Publisher<adma_ros_driver_msgs::msg::Delta1170Scaled>::SharedPtr pub_delta_scaled_;
};
}  // namespace genesys
