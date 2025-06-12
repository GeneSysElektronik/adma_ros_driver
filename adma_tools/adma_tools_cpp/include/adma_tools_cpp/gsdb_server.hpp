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

#include <netdb.h>

#include <fstream>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#pragma once

namespace genesys
{
namespace tools
{
class GSDBServer : public rclcpp::Node
{
public:
  explicit GSDBServer(const rclcpp::NodeOptions & options);
  virtual ~GSDBServer();

private:
  void updateLoop();

  // ADMANet specific
  int admanet_send_socket_fd_;
  struct sockaddr_in admanet_socket_address_;
  socklen_t admanet_address_length_;
  int admanet_port_;
  std::string admanet_protocol_version_;
  uint64_t admanet_msgCounter_;
  uint64_t admanet_protocolLength_;
  char admanet_start_pattern[4] = {0x47, 0x42, 0x49, 0x4E};  // GBIN

  // AddOnDelta specific
  int addondelta_send_socket_fd_;
  struct sockaddr_in addondelta_socket_address_;
  socklen_t addondelta_address_length_;
  int addondelta_port_;
  uint64_t addondelta_msgCounter_;
  uint64_t addondelta_protocolLength_;
  unsigned char addondelta_start_pattern[4] = {0xC0, 0xB5, 0x3E, 0x70};

  bool contains_delta_;
  uint16_t frequency_;
  std::string gsdbFilePath_;
  std::fstream gsdbFile_;
};
}  // end namespace tools
}  // end namespace genesys
