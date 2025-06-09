// Copyright (c) 2023, GeneSys Elektronik
//
// Licensed under the 3-Clause BSD License
// SPDX-License-Identifier: 3-Clause BSD License
//
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

#include "adma_core_lib/network/udp_socket.hpp"

#include <arpa/inet.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <sys/types.h>

namespace genesys
{
namespace core
{
UDPSocket::UDPSocket(size_t buffer_length)
{
  RCLCPP_INFO(rclcpp::get_logger("UDP-Socket"), "setup Socket with length: %ld", buffer_length);
  rcv_sock_fd_ = -1;
  rcv_addr_info_ = NULL;
  address_length_ = 4;
  len_ = buffer_length;
  send_socket_fd_ = -1;
}

UDPSocket::~UDPSocket()
{
  freeaddrinfo(rcv_addr_info_);
  ::shutdown(rcv_sock_fd_, SHUT_RDWR);
  rcv_sock_fd_ = -1;
  ::shutdown(send_socket_fd_, SHUT_RDWR);
  send_socket_fd_ = -1;
}

void UDPSocket::setupReceiveSocket(std::string adma_address, int adma_port)
{
  // setup socket
  struct addrinfo hints;
  memset(&hints, 0, sizeof(hints));
  hints.ai_family = AF_UNSPEC;
  hints.ai_socktype = SOCK_DGRAM;
  hints.ai_protocol = IPPROTO_UDP;
  std::string rcv_port_str = std::to_string(adma_port);

  address_length_ = sizeof(rcv_socket_address_);
  memset(reinterpret_cast<char *>(&rcv_socket_address_), 0, address_length_);
  rcv_socket_address_.sin_family = AF_INET;
  rcv_socket_address_.sin_port = htons(adma_port);
  inet_aton(adma_address.c_str(), &(rcv_socket_address_.sin_addr));

  // define some error handling
  int r = getaddrinfo(adma_address.c_str(), rcv_port_str.c_str(), &hints, &rcv_addr_info_);
  if (r != 0 || rcv_addr_info_ == NULL) {
    RCLCPP_FATAL(
      rclcpp::get_logger("UDP-Socket"), "Invalid port for UDP socket: \"%s:%s\"",
      adma_address.c_str(), rcv_port_str.c_str());
    throw rclcpp::exceptions::InvalidParameterValueException(
      "Invalid port for UDP socket: \"" + adma_address + ":" + rcv_port_str + "\"");
  }
  rcv_sock_fd_ = socket(rcv_addr_info_->ai_family, SOCK_DGRAM | SOCK_CLOEXEC, IPPROTO_UDP);
  if (rcv_sock_fd_ == -1) {
    freeaddrinfo(rcv_addr_info_);
    RCLCPP_FATAL(
      rclcpp::get_logger("UDP-Socket"), "Could not create UDP socket for: \"%s:%s",
      adma_address.c_str(), rcv_port_str.c_str());
    throw rclcpp::exceptions::InvalidParameterValueException(
        "Could not create UDP socket for: \"" + adma_address + ":" + rcv_port_str + "\"");
  }
  r = bind(rcv_sock_fd_, rcv_addr_info_->ai_addr, rcv_addr_info_->ai_addrlen);
  if (r != 0) {
    freeaddrinfo(rcv_addr_info_);
    ::shutdown(rcv_sock_fd_, SHUT_RDWR);
    RCLCPP_FATAL(
      rclcpp::get_logger("UDP-Socket"), "Could not bind UDP socket with: \"%s:%s",
      adma_address.c_str(), rcv_port_str.c_str());
    throw rclcpp::exceptions::InvalidParameterValueException(
        "Could not bind UDP socket with: \"" + adma_address + ":" + rcv_port_str + "\"");
  }

  RCLCPP_INFO(
    rclcpp::get_logger("UDP-Socket"), "Try opening UDP socket with: \"%s:%s",
    adma_address.c_str(), rcv_port_str.c_str());
}

void UDPSocket::setupSendingSocket(std::string socket_adress, int port)
{
  send_socket_fd_ = socket(AF_INET, SOCK_DGRAM | SOCK_CLOEXEC, IPPROTO_UDP);
  address_length_ = sizeof(send_socket_address_);
  memset(reinterpret_cast<char *>(&send_socket_address_), 0, address_length_);
  send_socket_address_.sin_family = AF_INET;
  send_socket_address_.sin_port = htons(port);
  inet_aton(socket_adress.c_str(), &(send_socket_address_.sin_addr));
}

void UDPSocket::sendUDPPacket(char buffer[856])
{
  ::sendto(
    send_socket_fd_, reinterpret_cast<void *>(&buffer), len_, 0,
    (struct sockaddr *)&send_socket_address_, address_length_);
}

}  // namespace core
}  // namespace genesys
