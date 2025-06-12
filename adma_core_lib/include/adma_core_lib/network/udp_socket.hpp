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

#include <iostream>
#include <string>

#include <rclcpp/rclcpp.hpp>

#pragma once

namespace genesys
{
namespace core
{
class UDPSocket
{
public:
  explicit UDPSocket(size_t buffer_length);
  virtual ~UDPSocket();

  void setupReceiveSocket(std::string adma_address, int adma_port);

  template<std::size_t Buffersize>
  void receiveUDPPacket(std::array<char, Buffersize> & recv_buf)
  {
    // check if new data is available
    FD_ZERO(&s);
    FD_SET(rcv_sock_fd_, &s);
    timeout.tv_sec = 1;
    timeout.tv_usec = 0;
    int ret = select(rcv_sock_fd_ + 1, &s, NULL, NULL, &timeout);
    if (ret == 0) {
      // reached timeout
      RCLCPP_INFO(rclcpp::get_logger("UDP-Socket"), "Waiting for ADMA data...");
      return;
    } else if (ret == -1) {
      // error
      RCLCPP_WARN(rclcpp::get_logger("UDP-Socket"), "Select-error: %s", strerror(errno));
      return;
    }

    ret = ::recv(rcv_sock_fd_, reinterpret_cast<void *>(&recv_buf), len_, 0);
    if (ret < 0) {
      RCLCPP_WARN(rclcpp::get_logger("UDP-Socket"), "Receive-error: %s", strerror(errno));
      return;
    } else if (ret != len_) {
      RCLCPP_WARN(
        rclcpp::get_logger("UDP-Socket"), "Invalid ADMA message size: %d instead of %ld",
        ret, len_);
      return;
    }
  }

  void setupSendingSocket(std::string socket_adress, int port);

  void sendUDPPacket(char buffer[856]);

private:
  // general attributes
  // Adma  socket address length
  socklen_t address_length_;
  ssize_t len_ = 0;

  // attributes for receiving UDP packets
  struct sockaddr_in rcv_socket_address_;
  // Address info for receiving from adma
  struct addrinfo * rcv_addr_info_;
  // Socket file descriptor for receiving from adma
  int rcv_sock_fd_;
  fd_set s;
  struct timeval timeout;
  // struct sockaddr src_addr;
  // socklen_t src_addr_len;

  // attributes for sending UDP packets
  struct sockaddr_in send_socket_address_;
  int send_socket_fd_;
};
}  // namespace core
}  // namespace genesys
