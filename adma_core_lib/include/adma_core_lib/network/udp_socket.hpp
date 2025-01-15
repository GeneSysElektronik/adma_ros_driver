#include <netdb.h>
#include <iostream>

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

                void initializeUDP(std::string adma_address, int adma_port);
                
                template <std::size_t Buffersize>
                void updateLoop(std::array<char, Buffersize>& recv_buf)
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

                        ret = ::recv(rcv_sock_fd_, (void *)(&recv_buf), len_, 0);
                        if (ret < 0) {
                                RCLCPP_WARN(rclcpp::get_logger("UDP-Socket"), "Receive-error: %s", strerror(errno));
                                return;
                        } else if (ret != len_) {
                                RCLCPP_WARN(rclcpp::get_logger("UDP-Socket"), "Invalid ADMA message size: %d instead of %ld", ret, len_);
                                return;
                        }
                }

        private:
                // Address info for receiving from adma
                struct addrinfo * rcv_addr_info_;
                // adma socket address
                struct sockaddr_in adma_address_;
                //Adma  socket address length
                socklen_t adma_address_length_;
                size_t len_ = 0;
                // Socket file descriptor for receiving from adma
                int rcv_sock_fd_;
                fd_set s;
                struct timeval timeout;
                // struct sockaddr src_addr;
                // socklen_t src_addr_len;

};
} // namespace core
} // namespace genesys