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

                void setupReceiveSocket(std::string adma_address, int adma_port);
                
                template <std::size_t Buffersize>
                void receiveUDPPacket(std::array<char, Buffersize>& recv_buf)
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

                void setupSendingSocket(std::string socket_adress, int port);

                void sendUDPPacket(char buffer[856]);

        private:
                // general attributes
                //Adma  socket address length
                socklen_t address_length_;
                size_t len_ = 0;
                
                //attributes for receiving UDP packets
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
} // namespace core
} // namespace genesys