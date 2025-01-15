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
        rcv_sock_fd_ = -1;
        rcv_addr_info_ = NULL;
        adma_address_length_ = 4;
        len_ = buffer_length;
}

UDPSocket::~UDPSocket()
{
        freeaddrinfo(rcv_addr_info_);
        ::shutdown(rcv_sock_fd_, SHUT_RDWR);
        rcv_sock_fd_ = -1;
}

void UDPSocket::initializeUDP(std::string adma_address, int adma_port)
{
        // setup socket
        struct addrinfo hints;
        memset(&hints, 0, sizeof(hints));
        hints.ai_family = AF_UNSPEC;
        hints.ai_socktype = SOCK_DGRAM;
        hints.ai_protocol = IPPROTO_UDP;
        std::string rcv_port_str = std::to_string(adma_port);

        adma_address_length_ = sizeof(adma_address_);
        memset((char *)&adma_address_, 0, adma_address_length_);
        adma_address_.sin_family = AF_INET;
        adma_address_.sin_port = htons(adma_port);
        inet_aton(adma_address.c_str(), &(adma_address_.sin_addr));

        // define some error handling
        int r = getaddrinfo(adma_address.c_str(), rcv_port_str.c_str(), &hints, &rcv_addr_info_);
        if (r != 0 || rcv_addr_info_ == NULL) {
        RCLCPP_FATAL(
        rclcpp::get_logger("UDP-Socket"), "Invalid port for UDP socket: \"%s:%s\"", adma_address.c_str(),
        rcv_port_str.c_str());
        throw rclcpp::exceptions::InvalidParameterValueException(
        "Invalid port for UDP socket: \"" + adma_address + ":" + rcv_port_str + "\"");
        }
        rcv_sock_fd_ = socket(rcv_addr_info_->ai_family, SOCK_DGRAM | SOCK_CLOEXEC, IPPROTO_UDP);
        if (rcv_sock_fd_ == -1) {
        freeaddrinfo(rcv_addr_info_);
        RCLCPP_FATAL(
        rclcpp::get_logger("UDP-Socket"), "Could not create UDP socket for: \"%s:%s", adma_address.c_str(),
        rcv_port_str.c_str());
        throw rclcpp::exceptions::InvalidParameterValueException(
        "Could not create UDP socket for: \"" + adma_address + ":" + rcv_port_str + "\"");
        }
        r = bind(rcv_sock_fd_, rcv_addr_info_->ai_addr, rcv_addr_info_->ai_addrlen);
        if (r != 0) {
        freeaddrinfo(rcv_addr_info_);
        ::shutdown(rcv_sock_fd_, SHUT_RDWR);
        RCLCPP_FATAL(
        rclcpp::get_logger("UDP-Socket"), "Could not bind UDP socket with: \"%s:%s", adma_address.c_str(),
        rcv_port_str.c_str());
        throw rclcpp::exceptions::InvalidParameterValueException(
        "Could not bind UDP socket with: \"" + adma_address + ":" + rcv_port_str + "\"");
        }

        RCLCPP_INFO(
        rclcpp::get_logger("UDP-Socket"), "Try opening UDP socket with: \"%s:%s", adma_address.c_str(),
        rcv_port_str.c_str());
}

}
}