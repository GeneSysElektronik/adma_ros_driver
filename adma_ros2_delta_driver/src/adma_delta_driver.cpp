#include "adma_ros2_delta_driver/adma_delta_driver.hpp"

#include <rclcpp_components/register_node_macro.hpp>

namespace genesys
{
ADMADeltaDriver::ADMADeltaDriver(const rclcpp::NodeOptions & options)
: Node("adma_delta_driver", options)
{
        // define ROS parameters
        std::string param_address = this->declare_parameter("destination_ip", "0.0.0.0");
        int adma_port = this->declare_parameter("destination_port", 1040);
        protocol_version_ = this->declare_parameter("protocol_version", "v7.0");
        
        // setup publisher
        pub_delta_ = this->create_publisher<adma_ros_driver_msgs::msg::Delta1170>("adma/delta", 1);

        // setup UDP socket communication
        len_ = 88;
        socket_ = new genesys::core::UDPSocket(len_);
        socket_->setupReceiveSocket(param_address, adma_port);
        updateLoop();
}

ADMADeltaDriver::~ADMADeltaDriver()
{
  // unlock socket when stopping application
    socket_->~UDPSocket();
}

void ADMADeltaDriver::updateLoop()
{
        adma_ros_driver_msgs::msg::Delta1170 delta_msg;
        std::array<char, 88> recv_buf;
        while(rclcpp::ok()) {
                socket_->receiveUDPPacket(recv_buf);
                memcpy(&delta_msg, &recv_buf, sizeof(delta_msg));
                pub_delta_->publish(delta_msg);
        }
}

}

RCLCPP_COMPONENTS_REGISTER_NODE(genesys::ADMADeltaDriver)