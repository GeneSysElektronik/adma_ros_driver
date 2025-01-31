#include <rclcpp/rclcpp.hpp>

#include "adma_core_lib/network/udp_socket.hpp"
#include "adma_ros_driver_msgs/msg/delta1170.hpp"

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

                rclcpp::Publisher<adma_ros_driver_msgs::msg::Delta1170>::SharedPtr pub_delta_;
};
}