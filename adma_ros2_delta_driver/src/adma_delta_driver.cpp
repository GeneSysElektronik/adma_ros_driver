#include "adma_ros2_delta_driver/adma_delta_driver.hpp"

#include <rclcpp_components/register_node_macro.hpp>
#include <adma_core_lib/parser/parser_utils.hpp>

namespace genesys
{
ADMADeltaDriver::ADMADeltaDriver(const rclcpp::NodeOptions & options)
: Node("adma_delta_driver", options)
{
        // define ROS parameters
        std::string param_address = this->declare_parameter("destination_ip", "0.0.0.0");
        int adma_port = this->declare_parameter("destination_port", 1025);
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
                
                // copy UDP packet directly into XML-generated ROS msg
                memcpy(&delta_msg, &recv_buf, sizeof(delta_msg));
                
                // modify individual values where required (e.g. coordinates, LSB factor)
                delta_msg.target_longitude = convertCoordinates(delta_msg.target_longitude);
                delta_msg.target_latitude = convertCoordinates(delta_msg.target_latitude);
                delta_msg.target_forward_velocity = getScaledValue(delta_msg.target_forward_velocity, 0.005);
                delta_msg.hunter_forward_velocity = getScaledValue(delta_msg.hunter_forward_velocity, 0.005);
                delta_msg.target_forward_acceleration = getScaledValue(delta_msg.target_forward_acceleration, 0.005);
                delta_msg.hunter_forward_acceleration = getScaledValue(delta_msg.hunter_forward_acceleration, 0.005);
                delta_msg.target_lateral_velocity = getScaledValue(delta_msg.target_lateral_velocity, 0.005);
                delta_msg.hunter_lateral_velocity = getScaledValue(delta_msg.hunter_lateral_velocity, 0.005);
                delta_msg.target_lateral_acceleration = getScaledValue(delta_msg.target_lateral_acceleration, 0.005);
                delta_msg.hunter_lateral_acceleration = getScaledValue(delta_msg.hunter_lateral_acceleration, 0.005);
                delta_msg.target_pitch_angle = getScaledValue(delta_msg.target_pitch_angle, 0.02);
                delta_msg.hunter_pitch_angle = getScaledValue(delta_msg.hunter_pitch_angle, 0.02);
                pub_delta_->publish(delta_msg);
        }
}

double ADMADeltaDriver::convertCoordinates(double rawValue)
{
        // convert raw double value to bytes
        unsigned char raw_bytes[8];
        std::memcpy(raw_bytes, &rawValue, sizeof(double));
        // extract first 4 bytes as long 
        long part_1 = (raw_bytes[3] << 24) | (raw_bytes[2] << 16) | (raw_bytes[1] << 8) | raw_bytes[0];
        
        // extract last 4 bytes as float for decimal values of coordinates
        float part_2;
        unsigned char float_bytes[4] = {raw_bytes[4], raw_bytes[5], raw_bytes[6], raw_bytes[7]};
        std::memcpy(&part_2, float_bytes, sizeof(part_2));
        
        double coordinate = (static_cast<double>(part_1) + part_2);
        return getScaledValue(coordinate, 0.001);
}

}

RCLCPP_COMPONENTS_REGISTER_NODE(genesys::ADMADeltaDriver)