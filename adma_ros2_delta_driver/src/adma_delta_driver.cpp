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
        pub_delta_raw_ = this->create_publisher<adma_ros_driver_msgs::msg::Delta1170Raw>("adma/delta_raw", 1);
        pub_delta_scaled_ = this->create_publisher<adma_ros_driver_msgs::msg::Delta1170Scaled>("adma/delta_scaled", 1);

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
        adma_ros_driver_msgs::msg::Delta1170Raw delta_msg_raw;
        adma_ros_driver_msgs::msg::Delta1170Scaled delta_msg_scaled;
        std::array<char, 88> recv_buf;
        while(rclcpp::ok()) {
                socket_->receiveUDPPacket(recv_buf);
                
                // copy UDP packet directly into XML-generated ROS msg
                memcpy(&delta_msg_raw, &recv_buf, sizeof(delta_msg_raw));
                
                
                // fill msg header for scaled msg
                delta_msg_scaled.header.frame_id = "addondelta";
                // TODO: may fill timestamp with data based on delta input
                delta_msg_scaled.header.stamp = get_clock()->now();
                // fill scaled msg with content
                delta_msg_scaled.abd_header = delta_msg_raw.abd_header;
                delta_msg_scaled.code_version = delta_msg_raw.code_version;
                delta_msg_scaled.long_delta_distance = delta_msg_raw.long_delta_distance;
                delta_msg_scaled.long_delta_velocity = delta_msg_raw.long_delta_velocity;
                delta_msg_scaled.lat_delta_distance = delta_msg_raw.lat_delta_distance;
                delta_msg_scaled.lat_delta_velocity = delta_msg_raw.lat_delta_velocity;
                delta_msg_scaled.resultant_distance = delta_msg_raw.resultant_distance;
                delta_msg_scaled.resultant_velocity = delta_msg_raw.resultant_velocity;
                delta_msg_scaled.angle_of_orientation = delta_msg_raw.angle_of_orientation;
                delta_msg_scaled.delta_time = delta_msg_raw.delta_time;
                delta_msg_scaled.target_status = delta_msg_raw.target_status;
                delta_msg_scaled.hunter_status = delta_msg_raw.hunter_status;

                // modify individual values where required (e.g. coordinates, LSB factor)
                delta_msg_scaled.target_longitude = convertCoordinates(delta_msg_raw.target_longitude);
                delta_msg_scaled.target_latitude = convertCoordinates(delta_msg_raw.target_latitude);
                delta_msg_scaled.target_forward_velocity = getScaledValue(delta_msg_raw.target_forward_velocity, 0.005);
                delta_msg_scaled.hunter_forward_velocity = getScaledValue(delta_msg_raw.hunter_forward_velocity, 0.005);
                delta_msg_scaled.target_forward_acceleration = getScaledValue(delta_msg_raw.target_forward_acceleration, 0.005);
                delta_msg_scaled.hunter_forward_acceleration = getScaledValue(delta_msg_raw.hunter_forward_acceleration, 0.005);
                delta_msg_scaled.target_lateral_velocity = getScaledValue(delta_msg_raw.target_lateral_velocity, 0.005);
                delta_msg_scaled.hunter_lateral_velocity = getScaledValue(delta_msg_raw.hunter_lateral_velocity, 0.005);
                delta_msg_scaled.target_lateral_acceleration = getScaledValue(delta_msg_raw.target_lateral_acceleration, 0.005);
                delta_msg_scaled.hunter_lateral_acceleration = getScaledValue(delta_msg_raw.hunter_lateral_acceleration, 0.005);
                delta_msg_scaled.target_pitch_angle = getScaledValue(delta_msg_raw.target_pitch_angle, 0.02);
                delta_msg_scaled.hunter_pitch_angle = getScaledValue(delta_msg_raw.hunter_pitch_angle, 0.02);
                
                // publish both msgs
                pub_delta_raw_->publish(delta_msg_raw);
                pub_delta_scaled_->publish(delta_msg_scaled);
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