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
        protocol_version_ = this->declare_parameter("protocol_version", "11700");

        // setup publisher
        pub_delta_raw_ = this->create_publisher<adma_ros_driver_msgs::msg::Delta1170Raw>("adma/delta_raw", 1);
        pub_delta_scaled_ = this->create_publisher<adma_ros_driver_msgs::msg::Delta1170Scaled>("adma/delta_scaled", 1);

        // setup UDP socket communication
        len_ = 88;
        socket_ = new genesys::core::UDPSocket(len_);
        socket_->setupReceiveSocket(param_address, adma_port);
        mapping_ = new genesys::parser::Mapping(11700, "adma_ros2_delta_driver");
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
        std::array<char, 856> recv_buf_2;
        while(rclcpp::ok()) {
                socket_->receiveUDPPacket(recv_buf);

                //TODO: make adma_core_lib/mapping a bit generic for different array size
                memcpy(&recv_buf_2, &recv_buf, sizeof(recv_buf));

                delta_msg_scaled.long_delta_distance = mapping_->loadDataFromBuffer<float, double>("delta_scaled.long_delta_distance", recv_buf_2);
                delta_msg_scaled.long_delta_velocity = mapping_->loadDataFromBuffer<float, double>("delta_scaled.long_delta_velocity", recv_buf_2);
                delta_msg_scaled.lat_delta_distance = mapping_->loadDataFromBuffer<float, double>("delta_scaled.lat_delta_distance", recv_buf_2);
                delta_msg_scaled.lat_delta_velocity = mapping_->loadDataFromBuffer<float, double>("delta_scaled.lat_delta_velocity", recv_buf_2);
                delta_msg_scaled.resultant_distance = mapping_->loadDataFromBuffer<float, double>("delta_scaled.resultant_distance", recv_buf_2);
                delta_msg_scaled.resultant_velocity = mapping_->loadDataFromBuffer<float, double>("delta_scaled.resultant_velocity", recv_buf_2);
                delta_msg_scaled.code_version = mapping_->loadDataFromBuffer<uint16_t, uint16_t>("delta_scaled.code_version", recv_buf_2);
                delta_msg_scaled.angle_of_orientation = mapping_->loadDataFromBuffer<float, double>("delta_scaled.angle_of_orientation", recv_buf_2);
                delta_msg_scaled.delta_time = mapping_->loadDataFromBuffer<int32_t, int32_t>("delta_scaled.delta_time", recv_buf_2);
                delta_msg_scaled.target_status = mapping_->loadDataFromBuffer<uint16_t, uint16_t>("delta_scaled.target_status", recv_buf_2);
                delta_msg_scaled.hunter_status = mapping_->loadDataFromBuffer<uint16_t, uint16_t>("delta_scaled.hunter_status", recv_buf_2);
                // fill msg header for scaled msg
                delta_msg_scaled.header.frame_id = "addondelta";
                // TODO: may fill timestamp with data based on delta input
                delta_msg_scaled.header.stamp = get_clock()->now();
                // fill scaled msg with content
                // delta_msg_scaled.abd_header = delta_msg_raw.abd_header;

                delta_msg_scaled.target_forward_velocity = mapping_->loadDataFromBuffer<int16_t, double>("delta_scaled.target_forward_velocity", recv_buf_2);
                delta_msg_scaled.hunter_forward_velocity = mapping_->loadDataFromBuffer<int16_t, double>("delta_scaled.hunter_forward_velocity", recv_buf_2);
                delta_msg_scaled.target_forward_acceleration = mapping_->loadDataFromBuffer<int16_t, double>("delta_scaled.target_forward_acceleration", recv_buf_2);
                delta_msg_scaled.hunter_forward_acceleration = mapping_->loadDataFromBuffer<int16_t, double>("delta_scaled.hunter_forward_acceleration", recv_buf_2);
                delta_msg_scaled.target_lateral_velocity = mapping_->loadDataFromBuffer<int16_t, double>("delta_scaled.target_lateral_velocity", recv_buf_2);
                delta_msg_scaled.hunter_lateral_velocity = mapping_->loadDataFromBuffer<int16_t, double>("delta_scaled.hunter_lateral_velocity", recv_buf_2);
                delta_msg_scaled.target_lateral_acceleration = mapping_->loadDataFromBuffer<int16_t, double>("delta_scaled.target_lateral_acceleration", recv_buf_2);
                delta_msg_scaled.hunter_lateral_acceleration = mapping_->loadDataFromBuffer<int16_t, double>("delta_scaled.hunter_lateral_acceleration", recv_buf_2);
                delta_msg_scaled.target_pitch_angle = mapping_->loadDataFromBuffer<int16_t, double>("delta_scaled.target_pitch_angle", recv_buf_2);
                delta_msg_scaled.hunter_pitch_angle = mapping_->loadDataFromBuffer<int16_t, double>("delta_scaled.hunter_pitch_angle", recv_buf_2);
                // // modify individual values where required (e.g. coordinates, LSB factor)
                delta_msg_scaled.target_longitude = convertCoordinates(mapping_->loadDataFromBuffer<double, double>("delta_scaled.target_longitude", recv_buf_2));
                delta_msg_scaled.target_latitude = convertCoordinates(mapping_->loadDataFromBuffer<double, double>("delta_scaled.target_latitude", recv_buf_2));

                // publish raw data as byte array
                delta_msg_raw.data_size = len_;
                delta_msg_raw.header = delta_msg_scaled.header;
                // copy raw data
                for (int i = 0; i < len_; ++i) {
                        delta_msg_raw.raw_data.push_back(recv_buf[i]);
                }
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
