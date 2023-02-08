#include <netdb.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>

#include "adma_ros2_driver/parser/adma2ros_parser.hpp"
#include "adma_ros_driver_msgs/msg/adma_data.hpp"
#include "adma_ros_driver_msgs/msg/adma_data_raw.hpp"
#include "adma_ros_driver_msgs/msg/adma_data_scaled.hpp"
#include "adma_ros_driver_msgs/msg/adma_status.hpp"

#define MODE_DEFAULT "default"
#define MODE_RECORD "record"
#define MODE_REPLAY "replay"

#pragma once

namespace genesys
{
class ADMADriver : public rclcpp::Node
{
public:
  explicit ADMADriver(const rclcpp::NodeOptions & options);
  virtual ~ADMADriver();

private:
  void initializeUDP(std::string adma_address);
  void updateLoop();
  void parseData(std::array<char, 856> recv_buf);
  void recordedDataCB(adma_ros_driver_msgs::msg::AdmaDataRaw::SharedPtr data_msg);

  // Socket file descriptor for receiving from adma
  int rcv_sock_fd_;
  // Address info for receiving from adma
  struct addrinfo * rcv_addr_info_;
  // adma socket address
  struct sockaddr_in adma_address_;
  //Adma  socket address length
  socklen_t adma_address_length_;
  int adma_port_;
  size_t len_ = 0;
  /** \brief Check the timings */
  bool performance_check_ = true;
  // the mode this driver node is used
  std::string mode_;
  std::string protocol_version_;

  // publisher
  rclcpp::Publisher<adma_ros_driver_msgs::msg::AdmaData>::SharedPtr pub_adma_data_;
  rclcpp::Publisher<adma_ros_driver_msgs::msg::AdmaDataRaw>::SharedPtr pub_adma_data_raw_;
  rclcpp::Publisher<adma_ros_driver_msgs::msg::AdmaDataRaw>::SharedPtr pub_adma_data_recorded_;
  rclcpp::Publisher<adma_ros_driver_msgs::msg::AdmaDataScaled>::SharedPtr pub_adma_data_scaled_;
  rclcpp::Publisher<adma_ros_driver_msgs::msg::AdmaStatus>::SharedPtr pub_adma_status_;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr pub_navsat_fix_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_heading_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_velocity_;

  // subscriber
  rclcpp::Subscription<adma_ros_driver_msgs::msg::AdmaDataRaw>::SharedPtr sub_raw_data_;

  // frame_ids for the ros msgs
  std::string gnss_frame_;
  std::string imu_frame_;
  std::string adma_frame_;
  std::string adma_status_frame_;
  std::string raw_data_frame_;

  ADMA2ROSParser * parser_;
};
}  // namespace genesys
