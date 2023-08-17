#include <netdb.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include "adma_ros2_driver/parser/adma2ros_parser.hpp"
#include "adma_ros_driver_msgs/msg/adma_data.hpp"
#include "adma_ros_driver_msgs/msg/adma_data_raw.hpp"
#include "adma_ros_driver_msgs/msg/adma_data_scaled.hpp"
#include "adma_ros_driver_msgs/msg/adma_status.hpp"

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
  std::string protocol_version_;

  // publisher
  rclcpp::Publisher<adma_ros_driver_msgs::msg::AdmaData>::SharedPtr pub_adma_data_;
  rclcpp::Publisher<adma_ros_driver_msgs::msg::AdmaDataRaw>::SharedPtr pub_adma_data_raw_;
  rclcpp::Publisher<adma_ros_driver_msgs::msg::AdmaDataScaled>::SharedPtr pub_adma_data_scaled_;
  rclcpp::Publisher<adma_ros_driver_msgs::msg::AdmaStatus>::SharedPtr pub_adma_status_;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr pub_navsat_fix_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_heading_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_velocity_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odometry_;

  // frame_ids for the ros msgs
  std::string gnss_frame_;
  std::string imu_frame_;
  std::string adma_frame_;
  std::string adma_status_frame_;
  std::string raw_data_frame_;
  std::string odometry_pose_frame_;
  std::string odometry_child_frame_;

  ADMA2ROSParser * parser_;

  // yaw offset angle if the odometry should be rotated by a fixed angle
  double odometry_yaw_offset_;

  // desired data sources per topic (POI_x or MRP)
  uint8_t navsatfix_id_;
  uint8_t imu_id_;
  uint8_t velocity_id_;
  uint8_t odometry_id_;
  std::array<adma_ros_driver_msgs::msg::POI, 8> pois;
};
}  // namespace genesys
