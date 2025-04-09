#pragma once

#include <iostream>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float64.hpp>

#include "adma_ros2_driver/data/adma_data_v32.hpp"
#include "adma_ros2_driver/parser/adma2ros_parser_v32.hpp"
#include "adma_ros_driver_msgs/msg/adma_data.hpp"
#include "adma_ros_driver_msgs/msg/adma_data_scaled.hpp"
#include "adma_ros_driver_msgs/msg/adma_status.hpp"
#include "adma_core_lib/parser/mapping.hpp"

class ADMA2ROSParser
{
public:
  ADMA2ROSParser(u_int16_t version);
  ~ADMA2ROSParser() {}
  void findMappingFiles(std::string &protocolFileName, std::string &glossarFileName);
  void mapAdmaMessageToROS(
    adma_ros_driver_msgs::msg::AdmaData & ros_msg, std::array<char, 856> & recv_data);
  void extractNavSatFix(
    adma_ros_driver_msgs::msg::AdmaData & ros_msg, sensor_msgs::msg::NavSatFix & nav_ros_msg);
  void extractNavSatFix(
    adma_ros_driver_msgs::msg::AdmaDataScaled & ros_msg, sensor_msgs::msg::NavSatFix & nav_ros_msg,
    std::array<adma_ros_driver_msgs::msg::POI, 8> &pois, uint8_t desiredSource);
  void extractIMU(
    adma_ros_driver_msgs::msg::AdmaData & ros_msg, sensor_msgs::msg::Imu & imu_ros_msg);
  void extractIMU(
    adma_ros_driver_msgs::msg::AdmaDataScaled & ros_msg, sensor_msgs::msg::Imu & imu_ros_msg,
    std::array<adma_ros_driver_msgs::msg::POI, 8> &pois, uint8_t desiredSource);
  void extractOdometry(
    adma_ros_driver_msgs::msg::AdmaDataScaled & ros_msg, 
    nav_msgs::msg::Odometry & odometry_msg, double yawOffset,
    std::array<adma_ros_driver_msgs::msg::POI, 8> &pois, uint8_t desiredSource);
  void parseScaledData(adma_ros_driver_msgs::msg::AdmaData & ros_msg);
  ADMA2ROSParserV32 * parserV32_;
  genesys::parser::Mapping * mapping_;
  void extractHeading(std_msgs::msg::Float64 &headingMsg, std::array<char, 856> & recv_data);
  void extractAdmaStatus(adma_ros_driver_msgs::msg::AdmaStatus &statusMsg, std::array<char, 856> & recv_data);
  void extractAdmaDataScaled(adma_ros_driver_msgs::msg::AdmaDataScaled &admaScaledMsg, std::array<char, 856> & recv_data);
  void extractPOIs(adma_ros_driver_msgs::msg::AdmaDataScaled &admaScaledMsg, std::array<char, 856> &recv_data);

private:
  void getStatusGPS(adma_ros_driver_msgs::msg::AdmaData & ros_msg, unsigned char gps_status);
  void getStatusTrigger(
    adma_ros_driver_msgs::msg::AdmaData & ros_msg, unsigned char gps_trigger_status);
  void getEVKStatus(adma_ros_driver_msgs::msg::AdmaData & ros_msg, unsigned char evk_status);
  void getErrorandWarning(
    adma_ros_driver_msgs::msg::AdmaData & ros_msg, unsigned char adma_data[4]);
  
  uint16_t protocolVersion_;
};
