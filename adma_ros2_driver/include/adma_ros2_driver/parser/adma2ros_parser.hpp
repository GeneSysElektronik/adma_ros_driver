#pragma once

#include <iostream>
#include <memory>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include "adma_ros2_driver/data/adma_data_v32.hpp"
#include "adma_ros2_driver/parser/adma2ros_parser_v32.hpp"
#include "adma_ros2_driver/parser/adma2ros_parser_v333.hpp"
#include "adma_ros2_driver/parser/adma2ros_parser_v334.hpp"
#include "adma_ros_driver_msgs/msg/adma_data.hpp"
#include "adma_ros_driver_msgs/msg/adma_data_scaled.hpp"

class ADMA2ROSParser
{
public:
  ADMA2ROSParser(std::string version);
  ~ADMA2ROSParser() {}
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
  void parseV334(adma_ros_driver_msgs::msg::AdmaDataScaled & ros_msg, AdmaDataV334 & local_data);
  void parseV334Status(adma_ros_driver_msgs::msg::AdmaStatus & ros_msg, AdmaDataV334 & local_data);
  void parseScaledData(adma_ros_driver_msgs::msg::AdmaData & ros_msg);
  ADMA2ROSParserV32 parserV32_;
  ADMA2ROSParserV333 parserV333_;
  ADMA2ROSParserV334 parserV334_;

private:
  template <typename AdmaDataHeaderStruct>
  void parseStaticHeader(
    adma_ros_driver_msgs::msg::AdmaData & ros_msg, AdmaDataHeaderStruct & static_header);
  template <typename AdmaDataHeaderStruct>
  void parseDynamicHeader(
    adma_ros_driver_msgs::msg::AdmaData & ros_msg, AdmaDataHeaderStruct & dynamic_header);
  void getStatusGPS(adma_ros_driver_msgs::msg::AdmaData & ros_msg, unsigned char gps_status);
  void getStatusTrigger(
    adma_ros_driver_msgs::msg::AdmaData & ros_msg, unsigned char gps_trigger_status);
  void getEVKStatus(adma_ros_driver_msgs::msg::AdmaData & ros_msg, unsigned char evk_status);
  void getErrorandWarning(
    adma_ros_driver_msgs::msg::AdmaData & ros_msg, unsigned char adma_data[4]);
  
  std::string version_;
};
