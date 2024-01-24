#pragma once

#include <bitset>

#include "adma_ros2_driver/data/adma_data_v335.hpp"
#include "adma_ros_driver_msgs/msg/adma_data_scaled.hpp"
#include "adma_ros_driver_msgs/msg/adma_status.hpp"
#include "adma_ros_driver_msgs/msg/byte_error_warning.hpp"
#include "adma_ros_driver_msgs/msg/byte_status.hpp"
#include "adma_ros_driver_msgs/msg/error_warning.hpp"
#include "adma_ros_driver_msgs/msg/status.hpp"

class ADMA2ROSParserV335
{
public:
  ADMA2ROSParserV335();
  ~ADMA2ROSParserV335() {}
  void mapAdmaMessageToROS(
    adma_ros_driver_msgs::msg::AdmaDataScaled & ros_msg, AdmaDataV335 & adma_data);
  void mapStatusToROS(adma_ros_driver_msgs::msg::AdmaStatus & ros_msg, AdmaDataV335 & adma_data);
  void mapAdmaHeader(adma_ros_driver_msgs::msg::AdmaDataScaled & ros_msg, AdmaDataV335 & adma_data);
  void mapStatusBytes(adma_ros_driver_msgs::msg::ByteStatus & ros_msg_byte_status, AdmaDataV335 & adma_data);
  void mapStatusBitfields(adma_ros_driver_msgs::msg::Status & ros_msg_status, AdmaDataV335 & adma_data);
  void mapStatusBit0(adma_ros_driver_msgs::msg::Status & ros_msg_status, unsigned char status_byte);
  void mapStatusBit1(adma_ros_driver_msgs::msg::Status & ros_msg_status, unsigned char status_byte);
  void mapStatusBit2(adma_ros_driver_msgs::msg::Status & ros_msg_status, unsigned char status_byte);
  void mapStatusBit4(adma_ros_driver_msgs::msg::Status & ros_msg_status, unsigned char status_byte);
  void mapStatusBit5(adma_ros_driver_msgs::msg::Status & ros_msg_status, unsigned char status_byte);
  void mapErrorWarningBytes(adma_ros_driver_msgs::msg::ByteErrorWarning & ros_msg_byte_error_warning, AdmaDataV335 & adma_data);
  void mapErrorWarningBitfields(adma_ros_driver_msgs::msg::ErrorWarning & ros_msg_error_warning, AdmaDataV335 & adma_data);
  void mapErrorBit0(adma_ros_driver_msgs::msg::ErrorWarning & ros_msg_error_warning, unsigned char error_byte);
  void mapErrorBit1(adma_ros_driver_msgs::msg::ErrorWarning & ros_msg_error_warning, unsigned char error_byte);
  void mapWarningBit0(adma_ros_driver_msgs::msg::ErrorWarning & ros_msg_error_warning, unsigned char warning_byte);
  void mapErrorBit2(adma_ros_driver_msgs::msg::ErrorWarning & ros_msg_error_warning, unsigned char error_byte);
  void mapUnscaledData(adma_ros_driver_msgs::msg::AdmaDataScaled & ros_msg, AdmaDataV335 & adma_data);
  void mapScaledData(adma_ros_driver_msgs::msg::AdmaDataScaled & ros_msg, AdmaDataV335 & adma_data);
  void mapPOI(adma_ros_driver_msgs::msg::AdmaDataScaled & ros_msg, AdmaDataV335 & adma_data);
};
