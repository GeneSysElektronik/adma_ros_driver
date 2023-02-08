#pragma once

#include <bitset>

#include "adma_ros2_driver/data/adma_data_v32.hpp"
#include "adma_ros_driver_msgs/msg/adma_data.hpp"

class ADMA2ROSParserV32
{
public:
  ADMA2ROSParserV32();
  ~ADMA2ROSParserV32() {}
  void mapAdmaMessageToROS(adma_ros_driver_msgs::msg::AdmaData & ros_msg, AdmaDataV32 & adma_data);
};
