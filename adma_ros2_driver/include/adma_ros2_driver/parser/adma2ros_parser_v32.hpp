#pragma once

#include <bitset>

#include "adma_msgs/msg/adma_data.hpp"
#include "adma_ros2_driver/data/adma_data_v32.hpp"

class ADMA2ROSParserV32
{
public:
  ADMA2ROSParserV32();
  ~ADMA2ROSParserV32() {}
  void mapAdmaMessageToROS(adma_msgs::msg::AdmaData & ros_msg, AdmaDataV32 & adma_data);
};
