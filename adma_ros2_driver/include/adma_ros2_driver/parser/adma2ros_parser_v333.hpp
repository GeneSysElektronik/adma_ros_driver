#pragma once

#include <bitset>

#include "adma_msgs/msg/adma_data.hpp"
#include "adma_ros2_driver/data/adma_data_v333.hpp"

class ADMA2ROSParserV333
{
public:
  ADMA2ROSParserV333();
  ~ADMA2ROSParserV333();
  void mapAdmaMessageToROS(adma_msgs::msg::AdmaData & ros_msg, AdmaDataV333 & local_data);
  void getKFStatus(adma_msgs::msg::AdmaData & ros_msg, unsigned char kf_status);

protected:
};
