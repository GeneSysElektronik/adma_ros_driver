#pragma once

#include <bitset>

#include "adma_ros2_driver/data/adma_data_v333.hpp"
#include "adma_ros_driver_msgs/msg/adma_data.hpp"

class ADMA2ROSParserV333
{
public:
  ADMA2ROSParserV333();
  ~ADMA2ROSParserV333();
  void mapAdmaMessageToROS(
    adma_ros_driver_msgs::msg::AdmaData & ros_msg, AdmaDataV333 & local_data);
  void getKFStatus(adma_ros_driver_msgs::msg::AdmaData & ros_msg, unsigned char kf_status);

protected:
};
