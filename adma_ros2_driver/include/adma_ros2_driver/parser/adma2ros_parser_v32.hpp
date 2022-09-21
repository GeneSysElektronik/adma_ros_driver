#pragma once

#include "std_msgs/msg/float64.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "adma_msgs/msg/adma_data.hpp"
#include "adma_ros2_driver/data/adma_data_v32.hpp"
#include <bitset>

void mapAdmaMessageToROS(adma_msgs::msg::AdmaData& rosMsg, AdmaDataV32& admaMsg);

void getstatusgps(adma_msgs::msg::AdmaData& rosMsg, AdmaDataV32& admaMsg);
void getstatustrigger(adma_msgs::msg::AdmaData& rosMsg, AdmaDataV32& admaMsg);
void getevkstatus(adma_msgs::msg::AdmaData& rosMsg, AdmaDataV32& admaMsg);
void geterrorandwarning(adma_msgs::msg::AdmaData& rosMsg, AdmaDataV32& admaMsg);
void extractNavSatFix(adma_msgs::msg::AdmaData& rosMsg, sensor_msgs::msg::NavSatFix& navRosMsg);
void extractIMU(adma_msgs::msg::AdmaData& rosMsg, sensor_msgs::msg::Imu& imuRosMsg);
