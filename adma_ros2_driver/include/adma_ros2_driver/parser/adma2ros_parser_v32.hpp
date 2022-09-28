#pragma once

#include "adma_msgs/msg/adma_data.hpp"
#include "adma_ros2_driver/data/adma_data_v32.hpp"
#include <bitset>

class ADMA2ROSParserV32
{
        public:
                ADMA2ROSParserV32();
                ~ADMA2ROSParserV32(){};
                void mapAdmaMessageToROS(adma_msgs::msg::AdmaData& rosMsg, AdmaDataV32& admaData);
};

