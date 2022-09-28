#pragma once

#include "adma_msgs/msg/adma_data.hpp"
#include "adma_ros2_driver/data/adma_data_v333.hpp"
#include <bitset>

class ADMA2ROSParserV333
{
        public:
                ADMA2ROSParserV333();
                ~ADMA2ROSParserV333();
                void mapAdmaMessageToROS(adma_msgs::msg::AdmaData& rosMsg, AdmaDataV333& localdata);
                void getKFStatus(adma_msgs::msg::AdmaData& rosMsg, unsigned char kfStatus);
        protected:
                
};
