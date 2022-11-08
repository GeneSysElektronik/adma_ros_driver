#pragma once

#include "adma_msgs/msg/adma_data_scaled.hpp"
#include "adma_ros2_driver/data/adma_data_v333.hpp"
#include <bitset>

class ADMA2ROSParserV334
{
        public:
                ADMA2ROSParserV334();
                ~ADMA2ROSParserV334(){};
                void mapAdmaMessageToROS(adma_msgs::msg::AdmaDataScaled& rosMsg, AdmaDataV333& admaData);
                void mapAdmaHeader(adma_msgs::msg::AdmaDataScaled& rosMsg, AdmaDataV333& admaData);
                void mapBitfields(adma_msgs::msg::AdmaDataScaled& rosMsg, AdmaDataV333& admaData);
                void mapUnscaledData(adma_msgs::msg::AdmaDataScaled& rosMsg, AdmaDataV333& admaData);
                void mapScaledData(adma_msgs::msg::AdmaDataScaled& rosMsg, AdmaDataV333& admaData);
};

