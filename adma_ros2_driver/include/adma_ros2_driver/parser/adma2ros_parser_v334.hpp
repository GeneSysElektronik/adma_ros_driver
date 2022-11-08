#pragma once

#include "adma_msgs/msg/adma_data_scaled.hpp"
#include "adma_ros2_driver/data/adma_data_v334.hpp"
#include <bitset>

class ADMA2ROSParserV334
{
        public:
                ADMA2ROSParserV334();
                ~ADMA2ROSParserV334(){};
                void mapAdmaMessageToROS(adma_msgs::msg::AdmaDataScaled& rosMsg, AdmaDataV334& admaData);
                void mapAdmaHeader(adma_msgs::msg::AdmaDataScaled& rosMsg, AdmaDataV334& admaData);
                void mapBitfields(adma_msgs::msg::AdmaDataScaled& rosMsg, AdmaDataV334& admaData);
                void mapUnscaledData(adma_msgs::msg::AdmaDataScaled& rosMsg, AdmaDataV334& admaData);
                void mapScaledData(adma_msgs::msg::AdmaDataScaled& rosMsg, AdmaDataV334& admaData);
                void mapPOI(std::vector<adma_msgs::msg::POI>& poiList, AdmaDataV334& admaData);
};

