#pragma once

#include "adma_msgs/msg/adma_data_scaled.hpp"
#include "adma_msgs/msg/byte_states.hpp"
#include "adma_msgs/msg/byte_ew.hpp"
#include "adma_msgs/msg/states.hpp"
#include "adma_msgs/msg/ew.hpp"
#include "adma_msgs/msg/adma_state.hpp"
#include "adma_ros2_driver/data/adma_data_v334.hpp"
#include <bitset>

class ADMA2ROSParserV334
{
        public:
                ADMA2ROSParserV334();
                ~ADMA2ROSParserV334(){};
                void mapAdmaMessageToROS(adma_msgs::msg::AdmaDataScaled& rosMsg, AdmaDataV334& admaData);
                void mapStateToROS(adma_msgs::msg::AdmaState& rosMsg, AdmaDataV334& admaData);
                void mapAdmaHeader(adma_msgs::msg::AdmaDataScaled& rosMsg, AdmaDataV334& admaData);
                void mapStatusBytes(adma_msgs::msg::ByteStates& rosMsgByteStates, AdmaDataV334& admaData);
                void mapStatusBitfields(adma_msgs::msg::States& rosMsgStates, AdmaDataV334& admaData);
                void mapEWBytes(adma_msgs::msg::ByteEW& rosMsgByteEW, AdmaDataV334& admaData);
                void mapEWBitfields(adma_msgs::msg::EW& rosMsgEW, AdmaDataV334& admaData);
                void mapUnscaledData(adma_msgs::msg::AdmaDataScaled& rosMsg, AdmaDataV334& admaData);
                void mapScaledData(adma_msgs::msg::AdmaDataScaled& rosMsg, AdmaDataV334& admaData);
                void mapPOI(adma_msgs::msg::AdmaDataScaled& rosMsg, AdmaDataV334& admaData);
};

