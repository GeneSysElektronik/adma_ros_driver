#pragma once

#include "adma_msgs/msg/adma_data_scaled.hpp"
#include "adma_msgs/msg/byte_status.hpp"
#include "adma_msgs/msg/byte_error_warning.hpp"
#include "adma_msgs/msg/status.hpp"
#include "adma_msgs/msg/error_warning.hpp"
#include "adma_msgs/msg/adma_status.hpp"
#include "adma_ros2_driver/data/adma_data_v334.hpp"
#include <bitset>

class ADMA2ROSParserV334
{
        public:
                ADMA2ROSParserV334();
                ~ADMA2ROSParserV334(){};
                void mapAdmaMessageToROS(adma_msgs::msg::AdmaDataScaled& rosMsg, AdmaDataV334& admaData);
                void mapStatusToROS(adma_msgs::msg::AdmaStatus& rosMsg, AdmaDataV334& admaData);
                void mapAdmaHeader(adma_msgs::msg::AdmaDataScaled& rosMsg, AdmaDataV334& admaData);
                void mapStatusBytes(adma_msgs::msg::ByteStatus& rosMsgByteStatus, AdmaDataV334& admaData);
                void mapStatusBitfields(adma_msgs::msg::Status& rosMsgStatus, AdmaDataV334& admaData);
                void mapErrorWarningBytes(adma_msgs::msg::ByteErrorWarning& rosMsgByteErrorWarning, AdmaDataV334& admaData);
                void mapErrorWarningBitfields(adma_msgs::msg::ErrorWarning& rosMsgErrorWarning, AdmaDataV334& admaData);
                void mapUnscaledData(adma_msgs::msg::AdmaDataScaled& rosMsg, AdmaDataV334& admaData);
                void mapScaledData(adma_msgs::msg::AdmaDataScaled& rosMsg, AdmaDataV334& admaData);
                void mapPOI(adma_msgs::msg::AdmaDataScaled& rosMsg, AdmaDataV334& admaData);
};

