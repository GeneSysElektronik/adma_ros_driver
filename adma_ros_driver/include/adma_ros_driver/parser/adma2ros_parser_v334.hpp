#pragma once

#include "adma_ros_driver_msgs/AdmaDataScaled.h"
#include "adma_ros_driver_msgs/ByteStatus.h"
#include "adma_ros_driver_msgs/ByteErrorWarning.h"
#include "adma_ros_driver_msgs/Status.h"
#include "adma_ros_driver_msgs/ErrorWarning.h"
#include "adma_ros_driver_msgs/AdmaStatus.h"
#include "adma_ros_driver/data/adma_data_v334.hpp"
#include <bitset>

class ADMA2ROSParserV334
{
        public:
                ADMA2ROSParserV334();
                ~ADMA2ROSParserV334(){};
                void mapAdmaMessageToROS(adma_ros_driver_msgs::AdmaDataScaled& rosMsg, AdmaDataV334& admaData);
                void mapStatusToROS(adma_ros_driver_msgs::AdmaStatus& rosMsg, AdmaDataV334& admaData);
                void mapAdmaHeader(adma_ros_driver_msgs::AdmaDataScaled& rosMsg, AdmaDataV334& admaData);
                void mapStatusBytes(adma_ros_driver_msgs::ByteStatus& rosMsgByteStatus, AdmaDataV334& admaData);
                void mapStatusBitfields(adma_ros_driver_msgs::Status& rosMsgStatus, AdmaDataV334& admaData);
                void mapErrorWarningBytes(adma_ros_driver_msgs::ByteErrorWarning& rosMsgByteErrorWarning, AdmaDataV334& admaData);
                void mapErrorWarningBitfields(adma_ros_driver_msgs::ErrorWarning& rosMsgErrorWarning, AdmaDataV334& admaData);
                void mapUnscaledData(adma_ros_driver_msgs::AdmaDataScaled& rosMsg, AdmaDataV334& admaData);
                void mapScaledData(adma_ros_driver_msgs::AdmaDataScaled& rosMsg, AdmaDataV334& admaData);
                void mapPOI(adma_ros_driver_msgs::AdmaDataScaled& rosMsg, AdmaDataV334& admaData);
};

