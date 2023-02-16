#pragma once

#include "adma_ros_driver_msgs/Adma.h"
#include "adma_ros_driver/data/adma_data_v333.hpp"
#include <bitset>

class ADMA2ROSParserV333
{
        public:
                ADMA2ROSParserV333();
                ~ADMA2ROSParserV333();
                void mapAdmaMessageToROS(adma_ros_driver_msgs::Adma& rosMsg, AdmaDataV333& localdata);
                void getKFStatus(adma_ros_driver_msgs::Adma& rosMsg, unsigned char kfStatus);
                
};
