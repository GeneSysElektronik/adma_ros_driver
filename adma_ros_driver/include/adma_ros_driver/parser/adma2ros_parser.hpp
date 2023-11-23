#pragma once

#include <memory>
#include <iostream>

#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include "adma_ros_driver_msgs/Adma.h"
#include "adma_ros_driver_msgs/AdmaDataScaled.h"
#include "adma_ros_driver/parser/adma2ros_parser_v333.hpp"
#include "adma_ros_driver/parser/adma2ros_parser_v334.hpp"
#include "adma_ros_driver/parser/adma2ros_parser_v335.hpp"


class ADMA2ROSParser
{
        public:
                ADMA2ROSParser();
                ~ADMA2ROSParser(){};
                void parseV333(adma_ros_driver_msgs::Adma& rosMsg, std::array<char, 856>& recvData);
                void extractNavSatFix(adma_ros_driver_msgs::AdmaDataScaled& rosMsg, sensor_msgs::NavSatFix& navRosMsg); 
                void extractIMU(adma_ros_driver_msgs::AdmaDataScaled& rosMsg, sensor_msgs::Imu& imuRosMsg);
                void parseV334(adma_ros_driver_msgs::AdmaDataScaled& rosMsg, AdmaDataV334& localData);
                void parseV334Status(adma_ros_driver_msgs::AdmaStatus& rosMsg, AdmaDataV334& localData);
                void parseV335(adma_ros_driver_msgs::AdmaDataScaled& rosMsg, AdmaDataV335& localData);
                void parseV335Status(adma_ros_driver_msgs::AdmaStatus& rosMsg, AdmaDataV335& localData);
                void parseScaledData(adma_ros_driver_msgs::Adma& rosMsg);
        private:
                template <typename AdmaDataHeaderStruct>
                void parseStaticHeader(adma_ros_driver_msgs::Adma& rosMsg, AdmaDataHeaderStruct& staticHeader);
                template <typename AdmaDataHeaderStruct>
                void parseDynamicHeader(adma_ros_driver_msgs::Adma& rosMsg, AdmaDataHeaderStruct& dynamicHeader);
                void getstatusgps(adma_ros_driver_msgs::Adma& rosMsg, unsigned char gpsStatus);
                void getstatustrigger(adma_ros_driver_msgs::Adma& rosMsg, unsigned char gpsTriggerStatus);
                void getevkstatus(adma_ros_driver_msgs::Adma& rosMsg, unsigned char evkStatus);
                void geterrorandwarning(adma_ros_driver_msgs::Adma& rosMsg, unsigned char admaData[4]);
                ADMA2ROSParserV333 _parserV333;
                ADMA2ROSParserV334 _parserV334;
                ADMA2ROSParserV335 _parserV335;
};