#pragma once

#include <memory>
#include <iostream>

#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include "adma_msgs/msg/adma_data.hpp"
#include "adma_msgs/msg/adma_data_scaled.hpp"
#include "adma_ros2_driver/data/adma_data_v32.hpp"
#include "adma_ros2_driver/parser/adma2ros_parser_v32.hpp"
#include "adma_ros2_driver/parser/adma2ros_parser_v333.hpp"
#include "adma_ros2_driver/parser/adma2ros_parser_v334.hpp"


class ADMA2ROSParser
{
        public:
                ADMA2ROSParser(std::string version);
                ~ADMA2ROSParser(){};
                void mapAdmaMessageToROS(adma_msgs::msg::AdmaData& rosMsg, std::array<char, 856>& recvData);
                void extractNavSatFix(adma_msgs::msg::AdmaData& rosMsg, sensor_msgs::msg::NavSatFix& navRosMsg);
                void extractNavSatFix(adma_msgs::msg::AdmaDataScaled& rosMsg, sensor_msgs::msg::NavSatFix& navRosMsg); 
                void extractIMU(adma_msgs::msg::AdmaData& rosMsg, sensor_msgs::msg::Imu& imuRosMsg);
                void extractIMU(adma_msgs::msg::AdmaDataScaled& rosMsg, sensor_msgs::msg::Imu& imuRosMsg);
                void parseV334(adma_msgs::msg::AdmaDataScaled& rosMsg, AdmaDataV334& localData);
                void parseV334State(adma_msgs::msg::AdmaState& rosMsg, AdmaDataV334& localData);
                void parseScaledData(adma_msgs::msg::AdmaData& rosMsg);
        private:
                template <typename AdmaDataHeaderStruct>
                void parseStaticHeader(adma_msgs::msg::AdmaData& rosMsg, AdmaDataHeaderStruct& staticHeader);
                template <typename AdmaDataHeaderStruct>
                void parseDynamicHeader(adma_msgs::msg::AdmaData& rosMsg, AdmaDataHeaderStruct& dynamicHeader);
                void getstatusgps(adma_msgs::msg::AdmaData& rosMsg, unsigned char gpsStatus);
                void getstatustrigger(adma_msgs::msg::AdmaData& rosMsg, unsigned char gpsTriggerStatus);
                void getevkstatus(adma_msgs::msg::AdmaData& rosMsg, unsigned char evkStatus);
                void geterrorandwarning(adma_msgs::msg::AdmaData& rosMsg, unsigned char admaData[4]);
                ADMA2ROSParserV32 _parserV32;
                ADMA2ROSParserV333 _parserV333;
                ADMA2ROSParserV334 _parserV334;
                std::string _version;
};