#pragma once

#include <memory>
#include <iostream>

#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include "adma_msgs/msg/adma_data.hpp"
#include "adma_ros2_driver/data/adma_data_v32.hpp"
#include "adma_ros2_driver/parser/adma2ros_parser_v32.hpp"
#include "adma_ros2_driver/parser/adma2ros_parser_v333.hpp"


class ADMA2ROSParser
{
        public:
                ADMA2ROSParser(std::string version);
                ~ADMA2ROSParser(){};
                void mapAdmaMessageToROS(adma_msgs::msg::AdmaData& rosMsg, std::vector<char>& localData);
                void extractNavSatFix(adma_msgs::msg::AdmaData& rosMsg, sensor_msgs::msg::NavSatFix& navRosMsg);      
                void extractIMU(adma_msgs::msg::AdmaData& rosMsg, sensor_msgs::msg::Imu& imuRosMsg);
                void parseScaledData(adma_msgs::msg::AdmaData& rosMsg);
        private:
                void parseV32(adma_msgs::msg::AdmaData& rosMsg, std::vector<char>& localData);
                void parseV333(adma_msgs::msg::AdmaData& rosMsg, std::vector<char>& localData);
                template <typename AdmaDataStruct>
                void parseStaticHeader(adma_msgs::msg::AdmaData& rosMsg, AdmaDataStruct& staticHeader);
                template <typename AdmaDataStruct>
                void parseDynamicHeader(adma_msgs::msg::AdmaData& rosMsg, AdmaDataStruct& dynamicHeader);
                void getstatusgps(adma_msgs::msg::AdmaData& rosMsg, unsigned char gpsStatus);
                void getstatustrigger(adma_msgs::msg::AdmaData& rosMsg, unsigned char gpsTriggerStatus);
                void getevkstatus(adma_msgs::msg::AdmaData& rosMsg, unsigned char evkStatus);
                void geterrorandwarning(adma_msgs::msg::AdmaData& rosMsg, unsigned char admaData[4]);
                ADMA2ROSParserV32 _parserV32;
                ADMA2ROSParserV333 _parserV333;
                std::string _version;
};