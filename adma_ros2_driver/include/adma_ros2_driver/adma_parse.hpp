#include "adma_msgs/msg/adma_data.hpp"
#include "adma_msgs/msg/poi.hpp"
#include <std_msgs/msg/float64.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <bitset>

#pragma once

const short nrOfPOI = 8;
const uint16_t startIndexAccBodyPOI = 168;
const uint16_t startIndexAccHorPOI = 232;
const uint16_t startIndexMiscPOI = 352;
const uint16_t startIndexInsHeightPOI = 552;
const uint16_t startIndexInsPositionPOI = 608;
const uint16_t startIndexVelPOI = 752;

void getparseddata(const std::string& local_data, adma_msgs::msg::AdmaData& message, sensor_msgs::msg::NavSatFix& msg_fix, std_msgs::msg::Float64& msg_heading, std_msgs::msg::Float64& msg_velocity);
void getadmastaticheader(const std::string& local_data, adma_msgs::msg::AdmaData& message);
void getadmadynamicheader(const std::string& local_data, adma_msgs::msg::AdmaData& message);
void getstatusgps(const std::string& local_data, adma_msgs::msg::AdmaData& message);
void getstatustrigger(const std::string& local_data, adma_msgs::msg::AdmaData& message);
void getevkstatus(const std::string& local_data, adma_msgs::msg::AdmaData& message);
void getstatuscount(const std::string& local_data, adma_msgs::msg::AdmaData& message);
void geterrorandwarning(const std::string& local_data, adma_msgs::msg::AdmaData& message);
void getsensorbodyxyz(const std::string& local_data, adma_msgs::msg::AdmaData& message);
void getratesbodyxyz(const std::string& local_data, adma_msgs::msg::AdmaData& message);
void getrateshorizontalxyz(const std::string& local_data, adma_msgs::msg::AdmaData& message);
void getaccelerationbody(const std::string& local_data, adma_msgs::msg::AdmaData& message);
void getaccelerationhor(const std::string& local_data, adma_msgs::msg::AdmaData& message);
void getPOI(const std::string& local_data, std::vector<adma_msgs::msg::POI>& poiList);
void getexternalvelocityanalog(const std::string& local_data, adma_msgs::msg::AdmaData& message);
void getexternalvecovitydigpulses(const std::string& local_data, adma_msgs::msg::AdmaData& message);
void getexternalvelocitycorrected(const std::string& local_data, adma_msgs::msg::AdmaData& message);
void getbarometerpressure(const std::string& local_data, adma_msgs::msg::AdmaData& message);
void getbarometerheight(const std::string& local_data, adma_msgs::msg::AdmaData& message);
void getmiscellaneuos(const std::string& local_data, adma_msgs::msg::AdmaData& message);
void gettriggers(const std::string& local_data, adma_msgs::msg::AdmaData& message);
void getsystemdata(const std::string& local_data, adma_msgs::msg::AdmaData& message);
void getgpsabs(const std::string& local_data, adma_msgs::msg::AdmaData& message);
void getgpsposrel(const std::string& local_data, adma_msgs::msg::AdmaData& message);
void getgpsepe(const std::string& local_data, adma_msgs::msg::AdmaData& message);
void getgpsvelframe(const std::string& local_data, adma_msgs::msg::AdmaData& message, std_msgs::msg::Float64& msg_velocity);
void getgpsveleve(const std::string& local_data, adma_msgs::msg::AdmaData& message);
void getgpstimeutc(const std::string& local_data, adma_msgs::msg::AdmaData& message);
void getgpsauxdata1(const std::string& local_data, adma_msgs::msg::AdmaData& message);
void getgpsauxdata2(const std::string& local_data, adma_msgs::msg::AdmaData& message);
void getinsanglegpscog(const std::string& local_data, adma_msgs::msg::AdmaData& message, std_msgs::msg::Float64& msg_heading);
void getgpsheight(const std::string& local_data, adma_msgs::msg::AdmaData& message);
void getgpsdualanttimeutc(const std::string& local_data, adma_msgs::msg::AdmaData& message);
void getgpsdualantangle(const std::string& local_data, adma_msgs::msg::AdmaData& message);
void getgpsdualantangleete(const std::string& local_data, adma_msgs::msg::AdmaData& message);
void getinspositionheight(const std::string& local_data, adma_msgs::msg::AdmaData& message);
void getinstimeutc(const std::string& local_data, adma_msgs::msg::AdmaData& message);
void getinspositionabs(const std::string& local_data, adma_msgs::msg::AdmaData& message, sensor_msgs::msg::NavSatFix& msg_fix);
void getinsposrel(const std::string& local_data, adma_msgs::msg::AdmaData& message);
void getinsvelhorxyz(const std::string& local_data, adma_msgs::msg::AdmaData& message);
void getinsvelframexyz(const std::string& local_data, adma_msgs::msg::AdmaData& message);
void getinsepe(const std::string& local_data, adma_msgs::msg::AdmaData& message);
void getinseveandete(const std::string& local_data, adma_msgs::msg::AdmaData& message);
void getanalog(const std::string& local_data, adma_msgs::msg::AdmaData& message);
void getkalmanfilter(const std::string& local_data, adma_msgs::msg::AdmaData& message);
void getgnssreceiver(const std::string& local_data, adma_msgs::msg::AdmaData& message);
bool getbit(unsigned char byte, int position);
