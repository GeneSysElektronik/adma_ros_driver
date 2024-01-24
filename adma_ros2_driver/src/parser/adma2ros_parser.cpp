#include "adma_ros2_driver/parser/adma2ros_parser.hpp"

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <iostream>

#include "adma_ros2_driver/parser/parser_utils.hpp"

ADMA2ROSParser::ADMA2ROSParser(std::string version)
: parserV32_(), parserV333_(), parserV334_(), parserV335_(), version_(version)
{
}

void ADMA2ROSParser::mapAdmaMessageToROS(
  adma_ros_driver_msgs::msg::AdmaData & ros_msg, std::array<char, 856> & recv_data)
{
  if (version_ == "v3.2") {
    AdmaDataV32 adma_data;
    memcpy(&adma_data, &recv_data, sizeof(adma_data));
    parseStaticHeader(ros_msg, adma_data.staticHeader);
    parseDynamicHeader(ros_msg, adma_data.dynamicHeader);
    getStatusGPS(ros_msg, adma_data.gpsStatus);
    getStatusTrigger(ros_msg, adma_data.gpsTriggerStatus);
    getEVKStatus(ros_msg, adma_data.evkStatus);
    unsigned char ew_bytes[] = {
      adma_data.dataError1, adma_data.dataError2, adma_data.dataWarn1, adma_data.dataErrorHW};
    getErrorandWarning(ros_msg, ew_bytes);
    parserV32_.mapAdmaMessageToROS(ros_msg, adma_data);
  } else if (version_ == "v3.3.3") {
    AdmaDataV333 adma_data;
    memcpy(&adma_data, &recv_data, sizeof(adma_data));
    parseStaticHeader(ros_msg, adma_data.staticHeader);
    parseDynamicHeader(ros_msg, adma_data.dynamicHeader);
    getStatusGPS(ros_msg, adma_data.gnssStatus);
    getStatusTrigger(ros_msg, adma_data.signalInStatus);
    getEVKStatus(ros_msg, adma_data.miscStatus);
    parserV333_.getKFStatus(ros_msg, adma_data.kfStatus);
    unsigned char ew_bytes[] = {
      adma_data.dataError1, adma_data.dataError2, adma_data.dataWarn1, adma_data.dataError3};
    getErrorandWarning(ros_msg, ew_bytes);
    parserV333_.mapAdmaMessageToROS(ros_msg, adma_data);
  }
  parseScaledData(ros_msg);
}

void ADMA2ROSParser::parseV334(
  adma_ros_driver_msgs::msg::AdmaDataScaled & ros_msg, AdmaDataV334 & recv_data)
{
  parserV334_.mapAdmaMessageToROS(ros_msg, recv_data);
}

void ADMA2ROSParser::parseV334Status(
  adma_ros_driver_msgs::msg::AdmaStatus & ros_msg, AdmaDataV334 & local_data)
{
  parserV334_.mapStatusToROS(ros_msg, local_data);
}

void ADMA2ROSParser::parseV335(
    adma_ros_driver_msgs::msg::AdmaDataScaled& ros_msg, AdmaDataV335& recv_data)
{
    parserV335_.mapAdmaMessageToROS(ros_msg, recv_data);
}

void ADMA2ROSParser::parseV335Status(
    adma_ros_driver_msgs::msg::AdmaStatus& ros_msg, AdmaDataV335& local_data)
{
    parserV335_.mapStatusToROS(ros_msg, local_data);
}

template <typename AdmaDataHeaderStruct>
void ADMA2ROSParser::parseStaticHeader(
  adma_ros_driver_msgs::msg::AdmaData & ros_msg, AdmaDataHeaderStruct & static_header)
{
  // fill static header information
  ros_msg.genesysid = static_header.genesysid;
  std::stringstream ss;
  ss << int(static_header.headerversion[0]) << int(static_header.headerversion[1])
     << int(static_header.headerversion[2]) << int(static_header.headerversion[3]);
  ros_msg.headerversion = ss.str();
  ss.clear();
  ss.str("");
  ros_msg.formatid = static_header.formatid;
  //TODO: this value is parsed wrong?!
  ss << int(static_header.formatversion[0]) << int(static_header.formatversion[1])
     << int(static_header.formatversion[2]) << int(static_header.formatversion[3]);
  ros_msg.formatversion = ss.str();
  ros_msg.serialno = static_header.serialno;
}

template <typename AdmaDataHeaderStruct>
void ADMA2ROSParser::parseDynamicHeader(
  adma_ros_driver_msgs::msg::AdmaData & ros_msg, AdmaDataHeaderStruct & dynamic_header)
{
  // fill dynamic header information
  ros_msg.configid = dynamic_header.configid;
  ros_msg.configformat = dynamic_header.configformat;
  ros_msg.configversion = dynamic_header.configversion;
  ros_msg.configsize = dynamic_header.configsize;
  ros_msg.byteoffset = dynamic_header.byteoffset;
  ros_msg.slicesize = dynamic_header.slicesize;
  ros_msg.slicedata = dynamic_header.slicedata;
}

/// \file
/// \brief  getstatusgps function - adma status information
/// \param  ros_msg ros message to fill with content
/// \param  gps_status byte with gps states
void ADMA2ROSParser::getStatusGPS(
  adma_ros_driver_msgs::msg::AdmaData & ros_msg, unsigned char gps_status)
{
  bool status_external_vel = getbit(gps_status, 7);
  bool status_skidding = getbit(gps_status, 5);
  bool standstill_c = getbit(gps_status, 4);
  bool rtk_precise = getbit(gps_status, 3);
  bool rtk_coarse = getbit(gps_status, 2);
  bool gps_mode = getbit(gps_status, 1);
  bool gps_out = getbit(gps_status, 0);

  /* status gps mode */
  if (gps_out) {
    ros_msg.statusgpsmode = 1;
  } else if (gps_mode) {
    ros_msg.statusgpsmode = 2;
  } else if (rtk_coarse) {
    ros_msg.statusgpsmode = 4;
  } else if (rtk_precise) {
    ros_msg.statusgpsmode = 8;
  }
  /* status stand still */
  ros_msg.statusstandstill = standstill_c;
  /* status skidding */
  ros_msg.statusskidding = status_skidding;
  /* status external velocity slip */
  ros_msg.statusexternalvelout = status_external_vel;
}

/// \file
/// \brief  getstatustrigger function - adma gps trigger information
/// \param  ros_msg ros message to fill with content
/// \param  gps_trigger_status byte with gps trigger states
void ADMA2ROSParser::getStatusTrigger(
  adma_ros_driver_msgs::msg::AdmaData & ros_msg, unsigned char gps_trigger_status)
{
  bool status_synclock = getbit(gps_trigger_status, 7);
  bool status_dead_reckoning = getbit(gps_trigger_status, 6);
  bool status_ahrs_ins = getbit(gps_trigger_status, 5);
  bool status_alignment = getbit(gps_trigger_status, 4);
  bool status_signal_in1 = getbit(gps_trigger_status, 3);
  bool status_signal_in2 = getbit(gps_trigger_status, 2);
  bool status_signal_in3 = getbit(gps_trigger_status, 1);
  bool status_trig_gps = getbit(gps_trigger_status, 0);
  /* status statustriggps */
  ros_msg.statustriggps = status_trig_gps;
  /* status statussignalin3 */
  ros_msg.statussignalin3 = status_signal_in3;
  /* status statussignalin2 */
  ros_msg.statussignalin2 = status_signal_in2;
  /* status statussignalin1 */
  ros_msg.statussignalin1 = status_signal_in1;
  /* status statusalignment */
  ros_msg.statusalignment = status_alignment;
  /* status statusahrsins */
  ros_msg.statusahrsins = status_ahrs_ins;
  /* status statusdeadreckoning */
  ros_msg.statusdeadreckoning = status_dead_reckoning;
  /* status statussynclock */
  ros_msg.statussynclock = status_synclock;
}

/// \file
/// \brief  getstatustrigger function - adma gps trigger information
/// \param  ros_msg ros message to fill with content
/// \param  evk_status byte with evk states
void ADMA2ROSParser::getEVKStatus(
  adma_ros_driver_msgs::msg::AdmaData & ros_msg, unsigned char evk_status)
{
  bool status_pos_b2 = getbit(evk_status, 7);
  bool status_pos_b1 = getbit(evk_status, 6);
  bool status_tilt_b2 = getbit(evk_status, 5);
  bool status_tilt_b1 = getbit(evk_status, 4);
  bool status_configuration_changed = getbit(evk_status, 3);
  bool status_heading_executed = getbit(evk_status, 2);
  bool status_evk_estimates = getbit(evk_status, 1);
  bool status_evk_activ = getbit(evk_status, 0);
  /* status statustriggps */
  ros_msg.statusevkactiv = status_evk_activ;
  /* status status_evk_estimates */
  ros_msg.statusevkestimates = status_evk_estimates;
  /* status status_heading_executed */
  ros_msg.statusheadingexecuted = status_heading_executed;
  /* status status_configuration_changed */
  ros_msg.statusconfigurationchanged = status_configuration_changed;
  /* status tilt */
  if (status_tilt_b1 == 0 && status_tilt_b2 == 0) {
    ros_msg.statustilt = 0;
  } else if (status_tilt_b1 == 0 && status_tilt_b2 == 1) {
    ros_msg.statustilt = 1;
  } else if (status_tilt_b1 == 1 && status_tilt_b2 == 0) {
    ros_msg.statustilt = 2;
  }
  /* status pos */
  if (status_pos_b1 == 0 && status_pos_b2 == 0) {
    ros_msg.statuspos = 0;
  } else if (status_pos_b1 == 0 && status_pos_b2 == 1) {
    ros_msg.statuspos = 1;
  } else if (status_pos_b1 == 1 && status_pos_b2 == 0) {
    ros_msg.statuspos = 2;
  }
}

/// \file
/// \brief  geterrorandwarning function - adma error and warning
/// \param  ros_msg ros message to fill with content
/// \param  ew_bytes array of bytes with several error and warnings
void ADMA2ROSParser::getErrorandWarning(
  adma_ros_driver_msgs::msg::AdmaData & ros_msg, unsigned char ew_bytes[4])
{
  std::bitset<8> bitdataerror1 = ew_bytes[0];
  std::bitset<8> bitdataerror2 = ew_bytes[1];
  std::bitset<8> bitdatawarn3 = ew_bytes[2];
  std::bitset<8> errorhw = ew_bytes[3];
  std::bitset<4> erhw1;
  std::bitset<4> ermisc1;
  std::bitset<4> ermisc2;
  std::bitset<4> ermisc3;
  std::bitset<4> warngps;
  std::bitset<4> warnmisc1;
  std::bitset<1> erhwsticky;

  for (size_t i = 0; i < 4; i++) {
    erhw1[i] = bitdataerror1[i];
    ermisc1[i] = bitdataerror1[i + 4];
    ermisc2[i] = bitdataerror2[i];
    ermisc3[i] = bitdataerror2[i + 4];
    warngps[i] = bitdatawarn3[i];
    warnmisc1[i] = bitdatawarn3[i + 4];
  }
  erhwsticky[0] = errorhw[1];
  ros_msg.errorhardware = erhw1.to_string();
  ros_msg.error_misc1 = ermisc1.to_string();
  ros_msg.error_misc2 = ermisc2.to_string();
  ros_msg.error_misc3 = ermisc3.to_string();
  ros_msg.warngps = warngps.to_string();
  ros_msg.warnmisc1 = warnmisc1.to_string();
  ros_msg.errorhwsticky = erhwsticky.to_string();
}

/// \file
/// \brief  pareScaledData function - fills scaled values with LSB factor
/// \param  ros_msg ros message to fill with content
void ADMA2ROSParser::parseScaledData(adma_ros_driver_msgs::msg::AdmaData & ros_msg)
{
  ros_msg.faccbodyhrx = getScaledValue(ros_msg.accbodyhrx, 0.0001);
  ros_msg.fratebodyhrx = getScaledValue(ros_msg.ratebodyhrx, 0.0001);
  ros_msg.faccbodyhry = getScaledValue(ros_msg.accbodyhry, 0.0001);
  ros_msg.fratebodyhry = getScaledValue(ros_msg.ratebodyhry, 0.0001);
  ros_msg.faccbodyhrz = getScaledValue(ros_msg.accbodyhrz, 0.0001);
  ros_msg.fratebodyhrz = getScaledValue(ros_msg.ratebodyhrz, 0.0001);

  ros_msg.fratebodyx = getScaledValue(ros_msg.ratebodyx, 0.01);
  ros_msg.fratebodyy = getScaledValue(ros_msg.ratebodyy, 0.01);
  ros_msg.fratebodyz = getScaledValue(ros_msg.ratebodyz, 0.01);
  ros_msg.fratehorx = getScaledValue(ros_msg.ratehorx, 0.01);
  ros_msg.fratehory = getScaledValue(ros_msg.ratehory, 0.01);
  ros_msg.fratehorz = getScaledValue(ros_msg.ratehorz, 0.01);

  ros_msg.faccbodyx = getScaledValue(ros_msg.accbodyx, 0.0004);
  ros_msg.faccbodyy = getScaledValue(ros_msg.accbodyy, 0.0004);
  ros_msg.faccbodyz = getScaledValue(ros_msg.accbodyz, 0.0004);
  ros_msg.facchorx = getScaledValue(ros_msg.acchorx, 0.0004);
  ros_msg.facchory = getScaledValue(ros_msg.acchory, 0.0004);
  ros_msg.facchorz = getScaledValue(ros_msg.acchorz, 0.0004);

  ros_msg.faccbodyx_1 = getScaledValue(ros_msg.accbodyx_1, 0.0004);
  ros_msg.faccbodyy_1 = getScaledValue(ros_msg.accbodyy_1, 0.0004);
  ros_msg.faccbodyz_1 = getScaledValue(ros_msg.accbodyz_1, 0.0004);
  ros_msg.faccbodyx_2 = getScaledValue(ros_msg.accbodyx_2, 0.0004);
  ros_msg.faccbodyy_2 = getScaledValue(ros_msg.accbodyy_2, 0.0004);
  ros_msg.faccbodyz_2 = getScaledValue(ros_msg.accbodyz_2, 0.0004);
  ros_msg.faccbodyx_3 = getScaledValue(ros_msg.accbodyx_3, 0.0004);
  ros_msg.faccbodyy_3 = getScaledValue(ros_msg.accbodyy_3, 0.0004);
  ros_msg.faccbodyz_3 = getScaledValue(ros_msg.accbodyz_3, 0.0004);
  ros_msg.faccbodyx_4 = getScaledValue(ros_msg.accbodyx_4, 0.0004);
  ros_msg.faccbodyy_4 = getScaledValue(ros_msg.accbodyy_4, 0.0004);
  ros_msg.faccbodyz_4 = getScaledValue(ros_msg.accbodyz_4, 0.0004);
  ros_msg.faccbodyx_5 = getScaledValue(ros_msg.accbodyx_5, 0.0004);
  ros_msg.faccbodyy_5 = getScaledValue(ros_msg.accbodyy_5, 0.0004);
  ros_msg.faccbodyz_5 = getScaledValue(ros_msg.accbodyz_5, 0.0004);
  ros_msg.faccbodyx_6 = getScaledValue(ros_msg.accbodyx_6, 0.0004);
  ros_msg.faccbodyy_6 = getScaledValue(ros_msg.accbodyy_6, 0.0004);
  ros_msg.faccbodyz_6 = getScaledValue(ros_msg.accbodyz_6, 0.0004);
  ros_msg.faccbodyx_7 = getScaledValue(ros_msg.accbodyx_7, 0.0004);
  ros_msg.faccbodyy_7 = getScaledValue(ros_msg.accbodyy_7, 0.0004);
  ros_msg.faccbodyz_7 = getScaledValue(ros_msg.accbodyz_7, 0.0004);

  ros_msg.facchorx_1 = getScaledValue(ros_msg.acchorx_1, 0.0004);
  ros_msg.facchory_1 = getScaledValue(ros_msg.acchory_1, 0.0004);
  ros_msg.facchorz_1 = getScaledValue(ros_msg.acchorz_1, 0.0004);
  ros_msg.facchorx_2 = getScaledValue(ros_msg.acchorx_2, 0.0004);
  ros_msg.facchory_2 = getScaledValue(ros_msg.acchory_2, 0.0004);
  ros_msg.facchorz_2 = getScaledValue(ros_msg.acchorz_2, 0.0004);
  ros_msg.facchorx_3 = getScaledValue(ros_msg.acchorx_3, 0.0004);
  ros_msg.facchory_3 = getScaledValue(ros_msg.acchory_3, 0.0004);
  ros_msg.facchorz_3 = getScaledValue(ros_msg.acchorz_3, 0.0004);
  ros_msg.facchorx_4 = getScaledValue(ros_msg.acchorx_4, 0.0004);
  ros_msg.facchory_4 = getScaledValue(ros_msg.acchory_4, 0.0004);
  ros_msg.facchorz_4 = getScaledValue(ros_msg.acchorz_4, 0.0004);
  ros_msg.facchorx_5 = getScaledValue(ros_msg.acchorx_5, 0.0004);
  ros_msg.facchory_5 = getScaledValue(ros_msg.acchory_5, 0.0004);
  ros_msg.facchorz_5 = getScaledValue(ros_msg.acchorz_5, 0.0004);
  ros_msg.facchorx_6 = getScaledValue(ros_msg.acchorx_6, 0.0004);
  ros_msg.facchory_6 = getScaledValue(ros_msg.acchory_6, 0.0004);
  ros_msg.facchorz_6 = getScaledValue(ros_msg.acchorz_6, 0.0004);
  ros_msg.facchorx_7 = getScaledValue(ros_msg.acchorx_7, 0.0004);
  ros_msg.facchory_7 = getScaledValue(ros_msg.acchory_7, 0.0004);
  ros_msg.facchorz_7 = getScaledValue(ros_msg.acchorz_7, 0.0004);

  ros_msg.fextvelanx = getScaledValue(ros_msg.extvelanx, 0.005);
  ros_msg.fextvelany = getScaledValue(ros_msg.extvelany, 0.005);
  ros_msg.fextveldigx = getScaledValue(ros_msg.extveldigx, 0.005);
  ros_msg.fextveldigy = getScaledValue(ros_msg.extveldigy, 0.005);
  ros_msg.fextvelxcorrected = getScaledValue(ros_msg.extvelxcorrected, 0.005);
  ros_msg.fextvelycorrected = getScaledValue(ros_msg.extvelycorrected, 0.005);

  ros_msg.fextbaropressure = getScaledValue(ros_msg.extbaropressure, 0.01);
  ros_msg.fextbaroheight = getScaledValue(ros_msg.extbaroheight, 0.01);
  ros_msg.fextbaroheightcorrected = getScaledValue(ros_msg.extbaroheightcorrected, 0.01);

  ros_msg.finvpathradius = getScaledValue(ros_msg.invpathradius, 0.0001);
  ros_msg.fsideslipangle = getScaledValue(ros_msg.sideslipangle, 0.01);
  ros_msg.fdisttrav = getScaledValue(ros_msg.disttrav, 0.01);

  ros_msg.finvpathradius_1 = getScaledValue(ros_msg.invpathradius_1, 0.0001);
  ros_msg.fsideslipangle_1 = getScaledValue(ros_msg.sideslipangle_1, 0.01);
  ros_msg.fdisttrav_1 = getScaledValue(ros_msg.disttrav_1, 0.01);
  ros_msg.finvpathradius_2 = getScaledValue(ros_msg.invpathradius_2, 0.0001);
  ros_msg.fsideslipangle_2 = getScaledValue(ros_msg.sideslipangle_2, 0.01);
  ros_msg.fdisttrav_2 = getScaledValue(ros_msg.disttrav_2, 0.01);
  ros_msg.finvpathradius_3 = getScaledValue(ros_msg.invpathradius_3, 0.0001);
  ros_msg.fsideslipangle_3 = getScaledValue(ros_msg.sideslipangle_3, 0.01);
  ros_msg.fdisttrav_3 = getScaledValue(ros_msg.disttrav_3, 0.01);
  ros_msg.finvpathradius_4 = getScaledValue(ros_msg.invpathradius_4, 0.0001);
  ros_msg.fsideslipangle_4 = getScaledValue(ros_msg.sideslipangle_4, 0.01);
  ros_msg.fdisttrav_4 = getScaledValue(ros_msg.disttrav_4, 0.01);
  ros_msg.finvpathradius_5 = getScaledValue(ros_msg.invpathradius_5, 0.0001);
  ros_msg.fsideslipangle_5 = getScaledValue(ros_msg.sideslipangle_5, 0.01);
  ros_msg.fdisttrav_5 = getScaledValue(ros_msg.disttrav_5, 0.01);
  ros_msg.finvpathradius_6 = getScaledValue(ros_msg.invpathradius_6, 0.0001);
  ros_msg.fsideslipangle_6 = getScaledValue(ros_msg.sideslipangle_6, 0.01);
  ros_msg.fdisttrav_6 = getScaledValue(ros_msg.disttrav_6, 0.01);
  ros_msg.finvpathradius_7 = getScaledValue(ros_msg.invpathradius_7, 0.0001);
  ros_msg.fsideslipangle_7 = getScaledValue(ros_msg.sideslipangle_7, 0.01);
  ros_msg.fdisttrav_7 = getScaledValue(ros_msg.disttrav_7, 0.01);

  ros_msg.fsystemtemp = getScaledValue(ros_msg.systemtemp, 0.1);
  ros_msg.fsystemdspload = getScaledValue(ros_msg.systemdspload, 0.1);

  ros_msg.fgpslatabs = getScaledValue(ros_msg.gpslatabs, 0.0000001);
  ros_msg.fgpslonabs = getScaledValue(ros_msg.gpslonabs, 0.0000001);
  ros_msg.fgpslatrel = getScaledValue(ros_msg.gpslatrel, 0.01);
  ros_msg.fgpslonrel = getScaledValue(ros_msg.gpslonrel, 0.01);

  ros_msg.fgpsstddevlat = getScaledValue(ros_msg.gpsstddevlat, 0.001);
  ros_msg.fgpsstddevlon = getScaledValue(ros_msg.gpsstddevlon, 0.001);
  ros_msg.fgpsstddevheight = getScaledValue(ros_msg.gpsstddevheight, 0.001);

  ros_msg.fgpsvelframex = getScaledValue(ros_msg.gpsvelframex, 0.005);
  ros_msg.fgpsvelframey = getScaledValue(ros_msg.gpsvelframey, 0.005);
  ros_msg.fgpsvelframez = getScaledValue(ros_msg.gpsvelframez, 0.005);
  ros_msg.fgpsvellatency = getScaledValue(ros_msg.gpsvellatency, 0.001);

  ros_msg.fgpsstddevvelx = getScaledValue(ros_msg.gpsstddevvelx, 0.001);
  ros_msg.fgpsstddevvely = getScaledValue(ros_msg.gpsstddevvely, 0.001);
  ros_msg.fgpsstddevvelz = getScaledValue(ros_msg.gpsstddevvelz, 0.001);

  ros_msg.fgpsdiffage = getScaledValue(ros_msg.gpsdiffage, 0.1);
  ros_msg.fgpsreceiverload = getScaledValue(ros_msg.gpsreceiverload, 0.5);

  ros_msg.finsroll = getScaledValue(ros_msg.insroll, 0.01);
  ros_msg.finspitch = getScaledValue(ros_msg.inspitch, 0.01);
  ros_msg.finsyaw = getScaledValue(ros_msg.insyaw, 0.01);
  ros_msg.fgpscog = getScaledValue(ros_msg.gpscog, 0.01);

  ros_msg.fgpsheight = getScaledValue(ros_msg.gpsheight, 0.01);
  ros_msg.fundulation = getScaledValue(ros_msg.undulation, 0.01);

  ros_msg.finsheight = getScaledValue(ros_msg.insheight, 0.01);
  ros_msg.finsheight_1 = getScaledValue(ros_msg.insheight_1, 0.01);
  ros_msg.finsheight_2 = getScaledValue(ros_msg.insheight_2, 0.01);
  ros_msg.finsheight_3 = getScaledValue(ros_msg.insheight_3, 0.01);
  ros_msg.finsheight_4 = getScaledValue(ros_msg.insheight_4, 0.01);
  ros_msg.finsheight_5 = getScaledValue(ros_msg.insheight_5, 0.01);
  ros_msg.finsheight_6 = getScaledValue(ros_msg.insheight_6, 0.01);
  ros_msg.finsheight_7 = getScaledValue(ros_msg.insheight_7, 0.01);

  ros_msg.finslatabs = getScaledValue(ros_msg.inslatabs, 0.0000001);
  ros_msg.finslonabs = getScaledValue(ros_msg.inslonabs, 0.0000001);
  ros_msg.finslatrel = getScaledValue(ros_msg.inslatrel, 0.01);
  ros_msg.finslonrel = getScaledValue(ros_msg.inslonrel, 0.01);
  ros_msg.finslatabs_1 = getScaledValue(ros_msg.inslatabs_1, 0.0000001);
  ros_msg.finslonabs_1 = getScaledValue(ros_msg.inslonabs_1, 0.0000001);
  ros_msg.finslatrel_1 = getScaledValue(ros_msg.inslatrel_1, 0.01);
  ros_msg.finslonrel_1 = getScaledValue(ros_msg.inslonrel_1, 0.01);
  ros_msg.finslatabs_2 = getScaledValue(ros_msg.inslatabs_2, 0.0000001);
  ros_msg.finslonabs_2 = getScaledValue(ros_msg.inslonabs_2, 0.0000001);
  ros_msg.finslatrel_2 = getScaledValue(ros_msg.inslatrel_2, 0.01);
  ros_msg.finslonrel_2 = getScaledValue(ros_msg.inslonrel_2, 0.01);
  ros_msg.finslatabs_3 = getScaledValue(ros_msg.inslatabs_3, 0.0000001);
  ros_msg.finslonabs_3 = getScaledValue(ros_msg.inslonabs_3, 0.0000001);
  ros_msg.finslatrel_3 = getScaledValue(ros_msg.inslatrel_3, 0.01);
  ros_msg.finslonrel_3 = getScaledValue(ros_msg.inslonrel_3, 0.01);
  ros_msg.finslatabs_4 = getScaledValue(ros_msg.inslatabs_4, 0.0000001);
  ros_msg.finslonabs_4 = getScaledValue(ros_msg.inslonabs_4, 0.0000001);
  ros_msg.finslatrel_4 = getScaledValue(ros_msg.inslatrel_4, 0.01);
  ros_msg.finslonrel_4 = getScaledValue(ros_msg.inslonrel_4, 0.01);
  ros_msg.finslatabs_5 = getScaledValue(ros_msg.inslatabs_5, 0.0000001);
  ros_msg.finslonabs_5 = getScaledValue(ros_msg.inslonabs_5, 0.0000001);
  ros_msg.finslatrel_5 = getScaledValue(ros_msg.inslatrel_5, 0.01);
  ros_msg.finslonrel_5 = getScaledValue(ros_msg.inslonrel_5, 0.01);
  ros_msg.finslatabs_6 = getScaledValue(ros_msg.inslatabs_6, 0.0000001);
  ros_msg.finslonabs_6 = getScaledValue(ros_msg.inslonabs_6, 0.0000001);
  ros_msg.finslatrel_6 = getScaledValue(ros_msg.inslatrel_6, 0.01);
  ros_msg.finslonrel_6 = getScaledValue(ros_msg.inslonrel_6, 0.01);
  ros_msg.finslatabs_7 = getScaledValue(ros_msg.inslatabs_7, 0.0000001);
  ros_msg.finslonabs_7 = getScaledValue(ros_msg.inslonabs_7, 0.0000001);
  ros_msg.finslatrel_7 = getScaledValue(ros_msg.inslatrel_7, 0.01);
  ros_msg.finslonrel_7 = getScaledValue(ros_msg.inslonrel_7, 0.01);

  ros_msg.finsvelhorx = getScaledValue(ros_msg.insvelhorx, 0.005);
  ros_msg.finsvelhory = getScaledValue(ros_msg.insvelhory, 0.005);
  ros_msg.finsvelhorz = getScaledValue(ros_msg.insvelhorz, 0.005);
  ros_msg.finsvelframex = getScaledValue(ros_msg.insvelframex, 0.005);
  ros_msg.finsvelframey = getScaledValue(ros_msg.insvelframey, 0.005);
  ros_msg.finsvelframez = getScaledValue(ros_msg.insvelframez, 0.005);

  ros_msg.finsvelhorx_1 = getScaledValue(ros_msg.insvelhorx_1, 0.005);
  ros_msg.finsvelhory_1 = getScaledValue(ros_msg.insvelhory_1, 0.005);
  ros_msg.finsvelhorz_1 = getScaledValue(ros_msg.insvelhorz_1, 0.005);
  ros_msg.finsvelhorx_2 = getScaledValue(ros_msg.insvelhorx_2, 0.005);
  ros_msg.finsvelhory_2 = getScaledValue(ros_msg.insvelhory_2, 0.005);
  ros_msg.finsvelhorz_2 = getScaledValue(ros_msg.insvelhorz_2, 0.005);
  ros_msg.finsvelhorx_3 = getScaledValue(ros_msg.insvelhorx_3, 0.005);
  ros_msg.finsvelhory_3 = getScaledValue(ros_msg.insvelhory_3, 0.005);
  ros_msg.finsvelhorz_3 = getScaledValue(ros_msg.insvelhorz_3, 0.005);
  ros_msg.finsvelhorx_4 = getScaledValue(ros_msg.insvelhorx_4, 0.005);
  ros_msg.finsvelhory_4 = getScaledValue(ros_msg.insvelhory_4, 0.005);
  ros_msg.finsvelhorz_4 = getScaledValue(ros_msg.insvelhorz_4, 0.005);
  ros_msg.finsvelhorx_5 = getScaledValue(ros_msg.insvelhorx_5, 0.005);
  ros_msg.finsvelhory_5 = getScaledValue(ros_msg.insvelhory_5, 0.005);
  ros_msg.finsvelhorz_5 = getScaledValue(ros_msg.insvelhorz_5, 0.005);
  ros_msg.finsvelhorx_6 = getScaledValue(ros_msg.insvelhorx_6, 0.005);
  ros_msg.finsvelhory_6 = getScaledValue(ros_msg.insvelhory_6, 0.005);
  ros_msg.finsvelhorz_6 = getScaledValue(ros_msg.insvelhorz_6, 0.005);
  ros_msg.finsvelhorx_7 = getScaledValue(ros_msg.insvelhorx_7, 0.005);
  ros_msg.finsvelhory_7 = getScaledValue(ros_msg.insvelhory_7, 0.005);
  ros_msg.finsvelhorz_7 = getScaledValue(ros_msg.insvelhorz_7, 0.005);

  ros_msg.finsstddevlat = getScaledValue(ros_msg.insstddevlat, 0.01);
  ros_msg.finsstddevlong = getScaledValue(ros_msg.insstddevlong, 0.01);
  ros_msg.finsstddevheight = getScaledValue(ros_msg.insstddevheight, 0.01);

  ros_msg.finsstddevvelx = getScaledValue(ros_msg.insstddevvelx, 0.01);
  ros_msg.finsstddevvely = getScaledValue(ros_msg.insstddevvely, 0.01);
  ros_msg.finsstddevvelz = getScaledValue(ros_msg.insstddevvelz, 0.01);
  ros_msg.finsstddevroll = getScaledValue(ros_msg.insstddevroll, 0.01);
  ros_msg.finsstddevpitch = getScaledValue(ros_msg.insstddevpitch, 0.01);
  ros_msg.finsstddevyaw = getScaledValue(ros_msg.insstddevyaw, 0.01);

  ros_msg.fan1 = getScaledValue(ros_msg.an1, 0.0005);
  ros_msg.fan2 = getScaledValue(ros_msg.an2, 0.0005);
  ros_msg.fan3 = getScaledValue(ros_msg.an3, 0.0005);
  ros_msg.fan4 = getScaledValue(ros_msg.an4, 0.0005);

  // only for >= v3.3.3
  ros_msg.fgpsdualantheading = getScaledValue(ros_msg.gpsdualantheading, 0.01);
  ros_msg.fgpsdualantpitch = getScaledValue(ros_msg.gpsdualantpitch, 0.01);
  ros_msg.fgpsdualantstddevheading = getScaledValue(ros_msg.gpsdualantstddevheading, 0.01);
  ros_msg.fgpsdualantstddevpitch = getScaledValue(ros_msg.gpsdualantstddevpitch, 0.01);
  ros_msg.fgpsdualantstddevheading_hr = getScaledValue(ros_msg.gpsdualantstddevheading_hr, 0.01);
  ros_msg.fgpsdualantstddevpitch_hr = getScaledValue(ros_msg.gpsdualantstddevpitch_hr, 0.01);
  ros_msg.faccbodyx_8 = getScaledValue(ros_msg.accbodyx_8, 0.0004);
  ros_msg.faccbodyy_8 = getScaledValue(ros_msg.accbodyy_8, 0.0004);
  ros_msg.faccbodyz_8 = getScaledValue(ros_msg.accbodyz_8, 0.0004);
  ros_msg.facchorx_8 = getScaledValue(ros_msg.acchorx_8, 0.0004);
  ros_msg.facchory_8 = getScaledValue(ros_msg.acchory_8, 0.0004);
  ros_msg.facchorz_8 = getScaledValue(ros_msg.acchorz_8, 0.0004);
  ros_msg.finvpathradius_8 = getScaledValue(ros_msg.invpathradius_8, 0.0001);
  ros_msg.fsideslipangle_8 = getScaledValue(ros_msg.sideslipangle_8, 0.01);
  ros_msg.fdisttrav_8 = getScaledValue(ros_msg.disttrav_8, 0.01);
  ros_msg.finsheight_8 = getScaledValue(ros_msg.insheight_8, 0.01);
  ros_msg.finslatabs_8 = getScaledValue(ros_msg.inslatabs_8, 0.0000001);
  ros_msg.finslonabs_8 = getScaledValue(ros_msg.inslonabs_8, 0.0000001);
  ros_msg.finslatrel_8 = getScaledValue(ros_msg.inslatrel_8, 0.01);
  ros_msg.finslonrel_8 = getScaledValue(ros_msg.inslonrel_8, 0.01);
  ros_msg.finsvelhorx_8 = getScaledValue(ros_msg.insvelhorx_8, 0.005);
  ros_msg.finsvelhory_8 = getScaledValue(ros_msg.insvelhory_8, 0.005);
  ros_msg.finsvelhorz_8 = getScaledValue(ros_msg.insvelhorz_8, 0.005);
}

void ADMA2ROSParser::extractNavSatFix(
  adma_ros_driver_msgs::msg::AdmaData & ros_msg, sensor_msgs::msg::NavSatFix & nav_ros_msg)
{
  // fil status
  switch (ros_msg.statusgpsmode) {
    case 1:
      // No GNSS Data
      nav_ros_msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
      break;
    case 2:
      // single GNSS
      nav_ros_msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
      break;
    case 4:
      // actually DGNSS Coarse Mode, but used to distinguish here
      nav_ros_msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_SBAS_FIX;
      break;
    case 8:
      // DGNSS Precise Mode
      nav_ros_msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX;
      break;
    default:
      break;
  }

  nav_ros_msg.altitude = ros_msg.finsheight + ros_msg.undulation;
  nav_ros_msg.latitude = ros_msg.finslatabs;
  nav_ros_msg.longitude = ros_msg.finslonabs;
  nav_ros_msg.position_covariance[0] = std::pow(ros_msg.finsstddevlat, 2);
  nav_ros_msg.position_covariance[4] = std::pow(ros_msg.finsstddevlong, 2);
  nav_ros_msg.position_covariance[8] = std::pow(ros_msg.finsstddevheight, 2);

  nav_ros_msg.position_covariance_type =
    sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
}

void ADMA2ROSParser::extractNavSatFix(
  adma_ros_driver_msgs::msg::AdmaDataScaled & ros_msg, sensor_msgs::msg::NavSatFix & nav_ros_msg,
  std::array<adma_ros_driver_msgs::msg::POI, 8> &pois, uint8_t desiredSource)
{
  // fil status
  switch (ros_msg.status.status_gnss_mode) {
    case 1:
      // No GNSS Data
      nav_ros_msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
      break;
    case 2:
      // single GNSS
      nav_ros_msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
      break;
    case 4:
      // actually DGNSS Coarse Mode, but used to distinguish here
      nav_ros_msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_SBAS_FIX;
      break;
    case 8:
      // DGNSS Precise Mode
      nav_ros_msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX;
      break;
    default:
      break;
  }

  // read POI specific height for NavSatFix msg
  nav_ros_msg.altitude = desiredSource == 0 ? ros_msg.ins_height : pois[desiredSource - 1].ins_height;
  nav_ros_msg.latitude = desiredSource == 0 ? ros_msg.ins_lat_abs : pois[desiredSource - 1].ins_lat_abs;
  nav_ros_msg.longitude = desiredSource == 0 ? ros_msg.ins_long_abs : pois[desiredSource - 1].ins_lon_abs;
  // add undulation to get WGS84 height for ROS standard
  nav_ros_msg.altitude += ros_msg.undulation;
  nav_ros_msg.position_covariance[0] = std::pow(ros_msg.ins_stddev_lat, 2);
  nav_ros_msg.position_covariance[4] = std::pow(ros_msg.ins_stddev_long, 2);
  nav_ros_msg.position_covariance[8] = std::pow(ros_msg.ins_stddev_height, 2);

  nav_ros_msg.position_covariance_type =
    sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
}

void ADMA2ROSParser::extractIMU(
  adma_ros_driver_msgs::msg::AdmaData & ros_msg, sensor_msgs::msg::Imu & imu_ros_msg)
{
  imu_ros_msg.linear_acceleration.x = ros_msg.faccbodyhrx * 9.81;
  imu_ros_msg.linear_acceleration.y = ros_msg.faccbodyhry * 9.81;
  imu_ros_msg.linear_acceleration.z = ros_msg.faccbodyhrz * 9.81;

  imu_ros_msg.angular_velocity.x = deg2Rad(ros_msg.fratebodyhrx);
  imu_ros_msg.angular_velocity.y = deg2Rad(ros_msg.fratebodyhry);
  imu_ros_msg.angular_velocity.z = deg2Rad(ros_msg.fratebodyhrz);

  tf2::Quaternion q;
  double roll_rad = deg2Rad(ros_msg.finsroll);
  double pitch_rad = deg2Rad(ros_msg.finspitch);
  double yaw_rad = deg2Rad(ros_msg.finsyaw);
  q.setRPY(roll_rad, pitch_rad, yaw_rad);
  imu_ros_msg.orientation = tf2::toMsg(q);

  imu_ros_msg.orientation_covariance[0] = std::pow(deg2Rad(ros_msg.finsstddevroll), 2);
  imu_ros_msg.orientation_covariance[4] = std::pow(deg2Rad(ros_msg.finsstddevpitch), 2);
  imu_ros_msg.orientation_covariance[8] = std::pow(deg2Rad(ros_msg.finsstddevyaw), 2);

  // ADMA does not provide covariance for linear acceleration and angular velocity.
  // These values need to be measured at standstill each ADMA model.
  imu_ros_msg.angular_velocity_covariance[0] = -1;
  imu_ros_msg.linear_acceleration_covariance[0] = -1;
}

void ADMA2ROSParser::extractIMU(
  adma_ros_driver_msgs::msg::AdmaDataScaled & ros_msg, sensor_msgs::msg::Imu & imu_ros_msg,
  std::array<adma_ros_driver_msgs::msg::POI, 8> &pois, uint8_t desiredSource)
{
  // get POI specific IMU data
  imu_ros_msg.linear_acceleration.x = desiredSource == 0 ? ros_msg.acc_body_hr.x : pois[desiredSource - 1].acc_body.x;
  imu_ros_msg.linear_acceleration.y = desiredSource == 0 ? ros_msg.acc_body_hr.y : pois[desiredSource - 1].acc_body.y;
  imu_ros_msg.linear_acceleration.z = desiredSource == 0 ? ros_msg.acc_body_hr.z : pois[desiredSource - 1].acc_body.z;
  // convert to m/s²
  imu_ros_msg.linear_acceleration.x *= 9.81;
  imu_ros_msg.linear_acceleration.y *= 9.81;
  imu_ros_msg.linear_acceleration.z *= 9.81;

  imu_ros_msg.angular_velocity.x = deg2Rad(ros_msg.rate_body_hr.x);
  imu_ros_msg.angular_velocity.y = deg2Rad(ros_msg.rate_body_hr.y);
  imu_ros_msg.angular_velocity.z = deg2Rad(ros_msg.rate_body_hr.z);

  tf2::Quaternion q;
  double roll_rad = deg2Rad(ros_msg.ins_roll);
  double pitch_rad = deg2Rad(ros_msg.ins_pitch);
  double yaw_rad = deg2Rad(ros_msg.ins_yaw);
  q.setRPY(roll_rad, pitch_rad, yaw_rad);
  imu_ros_msg.orientation = tf2::toMsg(q);

  imu_ros_msg.orientation_covariance[0] = std::pow(deg2Rad(ros_msg.ins_stddev_roll), 2);
  imu_ros_msg.orientation_covariance[4] = std::pow(deg2Rad(ros_msg.ins_stddev_pitch), 2);
  imu_ros_msg.orientation_covariance[8] = std::pow(deg2Rad(ros_msg.ins_stddev_yaw), 2);

  // ADMA does not provide covariance for linear acceleration and angular velocity.
  // These values need to be measured at standstill each ADMA model.
  imu_ros_msg.angular_velocity_covariance[0] = -1;
  imu_ros_msg.linear_acceleration_covariance[0] = -1;
}

void ADMA2ROSParser::extractOdometry(
    adma_ros_driver_msgs::msg::AdmaDataScaled & ros_msg, nav_msgs::msg::Odometry & odometry_msg,
    double yawOffset, std::array<adma_ros_driver_msgs::msg::POI, 8> &pois, uint8_t desiredSource)
{
  // extract POI specific odometry data
  odometry_msg.pose.pose.position.x = desiredSource == 0 ? ros_msg.ins_pos_rel_x : pois[desiredSource - 1].ins_pos_rel_x;
  odometry_msg.pose.pose.position.y = desiredSource == 0 ? ros_msg.ins_pos_rel_y : pois[desiredSource - 1].ins_pos_rel_y;
  odometry_msg.pose.pose.position.z = desiredSource == 0 ? ros_msg.ins_height : pois[desiredSource - 1].ins_height;

  double roll_rad = deg2Rad(ros_msg.ins_roll);
  double pitch_rad = deg2Rad(ros_msg.ins_pitch);
  double yaw_rad = deg2Rad((ros_msg.ins_yaw + yawOffset));
  tf2::Quaternion q;
  q.setRPY(roll_rad, pitch_rad, yaw_rad);
  odometry_msg.pose.pose.orientation = tf2::toMsg(q);

  odometry_msg.pose.covariance[21] = std::pow(deg2Rad(ros_msg.ins_stddev_roll), 2);
  odometry_msg.pose.covariance[28] = std::pow(deg2Rad(ros_msg.ins_stddev_pitch), 2);
  odometry_msg.pose.covariance[35] = std::pow(deg2Rad(ros_msg.ins_stddev_yaw), 2);

  odometry_msg.twist.twist.linear.x = desiredSource == 0 ? ros_msg.ins_vel_hor.x : pois[desiredSource - 1].ins_vel_hor.x;
  odometry_msg.twist.twist.linear.y = desiredSource == 0 ? ros_msg.ins_vel_hor.y : pois[desiredSource - 1].ins_vel_hor.y;
  odometry_msg.twist.twist.linear.z = desiredSource == 0 ? ros_msg.ins_vel_hor.z : pois[desiredSource - 1].ins_vel_hor.z;
  odometry_msg.twist.twist.angular.x = deg2Rad(ros_msg.rate_body.x);
  odometry_msg.twist.twist.angular.y = deg2Rad(ros_msg.rate_body.y);
  odometry_msg.twist.twist.angular.z = deg2Rad(ros_msg.rate_body.z);
  odometry_msg.twist.covariance[0] = std::pow(ros_msg.ins_stddev_vel.x, 2);
  odometry_msg.twist.covariance[7] = std::pow(ros_msg.ins_stddev_vel.y, 2);
  odometry_msg.twist.covariance[14] = std::pow(ros_msg.ins_stddev_vel.z, 2);
  
}
