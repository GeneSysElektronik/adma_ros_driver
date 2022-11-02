#include "adma_ros2_driver/parser/adma2ros_parser.hpp"
#include "adma_ros2_driver/parser/parser_utils.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <iostream>

ADMA2ROSParser::ADMA2ROSParser(std::string version)
        : _parserV32(), 
        _parserV333(), 
        _version(version)
{
}

void ADMA2ROSParser::mapAdmaMessageToROS(adma_msgs::msg::AdmaData& rosMsg, std::vector<char>& localdata)
{

        if(_version == "v3.2")
        {       
                AdmaDataV32 admaData;
                memcpy(&admaData , &localdata, sizeof(admaData));
                parseStaticHeader(rosMsg, admaData.staticHeader);
                parseDynamicHeader(rosMsg, admaData.dynamicHeader);
                getstatusgps(rosMsg, admaData.gpsStatus);
                getstatustrigger(rosMsg, admaData.gpsTriggerStatus);
                getevkstatus(rosMsg, admaData.evkStatus);
                unsigned char ewBytes[] = {admaData.dataError1, admaData.dataError2, admaData.dataWarn1, admaData.dataErrorHW};
                geterrorandwarning(rosMsg, ewBytes);
                _parserV32.mapAdmaMessageToROS(rosMsg, admaData);
        }else if (_version == "v3.3.3")
        {
                AdmaDataV333 admaData;
                memcpy(&admaData , &localdata, sizeof(admaData));
                parseStaticHeader(rosMsg, admaData.staticHeader);
                parseDynamicHeader(rosMsg, admaData.dynamicHeader);
                getstatusgps(rosMsg, admaData.gnssStatus);
                getstatustrigger(rosMsg, admaData.signalInStatus);
                getevkstatus(rosMsg, admaData.miscStatus);
                _parserV333.getKFStatus(rosMsg, admaData.kfStatus);
                unsigned char ewBytes[] = {admaData.dataError1, admaData.dataError2, admaData.dataWarn1, admaData.dataError3};
                geterrorandwarning(rosMsg, ewBytes);
                _parserV333.mapAdmaMessageToROS(rosMsg, admaData);
        }
        parseScaledData(rosMsg);
}

void ADMA2ROSParser::parseV32(adma_msgs::msg::AdmaData& rosMsg, std::vector<char>& localData)
{
        AdmaDataV32 admaData;
        parseStaticHeader(rosMsg, admaData.staticHeader);
        parseDynamicHeader(rosMsg, admaData.dynamicHeader);
}

void ADMA2ROSParser::parseV333(adma_msgs::msg::AdmaData& rosMsg, std::vector<char>& localData)
{
        
}

template <typename AdmaDataStruct>
void ADMA2ROSParser::parseStaticHeader(adma_msgs::msg::AdmaData& rosMsg, AdmaDataStruct& staticHeader)
{
    // fill static header information
    rosMsg.genesysid = staticHeader.genesysid;
    std::stringstream ss;
    ss <<  int(staticHeader.headerversion[0]) << int(staticHeader.headerversion[1]) << int(staticHeader.headerversion[2]) << int(staticHeader.headerversion[3]);
    rosMsg.headerversion = ss.str();
    ss.clear();
    ss.str("");
    rosMsg.formatid = staticHeader.formatid;
    //TODO: this value is parsed wrong?!        
    ss <<  int(staticHeader.formatversion[0]) << int(staticHeader.formatversion[1]) << int(staticHeader.formatversion[2]) << int(staticHeader.formatversion[3]);
    rosMsg.formatversion = ss.str();
    rosMsg.serialno = staticHeader.serialno;
}

template <typename AdmaDataStruct>
void ADMA2ROSParser::parseDynamicHeader(adma_msgs::msg::AdmaData& rosMsg, AdmaDataStruct& dynamicHeader)
{
    // fill dynamic header information
        rosMsg.configid = dynamicHeader.configid;
        rosMsg.configformat = dynamicHeader.configformat;
        rosMsg.configversion = dynamicHeader.configversion;
        rosMsg.configsize = dynamicHeader.configsize;
        rosMsg.byteoffset = dynamicHeader.byteoffset;
        rosMsg.slicesize = dynamicHeader.slicesize;
        rosMsg.slicedata = dynamicHeader.slicedata;
}


/// \file
/// \brief  getstatusgps function - adma status information
/// \param  rosMsg ros message to fill with content
/// \param  gpsStatus byte with gps states
void ADMA2ROSParser::getstatusgps(adma_msgs::msg::AdmaData& rosMsg, unsigned char gpsStatus)
{
    bool status_external_vel = getbit(gpsStatus,7);
    bool status_skidding = getbit(gpsStatus,5);
    bool standstill_c = getbit(gpsStatus,4);
    bool rtk_precise = getbit(gpsStatus,3);
    bool rtk_coarse = getbit(gpsStatus,2);
    bool gps_mode = getbit(gpsStatus,1);
    bool gps_out = getbit(gpsStatus,0);

    /* status gps mode */
    if(gps_out)
    {
        rosMsg.statusgpsmode = 1;
        // msg_fix.status.status = sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX; // No GNSS Data
    }
    else if (gps_mode) 
    {
        rosMsg.statusgpsmode = 2;
        // msg_fix.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX; // single GNSS
    }
    else if (rtk_coarse) 
    {
        rosMsg.statusgpsmode = 4;
        // msg_fix.status.status = sensor_msgs::msg::NavSatStatus::STATUS_SBAS_FIX; // actually DGNSS Coarse Mode, but used to distinguish here 
    }
    else if (rtk_precise) 
    {
        rosMsg.statusgpsmode = 8;
        // msg_fix.status.status = sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX; // DGNSS Precise Mode
    }
    /* status stand still */
    rosMsg.statusstandstill = standstill_c;
    /* status skidding */
    rosMsg.statusskidding = status_skidding;
    /* status external velocity slip */
    rosMsg.statusexternalvelout = status_external_vel;
}

/// \file
/// \brief  getstatustrigger function - adma gps trigger information
/// \param  rosMsg ros message to fill with content
/// \param  gpsTriggerStatus byte with gps trigger states
void ADMA2ROSParser::getstatustrigger(adma_msgs::msg::AdmaData& rosMsg, unsigned char gpsTriggerStatus)
{
    bool status_synclock = getbit(gpsTriggerStatus,7);
    bool status_dead_reckoning = getbit(gpsTriggerStatus,6);
    bool status_ahrs_ins = getbit(gpsTriggerStatus,5);
    bool status_alignment = getbit(gpsTriggerStatus,4);
    bool status_signal_in1 = getbit(gpsTriggerStatus,3);
    bool status_signal_in2 = getbit(gpsTriggerStatus,2);
    bool status_signal_in3 = getbit(gpsTriggerStatus,1);
    bool status_trig_gps = getbit(gpsTriggerStatus,0);
    /* status statustriggps */
    rosMsg.statustriggps = status_trig_gps;
    /* status statussignalin3 */
    rosMsg.statussignalin3 = status_signal_in3;
    /* status statussignalin2 */
    rosMsg.statussignalin2 = status_signal_in2;
    /* status statussignalin1 */
    rosMsg.statussignalin1 = status_signal_in1;
    /* status statusalignment */
    rosMsg.statusalignment = status_alignment;
    /* status statusahrsins */
    rosMsg.statusahrsins = status_ahrs_ins;
    /* status statusdeadreckoning */
    rosMsg.statusdeadreckoning = status_dead_reckoning;
    /* status statussynclock */
    rosMsg.statussynclock = status_synclock;
}

/// \file
/// \brief  getstatustrigger function - adma gps trigger information
/// \param  rosMsg ros message to fill with content
/// \param  evkStatus byte with evk states
void ADMA2ROSParser::getevkstatus(adma_msgs::msg::AdmaData& rosMsg, unsigned char evkStatus)
{
    bool status_pos_b2 = getbit(evkStatus,7);
    bool status_pos_b1 = getbit(evkStatus,6);
    bool status_tilt_b2 = getbit(evkStatus,5);
    bool status_tilt_b1 = getbit(evkStatus,4);
    bool status_configuration_changed = getbit(evkStatus,3);
    bool status_heading_executed = getbit(evkStatus,2);
    bool status_evk_estimates = getbit(evkStatus,1);
    bool status_evk_activ = getbit(evkStatus,0);
    /* status statustriggps */
    rosMsg.statusevkactiv = status_evk_activ;
    /* status status_evk_estimates */
    rosMsg.statusevkestimates = status_evk_estimates;
    /* status status_heading_executed */
    rosMsg.statusheadingexecuted = status_heading_executed;
    /* status status_configuration_changed */
    rosMsg.statusconfigurationchanged = status_configuration_changed;
    /* status tilt */
    if(status_tilt_b1==0 && status_tilt_b2==0)
    {
        rosMsg.statustilt = 0;
    }
    else if(status_tilt_b1==0 && status_tilt_b2==1)
    {
        rosMsg.statustilt = 1;
    }
    else if(status_tilt_b1==1 && status_tilt_b2==0)
    {
        rosMsg.statustilt = 2;
    }
    /* status pos */
    if(status_pos_b1==0 && status_pos_b2==0)
    {
        rosMsg.statuspos = 0;
    }
    else if(status_pos_b1==0 && status_pos_b2==1)
    {
        rosMsg.statuspos = 1;
    }
    else if(status_pos_b1==1 && status_pos_b2==0)
    {
        rosMsg.statuspos = 2;
    }
}

/// \file
/// \brief  geterrorandwarning function - adma error and warning
/// \param  rosMsg ros message to fill with content
/// \param  ewBytes array of bytes with several error and warnings
void ADMA2ROSParser::geterrorandwarning(adma_msgs::msg::AdmaData& rosMsg, unsigned char ewBytes[4])
{
    std::bitset<8> bitdataerror1 = ewBytes[0];
    std::bitset<8> bitdataerror2 = ewBytes[1];
    std::bitset<8> bitdatawarn3 = ewBytes[2];
    std::bitset<8> errorhw = ewBytes[3];
    std::bitset<4> erhw1;
    std::bitset<4> ermisc1;
    std::bitset<4> ermisc2;
    std::bitset<4> ermisc3;
    std::bitset<4> warngps;
    std::bitset<4> warnmisc1;
    std::bitset<1> erhwsticky;

    for(size_t i=0;i<4;i++)
    {
        erhw1[i]    = bitdataerror1[i];
        ermisc1[i]  = bitdataerror1[i+4];
        ermisc2[i]  = bitdataerror2[i];
        ermisc3[i]  = bitdataerror2[i+4];
        warngps[i]  = bitdatawarn3[i];
        warnmisc1[i]  = bitdatawarn3[i+4];
    }
    erhwsticky[0] = errorhw[1];
    rosMsg.errorhardware = erhw1.to_string();
    rosMsg.error_misc1 = ermisc1.to_string();
    rosMsg.error_misc2 = ermisc2.to_string();
    rosMsg.error_misc3 = ermisc3.to_string();
    rosMsg.warngps = warngps.to_string();
    rosMsg.warnmisc1 = warnmisc1.to_string();
    rosMsg.errorhwsticky = erhwsticky.to_string();
}

/// \file
/// \brief  pareScaledData function - fills scaled values with LSB factor
/// \param  rosMsg ros message to fill with content
void ADMA2ROSParser::parseScaledData(adma_msgs::msg::AdmaData& rosMsg)
{
        rosMsg.faccbodyhrx = getScaledValue(rosMsg.accbodyhrx, 0.0001);
        rosMsg.fratebodyhrx = getScaledValue(rosMsg.ratebodyhrx, 0.0001);
        rosMsg.faccbodyhry = getScaledValue(rosMsg.accbodyhry, 0.0001);
        rosMsg.fratebodyhry = getScaledValue(rosMsg.ratebodyhry, 0.0001);
        rosMsg.faccbodyhrz = getScaledValue(rosMsg.accbodyhrz, 0.0001);
        rosMsg.fratebodyhrz = getScaledValue(rosMsg.ratebodyhrz, 0.0001);

        rosMsg.fratebodyx = getScaledValue(rosMsg.ratebodyx, 0.01);
        rosMsg.fratebodyy = getScaledValue(rosMsg.ratebodyy, 0.01);
        rosMsg.fratebodyz = getScaledValue(rosMsg.ratebodyz, 0.01);
        rosMsg.fratehorx = getScaledValue(rosMsg.ratehorx, 0.01);
        rosMsg.fratehory = getScaledValue(rosMsg.ratehory, 0.01);
        rosMsg.fratehorz = getScaledValue(rosMsg.ratehorz, 0.01);

        rosMsg.faccbodyx = getScaledValue(rosMsg.accbodyx, 0.0004);
        rosMsg.faccbodyy = getScaledValue(rosMsg.accbodyy, 0.0004);
        rosMsg.faccbodyz = getScaledValue(rosMsg.accbodyz, 0.0004);
        rosMsg.facchorx = getScaledValue(rosMsg.acchorx, 0.0004);
        rosMsg.facchory = getScaledValue(rosMsg.acchory, 0.0004);
        rosMsg.facchorz = getScaledValue(rosMsg.acchorz, 0.0004);

        rosMsg.faccbodyx_1 = getScaledValue(rosMsg.accbodyx_1, 0.0004);
        rosMsg.faccbodyy_1 = getScaledValue(rosMsg.accbodyy_1, 0.0004);
        rosMsg.faccbodyz_1 = getScaledValue(rosMsg.accbodyz_1, 0.0004);
        rosMsg.faccbodyx_2 = getScaledValue(rosMsg.accbodyx_2, 0.0004);
        rosMsg.faccbodyy_2 = getScaledValue(rosMsg.accbodyy_2, 0.0004);
        rosMsg.faccbodyz_2 = getScaledValue(rosMsg.accbodyz_2, 0.0004);
        rosMsg.faccbodyx_3 = getScaledValue(rosMsg.accbodyx_3, 0.0004);
        rosMsg.faccbodyy_3 = getScaledValue(rosMsg.accbodyy_3, 0.0004);
        rosMsg.faccbodyz_3 = getScaledValue(rosMsg.accbodyz_3, 0.0004);
        rosMsg.faccbodyx_4 = getScaledValue(rosMsg.accbodyx_4, 0.0004);
        rosMsg.faccbodyy_4 = getScaledValue(rosMsg.accbodyy_4, 0.0004);
        rosMsg.faccbodyz_4 = getScaledValue(rosMsg.accbodyz_4, 0.0004);
        rosMsg.faccbodyx_5 = getScaledValue(rosMsg.accbodyx_5, 0.0004);
        rosMsg.faccbodyy_5 = getScaledValue(rosMsg.accbodyy_5, 0.0004);
        rosMsg.faccbodyz_5 = getScaledValue(rosMsg.accbodyz_5, 0.0004);
        rosMsg.faccbodyx_6 = getScaledValue(rosMsg.accbodyx_6, 0.0004);
        rosMsg.faccbodyy_6 = getScaledValue(rosMsg.accbodyy_6, 0.0004);
        rosMsg.faccbodyz_6 = getScaledValue(rosMsg.accbodyz_6, 0.0004);
        rosMsg.faccbodyx_7 = getScaledValue(rosMsg.accbodyx_7, 0.0004);
        rosMsg.faccbodyy_7 = getScaledValue(rosMsg.accbodyy_7, 0.0004);
        rosMsg.faccbodyz_7 = getScaledValue(rosMsg.accbodyz_7, 0.0004);

        rosMsg.facchorx_1 = getScaledValue(rosMsg.acchorx_1, 0.0004);
        rosMsg.facchory_1 = getScaledValue(rosMsg.acchory_1, 0.0004);
        rosMsg.facchorz_1 = getScaledValue(rosMsg.acchorz_1, 0.0004);
        rosMsg.facchorx_2 = getScaledValue(rosMsg.acchorx_2, 0.0004);
        rosMsg.facchory_2 = getScaledValue(rosMsg.acchory_2, 0.0004);
        rosMsg.facchorz_2 = getScaledValue(rosMsg.acchorz_2, 0.0004);
        rosMsg.facchorx_3 = getScaledValue(rosMsg.acchorx_3, 0.0004);
        rosMsg.facchory_3 = getScaledValue(rosMsg.acchory_3, 0.0004);
        rosMsg.facchorz_3 = getScaledValue(rosMsg.acchorz_3, 0.0004);
        rosMsg.facchorx_4 = getScaledValue(rosMsg.acchorx_4, 0.0004);
        rosMsg.facchory_4 = getScaledValue(rosMsg.acchory_4, 0.0004);
        rosMsg.facchorz_4 = getScaledValue(rosMsg.acchorz_4, 0.0004);
        rosMsg.facchorx_5 = getScaledValue(rosMsg.acchorx_5, 0.0004);
        rosMsg.facchory_5 = getScaledValue(rosMsg.acchory_5, 0.0004);
        rosMsg.facchorz_5 = getScaledValue(rosMsg.acchorz_5, 0.0004);
        rosMsg.facchorx_6 = getScaledValue(rosMsg.acchorx_6, 0.0004);
        rosMsg.facchory_6 = getScaledValue(rosMsg.acchory_6, 0.0004);
        rosMsg.facchorz_6 = getScaledValue(rosMsg.acchorz_6, 0.0004);
        rosMsg.facchorx_7 = getScaledValue(rosMsg.acchorx_7, 0.0004);
        rosMsg.facchory_7 = getScaledValue(rosMsg.acchory_7, 0.0004);
        rosMsg.facchorz_7 = getScaledValue(rosMsg.acchorz_7, 0.0004);

        rosMsg.fextvelanx = getScaledValue(rosMsg.extvelanx, 0.005);
        rosMsg.fextvelany = getScaledValue(rosMsg.extvelany, 0.005);
        rosMsg.fextveldigx = getScaledValue(rosMsg.extveldigx, 0.005);
        rosMsg.fextveldigy = getScaledValue(rosMsg.extveldigy, 0.005);
        rosMsg.fextvelxcorrected = getScaledValue(rosMsg.extvelxcorrected, 0.005);
        rosMsg.fextvelycorrected = getScaledValue(rosMsg.extvelycorrected, 0.005);

        rosMsg.fextbaropressure = getScaledValue(rosMsg.extbaropressure, 0.01);
        rosMsg.fextbaroheight = getScaledValue(rosMsg.extbaroheight, 0.01);
        rosMsg.fextbaroheightcorrected = getScaledValue(rosMsg.extbaroheightcorrected, 0.01);

        rosMsg.finvpathradius = getScaledValue(rosMsg.invpathradius, 0.0001);
        rosMsg.fsideslipangle = getScaledValue(rosMsg.sideslipangle, 0.01);
        rosMsg.fdisttrav = getScaledValue(rosMsg.disttrav, 0.01);

        rosMsg.finvpathradius_1 = getScaledValue(rosMsg.invpathradius_1, 0.0001);
        rosMsg.fsideslipangle_1 = getScaledValue(rosMsg.sideslipangle_1, 0.01);
        rosMsg.fdisttrav_1 = getScaledValue(rosMsg.disttrav_1, 0.01);
        rosMsg.finvpathradius_2 = getScaledValue(rosMsg.invpathradius_2, 0.0001);
        rosMsg.fsideslipangle_2 = getScaledValue(rosMsg.sideslipangle_2, 0.01);
        rosMsg.fdisttrav_2 = getScaledValue(rosMsg.disttrav_2, 0.01);
        rosMsg.finvpathradius_3 = getScaledValue(rosMsg.invpathradius_3, 0.0001);
        rosMsg.fsideslipangle_3 = getScaledValue(rosMsg.sideslipangle_3, 0.01);
        rosMsg.fdisttrav_3 = getScaledValue(rosMsg.disttrav_3, 0.01);
        rosMsg.finvpathradius_4 = getScaledValue(rosMsg.invpathradius_4, 0.0001);
        rosMsg.fsideslipangle_4 = getScaledValue(rosMsg.sideslipangle_4, 0.01);
        rosMsg.fdisttrav_4 = getScaledValue(rosMsg.disttrav_4, 0.01);
        rosMsg.finvpathradius_5 = getScaledValue(rosMsg.invpathradius_5, 0.0001);
        rosMsg.fsideslipangle_5 = getScaledValue(rosMsg.sideslipangle_5, 0.01);
        rosMsg.fdisttrav_5 = getScaledValue(rosMsg.disttrav_5, 0.01);
        rosMsg.finvpathradius_6 = getScaledValue(rosMsg.invpathradius_6, 0.0001);
        rosMsg.fsideslipangle_6 = getScaledValue(rosMsg.sideslipangle_6, 0.01);
        rosMsg.fdisttrav_6 = getScaledValue(rosMsg.disttrav_6, 0.01);
        rosMsg.finvpathradius_7 = getScaledValue(rosMsg.invpathradius_7, 0.0001);
        rosMsg.fsideslipangle_7 = getScaledValue(rosMsg.sideslipangle_7, 0.01);
        rosMsg.fdisttrav_7 = getScaledValue(rosMsg.disttrav_7, 0.01);

        rosMsg.fsystemtemp = getScaledValue(rosMsg.systemtemp, 0.1);
        rosMsg.fsystemdspload = getScaledValue(rosMsg.systemdspload, 0.1);

        rosMsg.fgpslatabs = getScaledValue(rosMsg.gpslatabs, 0.0000001);
        rosMsg.fgpslonabs = getScaledValue(rosMsg.gpslonabs, 0.0000001);
        rosMsg.fgpslatrel = getScaledValue(rosMsg.gpslatrel, 0.01);
        rosMsg.fgpslonrel = getScaledValue(rosMsg.gpslonrel, 0.01);

        rosMsg.fgpsstddevlat = getScaledValue(rosMsg.gpsstddevlat, 0.001);
        rosMsg.fgpsstddevlon = getScaledValue(rosMsg.gpsstddevlon, 0.001);
        rosMsg.fgpsstddevheight = getScaledValue(rosMsg.gpsstddevheight, 0.001);

        rosMsg.fgpsvelframex = getScaledValue(rosMsg.gpsvelframex, 0.005);
        rosMsg.fgpsvelframey = getScaledValue(rosMsg.gpsvelframey, 0.005);
        rosMsg.fgpsvelframez = getScaledValue(rosMsg.gpsvelframez, 0.005);
        rosMsg.fgpsvellatency = getScaledValue(rosMsg.gpsvellatency, 0.001);

        rosMsg.fgpsstddevvelx = getScaledValue(rosMsg.gpsstddevvelx, 0.001);
        rosMsg.fgpsstddevvely = getScaledValue(rosMsg.gpsstddevvely, 0.001);
        rosMsg.fgpsstddevvelz = getScaledValue(rosMsg.gpsstddevvelz, 0.001);

        rosMsg.fgpsdiffage = getScaledValue(rosMsg.gpsdiffage, 0.1);
        rosMsg.fgpsreceiverload = getScaledValue(rosMsg.gpsreceiverload, 0.5);

        rosMsg.finsroll = getScaledValue(rosMsg.insroll, 0.01);
        rosMsg.finspitch = getScaledValue(rosMsg.inspitch, 0.01);
        rosMsg.finsyaw = getScaledValue(rosMsg.insyaw, 0.01);
        rosMsg.fgpscog = getScaledValue(rosMsg.gpscog, 0.01);

        rosMsg.fgpsheight = getScaledValue(rosMsg.gpsheight, 0.01);
        rosMsg.fundulation = getScaledValue(rosMsg.undulation, 0.01);

        rosMsg.finsheight = getScaledValue(rosMsg.insheight, 0.01);
        rosMsg.finsheight_1 = getScaledValue(rosMsg.insheight_1, 0.01);
        rosMsg.finsheight_2 = getScaledValue(rosMsg.insheight_2, 0.01);
        rosMsg.finsheight_3 = getScaledValue(rosMsg.insheight_3, 0.01);
        rosMsg.finsheight_4 = getScaledValue(rosMsg.insheight_4, 0.01);
        rosMsg.finsheight_5 = getScaledValue(rosMsg.insheight_5, 0.01);
        rosMsg.finsheight_6 = getScaledValue(rosMsg.insheight_6, 0.01);
        rosMsg.finsheight_7 = getScaledValue(rosMsg.insheight_7, 0.01);

        rosMsg.finslatabs = getScaledValue(rosMsg.inslatabs, 0.0000001);
        rosMsg.finslonabs = getScaledValue(rosMsg.inslonabs, 0.0000001);
        rosMsg.finslatrel = getScaledValue(rosMsg.inslatrel, 0.01);
        rosMsg.finslonrel = getScaledValue(rosMsg.inslonrel, 0.01);
        rosMsg.finslatabs_1 = getScaledValue(rosMsg.inslatabs_1, 0.0000001);
        rosMsg.finslonabs_1 = getScaledValue(rosMsg.inslonabs_1, 0.0000001);
        rosMsg.finslatrel_1 = getScaledValue(rosMsg.inslatrel_1, 0.01);
        rosMsg.finslonrel_1 = getScaledValue(rosMsg.inslonrel_1, 0.01);
        rosMsg.finslatabs_2 = getScaledValue(rosMsg.inslatabs_2, 0.0000001);
        rosMsg.finslonabs_2 = getScaledValue(rosMsg.inslonabs_2, 0.0000001);
        rosMsg.finslatrel_2 = getScaledValue(rosMsg.inslatrel_2, 0.01);
        rosMsg.finslonrel_2 = getScaledValue(rosMsg.inslonrel_2, 0.01);
        rosMsg.finslatabs_3 = getScaledValue(rosMsg.inslatabs_3, 0.0000001);
        rosMsg.finslonabs_3 = getScaledValue(rosMsg.inslonabs_3, 0.0000001);
        rosMsg.finslatrel_3 = getScaledValue(rosMsg.inslatrel_3, 0.01);
        rosMsg.finslonrel_3 = getScaledValue(rosMsg.inslonrel_3, 0.01);
        rosMsg.finslatabs_4 = getScaledValue(rosMsg.inslatabs_4, 0.0000001);
        rosMsg.finslonabs_4 = getScaledValue(rosMsg.inslonabs_4, 0.0000001);
        rosMsg.finslatrel_4 = getScaledValue(rosMsg.inslatrel_4, 0.01);
        rosMsg.finslonrel_4 = getScaledValue(rosMsg.inslonrel_4, 0.01);
        rosMsg.finslatabs_5 = getScaledValue(rosMsg.inslatabs_5, 0.0000001);
        rosMsg.finslonabs_5 = getScaledValue(rosMsg.inslonabs_5, 0.0000001);
        rosMsg.finslatrel_5 = getScaledValue(rosMsg.inslatrel_5, 0.01);
        rosMsg.finslonrel_5 = getScaledValue(rosMsg.inslonrel_5, 0.01);
        rosMsg.finslatabs_6 = getScaledValue(rosMsg.inslatabs_6, 0.0000001);
        rosMsg.finslonabs_6 = getScaledValue(rosMsg.inslonabs_6, 0.0000001);
        rosMsg.finslatrel_6 = getScaledValue(rosMsg.inslatrel_6, 0.01);
        rosMsg.finslonrel_6 = getScaledValue(rosMsg.inslonrel_6, 0.01);
        rosMsg.finslatabs_7 = getScaledValue(rosMsg.inslatabs_7, 0.0000001);
        rosMsg.finslonabs_7 = getScaledValue(rosMsg.inslonabs_7, 0.0000001);
        rosMsg.finslatrel_7 = getScaledValue(rosMsg.inslatrel_7, 0.01);
        rosMsg.finslonrel_7 = getScaledValue(rosMsg.inslonrel_7, 0.01);

        rosMsg.finsvelhorx = getScaledValue(rosMsg.insvelhorx, 0.005);
        rosMsg.finsvelhory = getScaledValue(rosMsg.insvelhory, 0.005);
        rosMsg.finsvelhorz = getScaledValue(rosMsg.insvelhorz, 0.005);
        rosMsg.finsvelframex = getScaledValue(rosMsg.insvelframex, 0.005);
        rosMsg.finsvelframey = getScaledValue(rosMsg.insvelframey, 0.005);
        rosMsg.finsvelframez = getScaledValue(rosMsg.insvelframez, 0.005);

        rosMsg.finsvelhorx_1 = getScaledValue(rosMsg.insvelhorx_1, 0.005);
        rosMsg.finsvelhory_1 = getScaledValue(rosMsg.insvelhory_1, 0.005);
        rosMsg.finsvelhorz_1 = getScaledValue(rosMsg.insvelhorz_1, 0.005);
        rosMsg.finsvelhorx_2 = getScaledValue(rosMsg.insvelhorx_2, 0.005);
        rosMsg.finsvelhory_2 = getScaledValue(rosMsg.insvelhory_2, 0.005);
        rosMsg.finsvelhorz_2 = getScaledValue(rosMsg.insvelhorz_2, 0.005);
        rosMsg.finsvelhorx_3 = getScaledValue(rosMsg.insvelhorx_3, 0.005);
        rosMsg.finsvelhory_3 = getScaledValue(rosMsg.insvelhory_3, 0.005);
        rosMsg.finsvelhorz_3 = getScaledValue(rosMsg.insvelhorz_3, 0.005);
        rosMsg.finsvelhorx_4 = getScaledValue(rosMsg.insvelhorx_4, 0.005);
        rosMsg.finsvelhory_4 = getScaledValue(rosMsg.insvelhory_4, 0.005);
        rosMsg.finsvelhorz_4 = getScaledValue(rosMsg.insvelhorz_4, 0.005);
        rosMsg.finsvelhorx_5 = getScaledValue(rosMsg.insvelhorx_5, 0.005);
        rosMsg.finsvelhory_5 = getScaledValue(rosMsg.insvelhory_5, 0.005);
        rosMsg.finsvelhorz_5 = getScaledValue(rosMsg.insvelhorz_5, 0.005);
        rosMsg.finsvelhorx_6 = getScaledValue(rosMsg.insvelhorx_6, 0.005);
        rosMsg.finsvelhory_6 = getScaledValue(rosMsg.insvelhory_6, 0.005);
        rosMsg.finsvelhorz_6 = getScaledValue(rosMsg.insvelhorz_6, 0.005);
        rosMsg.finsvelhorx_7 = getScaledValue(rosMsg.insvelhorx_7, 0.005);
        rosMsg.finsvelhory_7 = getScaledValue(rosMsg.insvelhory_7, 0.005);
        rosMsg.finsvelhorz_7 = getScaledValue(rosMsg.insvelhorz_7, 0.005);

        rosMsg.finsstddevlat = getScaledValue(rosMsg.insstddevlat, 0.01);
        rosMsg.finsstddevlong = getScaledValue(rosMsg.insstddevlong, 0.01);
        rosMsg.finsstddevheight = getScaledValue(rosMsg.insstddevheight, 0.01);

        rosMsg.finsstddevvelx = getScaledValue(rosMsg.insstddevvelx, 0.01);
        rosMsg.finsstddevvely = getScaledValue(rosMsg.insstddevvely, 0.01);
        rosMsg.finsstddevvelz = getScaledValue(rosMsg.insstddevvelz, 0.01);
        rosMsg.finsstddevroll = getScaledValue(rosMsg.insstddevroll, 0.01);
        rosMsg.finsstddevpitch = getScaledValue(rosMsg.insstddevpitch, 0.01);
        rosMsg.finsstddevyaw = getScaledValue(rosMsg.insstddevyaw, 0.01);

        rosMsg.fan1 = getScaledValue(rosMsg.an1, 0.0005);
        rosMsg.fan2 = getScaledValue(rosMsg.an2, 0.0005);
        rosMsg.fan3 = getScaledValue(rosMsg.an3, 0.0005);
        rosMsg.fan4 = getScaledValue(rosMsg.an4, 0.0005);

        // only for >= v3.3.3
        rosMsg.fgpsdualantheading = getScaledValue(rosMsg.gpsdualantheading, 0.01);
        rosMsg.fgpsdualantpitch = getScaledValue(rosMsg.gpsdualantpitch, 0.01);
        rosMsg.fgpsdualantstddevheading = getScaledValue(rosMsg.gpsdualantstddevheading, 0.01);
        rosMsg.fgpsdualantstddevpitch = getScaledValue(rosMsg.gpsdualantstddevpitch, 0.01);
        rosMsg.fgpsdualantstddevheading_hr = getScaledValue(rosMsg.gpsdualantstddevheading_hr, 0.01);
        rosMsg.fgpsdualantstddevpitch_hr = getScaledValue(rosMsg.gpsdualantstddevpitch_hr, 0.01);
        rosMsg.faccbodyx_8 = getScaledValue(rosMsg.accbodyx_8, 0.0004);
        rosMsg.faccbodyy_8 = getScaledValue(rosMsg.accbodyy_8, 0.0004);
        rosMsg.faccbodyz_8 = getScaledValue(rosMsg.accbodyz_8, 0.0004);
        rosMsg.facchorx_8 = getScaledValue(rosMsg.acchorx_8, 0.0004);
        rosMsg.facchory_8 = getScaledValue(rosMsg.acchory_8, 0.0004);
        rosMsg.facchorz_8 = getScaledValue(rosMsg.acchorz_8, 0.0004);
        rosMsg.finvpathradius_8 = getScaledValue(rosMsg.invpathradius_8, 0.0001);
        rosMsg.fsideslipangle_8 = getScaledValue(rosMsg.sideslipangle_8, 0.01);
        rosMsg.fdisttrav_8 = getScaledValue(rosMsg.disttrav_8, 0.01);
        rosMsg.finsheight_8 = getScaledValue(rosMsg.insheight_8, 0.01);
        rosMsg.finslatabs_8 = getScaledValue(rosMsg.inslatabs_8, 0.0000001);
        rosMsg.finslonabs_8 = getScaledValue(rosMsg.inslonabs_8, 0.0000001);
        rosMsg.finslatrel_8 = getScaledValue(rosMsg.inslatrel_8, 0.01);
        rosMsg.finslonrel_8 = getScaledValue(rosMsg.inslonrel_8, 0.01);
        rosMsg.finsvelhorx_8 = getScaledValue(rosMsg.insvelhorx_8, 0.005);
        rosMsg.finsvelhory_8 = getScaledValue(rosMsg.insvelhory_8, 0.005);
        rosMsg.finsvelhorz_8 = getScaledValue(rosMsg.insvelhorz_8, 0.005);
}

void ADMA2ROSParser::extractNavSatFix(adma_msgs::msg::AdmaData& rosMsg, sensor_msgs::msg::NavSatFix& navRosMsg)    
{
        // fil status
        switch (rosMsg.statusgpsmode)
        {
        case 1:
                // No GNSS Data
                navRosMsg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX; 
                break;
        case 2:
                // single GNSS
                navRosMsg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX; 
                break;
        case 4:
                // actually DGNSS Coarse Mode, but used to distinguish here 
                navRosMsg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_SBAS_FIX;
                break;
        case 8:
                // DGNSS Precise Mode
                navRosMsg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX;
                break;
        default:
                break;
        }

        navRosMsg.altitude = rosMsg.finsheight;
        navRosMsg.latitude = rosMsg.finslatabs;
        navRosMsg.longitude = rosMsg.finslonabs;
        navRosMsg.position_covariance[0] = std::pow(rosMsg.finsstddevlat, 2);
        navRosMsg.position_covariance[4] = std::pow(rosMsg.finsstddevlong, 2);
        navRosMsg.position_covariance[8] = std::pow(rosMsg.finsstddevheight, 2);

        navRosMsg.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
}

void ADMA2ROSParser::extractIMU(adma_msgs::msg::AdmaData& rosMsg, sensor_msgs::msg::Imu& imuRosMsg)
{
         imuRosMsg.linear_acceleration.x = rosMsg.faccbodyhrx * 9.81;
        imuRosMsg.linear_acceleration.y = rosMsg.faccbodyhry * 9.81;
        imuRosMsg.linear_acceleration.z = rosMsg.faccbodyhrz * 9.81;

        imuRosMsg.angular_velocity.x = rosMsg.fratebodyhrx * PI / 180.0;
        imuRosMsg.angular_velocity.y = rosMsg.fratebodyhry * PI / 180.0;
        imuRosMsg.angular_velocity.z = rosMsg.fratebodyhrz * PI / 180.0;

        tf2::Quaternion q;
        double roll_rad = rosMsg.finsroll * PI / 180.0;
        double pitch_rad = rosMsg.finspitch * PI / 180.0;
        double yaw_rad = rosMsg.finsyaw * PI / 180.0;
        q.setRPY(roll_rad, pitch_rad, yaw_rad);
        imuRosMsg.orientation = tf2::toMsg(q);

        imuRosMsg.orientation_covariance[0] = std::pow(rosMsg.finsstddevroll * PI / 180.0, 2);
        imuRosMsg.orientation_covariance[4] = std::pow(rosMsg.finsstddevpitch * PI / 180.0, 2);
        imuRosMsg.orientation_covariance[8] = std::pow(rosMsg.finsstddevyaw * PI / 180.0, 2);

        // ADMA does not provide covariance for linear acceleration and angular velocity.
        // These values need to be measured at standstill each ADMA model.
        imuRosMsg.angular_velocity_covariance[0] = -1;
        imuRosMsg.linear_acceleration_covariance[0] = -1;
}

