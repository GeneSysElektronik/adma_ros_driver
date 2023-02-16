#include <iostream>
#include "../../include/adma_ros_driver/parser/adma2ros_parser.hpp"
#include "../../include/adma_ros_driver/parser/parser_utils.hpp"
// #include <tf2_geometry_msgs/tf2_geometry_msgs.h>

ADMA2ROSParser::ADMA2ROSParser()
        :_parserV333(),
        _parserV334()
{
}

void ADMA2ROSParser::parseV333(adma_ros_driver_msgs::Adma& rosMsg, std::array<char, 856>& recvData)
{
        AdmaDataV333 admaData;
        memcpy(&admaData, &recvData, sizeof(admaData));
        parseStaticHeader(rosMsg, admaData.staticHeader);
        parseDynamicHeader(rosMsg, admaData.dynamicHeader);
        getstatusgps(rosMsg, admaData.gnssStatus);
        getstatustrigger(rosMsg, admaData.signalInStatus);
        getevkstatus(rosMsg, admaData.miscStatus);
        _parserV333.getKFStatus(rosMsg, admaData.kfStatus);
        unsigned char ewBytes[] = {admaData.dataError1, admaData.dataError2, admaData.dataWarn1, admaData.dataError3};
        geterrorandwarning(rosMsg, ewBytes);
        _parserV333.mapAdmaMessageToROS(rosMsg, admaData);

        parseScaledData(rosMsg);
}

void ADMA2ROSParser::parseV334(adma_ros_driver_msgs::AdmaDataScaled& rosMsg, AdmaDataV334& recvData)
{
        _parserV334.mapAdmaMessageToROS(rosMsg, recvData);
}

void ADMA2ROSParser::parseV334Status(adma_ros_driver_msgs::AdmaStatus& rosMsg, AdmaDataV334& localData)
{
        _parserV334.mapStatusToROS(rosMsg, localData);
}

template <typename AdmaDataHeaderStruct>
void ADMA2ROSParser::parseStaticHeader(adma_ros_driver_msgs::Adma& rosMsg, AdmaDataHeaderStruct& staticHeader)
{
        // fill static header information
        rosMsg.GeneSysID = staticHeader.genesysid;
        std::stringstream ss;
        ss <<  int(staticHeader.headerversion[0]) << int(staticHeader.headerversion[1]) << int(staticHeader.headerversion[2]) << int(staticHeader.headerversion[3]);
        rosMsg.HeaderVersion = ss.str();
        ss.clear();
        ss.str("");
        rosMsg.FormatID = staticHeader.formatid;
        //TODO: this value is parsed wrong?!        
        ss <<  int(staticHeader.formatversion[0]) << int(staticHeader.formatversion[1]) << int(staticHeader.formatversion[2]) << int(staticHeader.formatversion[3]);
        rosMsg.FormatVersion = ss.str();
        rosMsg.SerialNo = staticHeader.serialno;
}

template <typename AdmaDataHeaderStruct>
void ADMA2ROSParser::parseDynamicHeader(adma_ros_driver_msgs::Adma& rosMsg, AdmaDataHeaderStruct& dynamicHeader)
{
        // fill dynamic header information
        rosMsg.ConfigID = dynamicHeader.configid;
        rosMsg.ConfigFormat = dynamicHeader.configformat;
        rosMsg.ConfigVersion = dynamicHeader.configversion;
        rosMsg.ConfigSize = dynamicHeader.configsize;
        rosMsg.ByteOffset = dynamicHeader.byteoffset;
        rosMsg.SliceSize = dynamicHeader.slicesize;
        rosMsg.SliceData = dynamicHeader.slicedata;
}


/// \file
/// \brief  getstatusgps function - adma status information
/// \param  rosMsg ros message to fill with content
/// \param  gpsStatus byte with gps states
void ADMA2ROSParser::getstatusgps(adma_ros_driver_msgs::Adma& rosMsg, unsigned char gpsStatus)
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
        rosMsg.StatusGPSMode = 1;
        }
        else if (gps_mode) 
        {
        rosMsg.StatusGPSMode = 2;
        }
        else if (rtk_coarse) 
        {
        rosMsg.StatusGPSMode = 4;
        }
        else if (rtk_precise) 
        {
        rosMsg.StatusGPSMode = 8;
        }
        /* status stand still */
        rosMsg.StatusStandStill = standstill_c;
        /* status skidding */
        rosMsg.StatusSkidding = status_skidding;
        /* status external velocity slip */
        rosMsg.StatusExternalVelOut = status_external_vel;
}

/// \file
/// \brief  getstatustrigger function - adma gps trigger information
/// \param  rosMsg ros message to fill with content
/// \param  gpsTriggerStatus byte with gps trigger states
void ADMA2ROSParser::getstatustrigger(adma_ros_driver_msgs::Adma& rosMsg, unsigned char gpsTriggerStatus)
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
        rosMsg.StatusTrigGps = status_trig_gps;
        /* status statussignalin3 */
        rosMsg.StatusSignalIN3 = status_signal_in3;
        /* status statussignalin2 */
        rosMsg.StatusSignalIn2 = status_signal_in2;
        /* status statussignalin1 */
        rosMsg.StatusSignalIn1 = status_signal_in1;
        /* status statusalignment */
        rosMsg.StatusAlignment = status_alignment;
        /* status statusahrsins */
        rosMsg.StatusAHRSINS = status_ahrs_ins;
        /* status statusdeadreckoning */
        rosMsg.StatusDeadReckoning = status_dead_reckoning;
        /* status statussynclock */
        rosMsg.StatusSyncLock = status_synclock;
}

/// \file
/// \brief  getstatustrigger function - adma gps trigger information
/// \param  rosMsg ros message to fill with content
/// \param  evkStatus byte with evk states
void ADMA2ROSParser::getevkstatus(adma_ros_driver_msgs::Adma& rosMsg, unsigned char evkStatus)
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
        rosMsg.StatusEVKActiv = status_evk_activ;
        /* status status_evk_estimates */
        rosMsg.StatusEVKEstimates = status_evk_estimates;
        /* status status_heading_executed */
        rosMsg.StatusHeadingExecuted = status_heading_executed;
        /* status status_configuration_changed */
        rosMsg.StatusConfigurationChanged = status_configuration_changed;
        /* status tilt */
        if(status_tilt_b1==0 && status_tilt_b2==0)
        {
        rosMsg.StatusTilt = 0;
        }
        else if(status_tilt_b1==0 && status_tilt_b2==1)
        {
        rosMsg.StatusTilt = 1;
        }
        else if(status_tilt_b1==1 && status_tilt_b2==0)
        {
        rosMsg.StatusTilt = 2;
        }
        /* status pos */
        if(status_pos_b1==0 && status_pos_b2==0)
        {
        rosMsg.StatusPos = 0;
        }
        else if(status_pos_b1==0 && status_pos_b2==1)
        {
        rosMsg.StatusPos = 1;
        }
        else if(status_pos_b1==1 && status_pos_b2==0)
        {
        rosMsg.StatusPos = 2;
        }
}

/// \file
/// \brief  geterrorandwarning function - adma error and warning
/// \param  rosMsg ros message to fill with content
/// \param  ewBytes array of bytes with several error and warnings
void ADMA2ROSParser::geterrorandwarning(adma_ros_driver_msgs::Adma& rosMsg, unsigned char ewBytes[4])
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
        rosMsg.ErrorHardware = erhw1.to_string();
        rosMsg.Error_Misc1 = ermisc1.to_string();
        rosMsg.Error_Misc2 = ermisc2.to_string();
        rosMsg.Error_Misc3 = ermisc3.to_string();
        rosMsg.WarnGPS = warngps.to_string();
        rosMsg.WarnMisc1 = warnmisc1.to_string();
        rosMsg.ErrorHWSticky = erhwsticky.to_string();
}

/// \file
/// \brief  pareScaledData function - fills scaled values with LSB factor
/// \param  rosMsg ros message to fill with content
void ADMA2ROSParser::parseScaledData(adma_ros_driver_msgs::Adma& rosMsg)
{
        rosMsg.fAccBodyHRX = getScaledValue(rosMsg.AccBodyHRX, 0.0001);
        rosMsg.fRateBodyHRX = getScaledValue(rosMsg.RateBodyHRX, 0.0001);
        rosMsg.fAccBodyHRY = getScaledValue(rosMsg.AccBodyHRY, 0.0001);
        rosMsg.fRateBodyHRY = getScaledValue(rosMsg.RateBodyHRY, 0.0001);
        rosMsg.fAccBodyHRZ = getScaledValue(rosMsg.AccBodyHRZ, 0.0001);
        rosMsg.fRateBodyHRZ = getScaledValue(rosMsg.RateBodyHRZ, 0.0001);

        rosMsg.fRateBodyX = getScaledValue(rosMsg.RateBodyX, 0.01);
        rosMsg.fRateBodyY = getScaledValue(rosMsg.RateBodyY, 0.01);
        rosMsg.fRateBodyZ = getScaledValue(rosMsg.RateBodyZ, 0.01);
        rosMsg.fRateHorX = getScaledValue(rosMsg.RateHorX, 0.01);
        rosMsg.fRateHorY = getScaledValue(rosMsg.RateHorY, 0.01);
        rosMsg.fRateHorZ = getScaledValue(rosMsg.RateHorZ, 0.01);

        rosMsg.fAccBodyX = getScaledValue(rosMsg.AccBodyX, 0.0004);
        rosMsg.fAccBodyY = getScaledValue(rosMsg.AccBodyY, 0.0004);
        rosMsg.fAccBodyZ = getScaledValue(rosMsg.AccBodyZ, 0.0004);
        rosMsg.fAccHorX = getScaledValue(rosMsg.AccHorX, 0.0004);
        rosMsg.fAccHorY = getScaledValue(rosMsg.AccHorY, 0.0004);
        rosMsg.fAccHorZ = getScaledValue(rosMsg.AccHorZ, 0.0004);

        rosMsg.fAccBodyX_1 = getScaledValue(rosMsg.AccBodyX_1, 0.0004);
        rosMsg.fAccBodyY_1 = getScaledValue(rosMsg.AccBodyY_1, 0.0004);
        rosMsg.fAccBodyZ_1 = getScaledValue(rosMsg.AccBodyZ_1, 0.0004);
        rosMsg.fAccBodyX_2 = getScaledValue(rosMsg.AccBodyX_2, 0.0004);
        rosMsg.fAccBodyY_2 = getScaledValue(rosMsg.AccBodyY_2, 0.0004);
        rosMsg.fAccBodyZ_2 = getScaledValue(rosMsg.AccBodyZ_2, 0.0004);
        rosMsg.fAccBodyX_3 = getScaledValue(rosMsg.AccBodyX_3, 0.0004);
        rosMsg.fAccBodyY_3 = getScaledValue(rosMsg.AccBodyY_3, 0.0004);
        rosMsg.fAccBodyZ_3 = getScaledValue(rosMsg.AccBodyZ_3, 0.0004);
        rosMsg.fAccBodyX_4 = getScaledValue(rosMsg.AccBodyX_4, 0.0004);
        rosMsg.fAccBodyY_4 = getScaledValue(rosMsg.AccBodyY_4, 0.0004);
        rosMsg.fAccBodyZ_4 = getScaledValue(rosMsg.AccBodyZ_4, 0.0004);
        rosMsg.fAccBodyX_5 = getScaledValue(rosMsg.AccBodyX_5, 0.0004);
        rosMsg.fAccBodyY_5 = getScaledValue(rosMsg.AccBodyY_5, 0.0004);
        rosMsg.fAccBodyZ_5 = getScaledValue(rosMsg.AccBodyZ_5, 0.0004);
        rosMsg.fAccBodyX_6 = getScaledValue(rosMsg.AccBodyX_6, 0.0004);
        rosMsg.fAccBodyY_6 = getScaledValue(rosMsg.AccBodyY_6, 0.0004);
        rosMsg.fAccBodyZ_6 = getScaledValue(rosMsg.AccBodyZ_6, 0.0004);
        rosMsg.fAccBodyX_7 = getScaledValue(rosMsg.AccBodyX_7, 0.0004);
        rosMsg.fAccBodyY_7 = getScaledValue(rosMsg.AccBodyY_7, 0.0004);
        rosMsg.fAccBodyZ_7 = getScaledValue(rosMsg.AccBodyZ_7, 0.0004);
        rosMsg.fAccBodyX_8 = getScaledValue(rosMsg.AccBodyX_8, 0.0004);
        rosMsg.fAccBodyY_8 = getScaledValue(rosMsg.AccBodyY_8, 0.0004);
        rosMsg.fAccBodyZ_8 = getScaledValue(rosMsg.AccBodyZ_8, 0.0004);

        rosMsg.fAccHorX_1 = getScaledValue(rosMsg.AccHorX_1, 0.0004);
        rosMsg.fAccHorY_1 = getScaledValue(rosMsg.AccHorY_1, 0.0004);
        rosMsg.fAccHorZ_1 = getScaledValue(rosMsg.AccHorZ_1, 0.0004);
        rosMsg.fAccHorX_2 = getScaledValue(rosMsg.AccHorX_2, 0.0004);
        rosMsg.fAccHorY_2 = getScaledValue(rosMsg.AccHorY_2, 0.0004);
        rosMsg.fAccHorZ_2 = getScaledValue(rosMsg.AccHorZ_2, 0.0004);
        rosMsg.fAccHorX_3 = getScaledValue(rosMsg.AccHorX_3, 0.0004);
        rosMsg.fAccHorY_3 = getScaledValue(rosMsg.AccHorY_3, 0.0004);
        rosMsg.fAccHorZ_3 = getScaledValue(rosMsg.AccHorZ_3, 0.0004);
        rosMsg.fAccHorX_4 = getScaledValue(rosMsg.AccHorX_4, 0.0004);
        rosMsg.fAccHorY_4 = getScaledValue(rosMsg.AccHorY_4, 0.0004);
        rosMsg.fAccHorZ_4 = getScaledValue(rosMsg.AccHorZ_4, 0.0004);
        rosMsg.fAccHorX_5 = getScaledValue(rosMsg.AccHorX_5, 0.0004);
        rosMsg.fAccHorY_5 = getScaledValue(rosMsg.AccHorY_5, 0.0004);
        rosMsg.fAccHorZ_5 = getScaledValue(rosMsg.AccHorZ_5, 0.0004);
        rosMsg.fAccHorX_6 = getScaledValue(rosMsg.AccHorX_6, 0.0004);
        rosMsg.fAccHorY_6 = getScaledValue(rosMsg.AccHorY_6, 0.0004);
        rosMsg.fAccHorZ_6 = getScaledValue(rosMsg.AccHorZ_6, 0.0004);
        rosMsg.fAccHorX_7 = getScaledValue(rosMsg.AccHorX_7, 0.0004);
        rosMsg.fAccHorY_7 = getScaledValue(rosMsg.AccHorY_7, 0.0004);
        rosMsg.fAccHorZ_7 = getScaledValue(rosMsg.AccHorZ_7, 0.0004);
        rosMsg.fAccHorX_8 = getScaledValue(rosMsg.AccHorX_8, 0.0004);
        rosMsg.fAccHorY_8 = getScaledValue(rosMsg.AccHorY_8, 0.0004);
        rosMsg.fAccHorZ_8 = getScaledValue(rosMsg.AccHorZ_8, 0.0004);

        rosMsg.fExtVelAnX = getScaledValue(rosMsg.ExtVelAnX, 0.005);
        rosMsg.fExtVelAnY = getScaledValue(rosMsg.ExtVelAnY, 0.005);
        rosMsg.fExtVelDigX = getScaledValue(rosMsg.ExtVelDigX, 0.005);
        rosMsg.fExtVelDigY = getScaledValue(rosMsg.ExtVelDigY, 0.005);
        rosMsg.fExtVelXCorrected = getScaledValue(rosMsg.ExtVelXCorrected, 0.005);
        rosMsg.fExtVelYCorrected = getScaledValue(rosMsg.ExtVelYCorrected, 0.005);

        rosMsg.fExtBaroPressure = getScaledValue(rosMsg.ExtBaroPressure, 0.01);
        rosMsg.fExtBaroHeight = getScaledValue(rosMsg.ExtBaroHeight, 0.01);
        rosMsg.fExtBaroHeightCorrected = getScaledValue(rosMsg.ExtBaroHeightCorrected, 0.01);

        rosMsg.fInvPathRadius = getScaledValue(rosMsg.InvPathRadius, 0.0001);
        rosMsg.fSideSlipAngle = getScaledValue(rosMsg.SideSlipAngle, 0.01);
        rosMsg.fDistTrav = getScaledValue(rosMsg.DistTrav, 0.01);

        rosMsg.fInvPathRadius_1 = getScaledValue(rosMsg.InvPathRadius_1, 0.0001);
        rosMsg.fSideSlipAngle_1 = getScaledValue(rosMsg.SideSlipAngle_1, 0.01);
        rosMsg.fDistTrav_1 = getScaledValue(rosMsg.DistTrav_1, 0.01);
        rosMsg.fInvPathRadius_2 = getScaledValue(rosMsg.InvPathRadius_2, 0.0001);
        rosMsg.fSideSlipAngle_2 = getScaledValue(rosMsg.SideSlipAngle_2, 0.01);
        rosMsg.fDistTrav_2 = getScaledValue(rosMsg.DistTrav_2, 0.01);
        rosMsg.fInvPathRadius_3 = getScaledValue(rosMsg.InvPathRadius_3, 0.0001);
        rosMsg.fSideSlipAngle_3 = getScaledValue(rosMsg.SideSlipAngle_3, 0.01);
        rosMsg.fDistTrav_3 = getScaledValue(rosMsg.DistTrav_3, 0.01);
        rosMsg.fInvPathRadius_4 = getScaledValue(rosMsg.InvPathRadius_4, 0.0001);
        rosMsg.fSideSlipAngle_4 = getScaledValue(rosMsg.SideSlipAngle_4, 0.01);
        rosMsg.fDistTrav_4 = getScaledValue(rosMsg.DistTrav_4, 0.01);
        rosMsg.fInvPathRadius_5 = getScaledValue(rosMsg.InvPathRadius_5, 0.0001);
        rosMsg.fSideSlipAngle_5 = getScaledValue(rosMsg.SideSlipAngle_5, 0.01);
        rosMsg.fDistTrav_5 = getScaledValue(rosMsg.DistTrav_5, 0.01);
        rosMsg.fInvPathRadius_6 = getScaledValue(rosMsg.InvPathRadius_6, 0.0001);
        rosMsg.fSideSlipAngle_6 = getScaledValue(rosMsg.SideSlipAngle_6, 0.01);
        rosMsg.fDistTrav_6 = getScaledValue(rosMsg.DistTrav_6, 0.01);
        rosMsg.fInvPathRadius_7 = getScaledValue(rosMsg.InvPathRadius_7, 0.0001);
        rosMsg.fSideSlipAngle_7 = getScaledValue(rosMsg.SideSlipAngle_7, 0.01);
        rosMsg.fDistTrav_7 = getScaledValue(rosMsg.DistTrav_7, 0.01);
        rosMsg.fInvPathRadius_8 = getScaledValue(rosMsg.InvPathRadius_8, 0.0001);
        rosMsg.fSideSlipAngle_8 = getScaledValue(rosMsg.SideSlipAngle_8, 0.01);
        rosMsg.fDistTrav_8 = getScaledValue(rosMsg.DistTrav_8, 0.01);

        rosMsg.fSystemTemp = getScaledValue(rosMsg.SystemTemp, 0.1);
        rosMsg.fSystemDSPLoad = getScaledValue(rosMsg.SystemDSPLoad, 0.1);

        rosMsg.fGPSLatAbs = getScaledValue(rosMsg.GPSLatAbs, 0.0000001);
        rosMsg.fGPSLonAbs = getScaledValue(rosMsg.GPSLonAbs, 0.0000001);
        rosMsg.fGPSLatRel = getScaledValue(rosMsg.GPSLatRel, 0.01);
        rosMsg.fGPSLonRel = getScaledValue(rosMsg.GPSLonRel, 0.01);

        rosMsg.fGPSStddevLat = getScaledValue(rosMsg.GPSStddevLat, 0.001);
        rosMsg.fGPSStddevLon = getScaledValue(rosMsg.GPSStddevLon, 0.001);
        rosMsg.fGPSStddevHeight = getScaledValue(rosMsg.GPSStddevHeight, 0.001);

        rosMsg.fGPSVelFrameX = getScaledValue(rosMsg.GPSVelFrameX, 0.005);
        rosMsg.fGPSVelFrameY = getScaledValue(rosMsg.GPSVelFrameY, 0.005);
        rosMsg.fGPSVelFrameZ = getScaledValue(rosMsg.GPSVelFrameZ, 0.005);
        rosMsg.fGPSVelLatency = getScaledValue(rosMsg.GPSVelLatency, 0.001);

        rosMsg.fGPSStddevVelX = getScaledValue(rosMsg.GPSStddevVelX, 0.001);
        rosMsg.fGPSStddevVelY = getScaledValue(rosMsg.GPSStddevVelY, 0.001);
        rosMsg.fGPSStddevVelZ = getScaledValue(rosMsg.GPSStddevVelZ, 0.001);

        rosMsg.fGPSDiffAge = getScaledValue(rosMsg.GPSDiffAge, 0.1);
        rosMsg.fGPSReceiverLoad = getScaledValue(rosMsg.GPSReceiverLoad, 0.5);

        rosMsg.fINSRoll = getScaledValue(rosMsg.INSRoll, 0.01);
        rosMsg.fINSPitch = getScaledValue(rosMsg.INSPitch, 0.01);
        rosMsg.fINSYaw = getScaledValue(rosMsg.INSYaw, 0.01);
        rosMsg.fGPSCOG = getScaledValue(rosMsg.GPSCOG, 0.01);

        rosMsg.fGPSHeight = getScaledValue(rosMsg.GPSHeight, 0.01);
        rosMsg.fUndulation = getScaledValue(rosMsg.Undulation, 0.01);

        rosMsg.fINSHeight = getScaledValue(rosMsg.INSHeight, 0.01);
        rosMsg.fINSHeight_1 = getScaledValue(rosMsg.INSHeight_1, 0.01);
        rosMsg.fINSHeight_2 = getScaledValue(rosMsg.INSHeight_2, 0.01);
        rosMsg.fINSHeight_3 = getScaledValue(rosMsg.INSHeight_3, 0.01);
        rosMsg.fINSHeight_4 = getScaledValue(rosMsg.INSHeight_4, 0.01);
        rosMsg.fINSHeight_5 = getScaledValue(rosMsg.INSHeight_5, 0.01);
        rosMsg.fINSHeight_6 = getScaledValue(rosMsg.INSHeight_6, 0.01);
        rosMsg.fINSHeight_7 = getScaledValue(rosMsg.INSHeight_7, 0.01);
        rosMsg.fINSHeight_8 = getScaledValue(rosMsg.INSHeight_8, 0.01);

        rosMsg.fINSLatAbs = getScaledValue(rosMsg.INSLatAbs, 0.0000001);
        rosMsg.fINSLonAbs = getScaledValue(rosMsg.INSLonAbs, 0.0000001);
        rosMsg.fINSLatRel = getScaledValue(rosMsg.INSLatRel, 0.01);
        rosMsg.fINSLonRel = getScaledValue(rosMsg.INSLonRel, 0.01);
        rosMsg.fINSLatAbs_1 = getScaledValue(rosMsg.INSLatAbs_1, 0.0000001);
        rosMsg.fINSLonAbs_1 = getScaledValue(rosMsg.INSLonAbs_1, 0.0000001);
        rosMsg.fINSLatRel_1 = getScaledValue(rosMsg.INSLatRel_1, 0.01);
        rosMsg.fINSLonRel_1 = getScaledValue(rosMsg.INSLonRel_1, 0.01);
        rosMsg.fINSLatAbs_2 = getScaledValue(rosMsg.INSLatAbs_2, 0.0000001);
        rosMsg.fINSLonAbs_2 = getScaledValue(rosMsg.INSLonAbs_2, 0.0000001);
        rosMsg.fINSLatRel_2 = getScaledValue(rosMsg.INSLatRel_2, 0.01);
        rosMsg.fINSLonRel_2 = getScaledValue(rosMsg.INSLonRel_2, 0.01);
        rosMsg.fINSLatAbs_3 = getScaledValue(rosMsg.INSLatAbs_3, 0.0000001);
        rosMsg.fINSLonAbs_3 = getScaledValue(rosMsg.INSLonAbs_3, 0.0000001);
        rosMsg.fINSLatRel_3 = getScaledValue(rosMsg.INSLatRel_3, 0.01);
        rosMsg.fINSLonRel_3 = getScaledValue(rosMsg.INSLonRel_3, 0.01);
        rosMsg.fINSLatAbs_4 = getScaledValue(rosMsg.INSLatAbs_4, 0.0000001);
        rosMsg.fINSLonAbs_4 = getScaledValue(rosMsg.INSLonAbs_4, 0.0000001);
        rosMsg.fINSLatRel_4 = getScaledValue(rosMsg.INSLatRel_4, 0.01);
        rosMsg.fINSLonRel_4 = getScaledValue(rosMsg.INSLonRel_4, 0.01);
        rosMsg.fINSLatAbs_5 = getScaledValue(rosMsg.INSLatAbs_5, 0.0000001);
        rosMsg.fINSLonAbs_5 = getScaledValue(rosMsg.INSLonAbs_5, 0.0000001);
        rosMsg.fINSLatRel_5 = getScaledValue(rosMsg.INSLatRel_5, 0.01);
        rosMsg.fINSLonRel_5 = getScaledValue(rosMsg.INSLonRel_5, 0.01);
        rosMsg.fINSLatAbs_6 = getScaledValue(rosMsg.INSLatAbs_6, 0.0000001);
        rosMsg.fINSLonAbs_6 = getScaledValue(rosMsg.INSLonAbs_6, 0.0000001);
        rosMsg.fINSLatRel_6 = getScaledValue(rosMsg.INSLatRel_6, 0.01);
        rosMsg.fINSLonRel_6 = getScaledValue(rosMsg.INSLonRel_6, 0.01);
        rosMsg.fINSLatAbs_7 = getScaledValue(rosMsg.INSLatAbs_7, 0.0000001);
        rosMsg.fINSLonAbs_7 = getScaledValue(rosMsg.INSLonAbs_7, 0.0000001);
        rosMsg.fINSLatRel_7 = getScaledValue(rosMsg.INSLatRel_7, 0.01);
        rosMsg.fINSLonRel_7 = getScaledValue(rosMsg.INSLonRel_7, 0.01);
        rosMsg.fINSLatAbs_8 = getScaledValue(rosMsg.INSLatAbs_8, 0.0000001);
        rosMsg.fINSLonAbs_8 = getScaledValue(rosMsg.INSLonAbs_8, 0.0000001);
        rosMsg.fINSLatRel_8 = getScaledValue(rosMsg.INSLatRel_8, 0.01);
        rosMsg.fINSLonRel_8 = getScaledValue(rosMsg.INSLonRel_8, 0.01);

        rosMsg.fINSVelHorX = getScaledValue(rosMsg.INSVelHorX, 0.005);
        rosMsg.fINSVelHorY = getScaledValue(rosMsg.INSVelHorY, 0.005);
        rosMsg.fINSVelHorZ = getScaledValue(rosMsg.INSVelHorZ, 0.005);
        rosMsg.fINSVelFrameX = getScaledValue(rosMsg.INSVelFrameX, 0.005);
        rosMsg.fINSVelFrameY = getScaledValue(rosMsg.INSVelFrameY, 0.005);
        rosMsg.fINSVelFrameZ = getScaledValue(rosMsg.INSVelFrameZ, 0.005);

        rosMsg.fINSVelHorX_1 = getScaledValue(rosMsg.INSVelHorX_1, 0.005);
        rosMsg.fINSVelHorY_1 = getScaledValue(rosMsg.INSVelHorY_1, 0.005);
        rosMsg.fINSVelHorZ_1 = getScaledValue(rosMsg.INSVelHorZ_1, 0.005);
        rosMsg.fINSVelHorX_2 = getScaledValue(rosMsg.INSVelHorX_2, 0.005);
        rosMsg.fINSVelHorY_2 = getScaledValue(rosMsg.INSVelHorY_2, 0.005);
        rosMsg.fINSVelHorZ_2 = getScaledValue(rosMsg.INSVelHorZ_2, 0.005);
        rosMsg.fINSVelHorX_3 = getScaledValue(rosMsg.INSVelHorX_3, 0.005);
        rosMsg.fINSVelHorY_3 = getScaledValue(rosMsg.INSVelHorY_3, 0.005);
        rosMsg.fINSVelHorZ_3 = getScaledValue(rosMsg.INSVelHorZ_3, 0.005);
        rosMsg.fINSVelHorX_4 = getScaledValue(rosMsg.INSVelHorX_4, 0.005);
        rosMsg.fINSVelHorY_4 = getScaledValue(rosMsg.INSVelHorY_4, 0.005);
        rosMsg.fINSVelHorZ_4 = getScaledValue(rosMsg.INSVelHorZ_4, 0.005);
        rosMsg.fINSVelHorX_5 = getScaledValue(rosMsg.INSVelHorX_5, 0.005);
        rosMsg.fINSVelHorY_5 = getScaledValue(rosMsg.INSVelHorY_5, 0.005);
        rosMsg.fINSVelHorZ_5 = getScaledValue(rosMsg.INSVelHorZ_5, 0.005);
        rosMsg.fINSVelHorX_6 = getScaledValue(rosMsg.INSVelHorX_6, 0.005);
        rosMsg.fINSVelHorY_6 = getScaledValue(rosMsg.INSVelHorY_6, 0.005);
        rosMsg.fINSVelHorZ_6 = getScaledValue(rosMsg.INSVelHorZ_6, 0.005);
        rosMsg.fINSVelHorX_7 = getScaledValue(rosMsg.INSVelHorX_7, 0.005);
        rosMsg.fINSVelHorY_7 = getScaledValue(rosMsg.INSVelHorY_7, 0.005);
        rosMsg.fINSVelHorZ_7 = getScaledValue(rosMsg.INSVelHorZ_7, 0.005);
        rosMsg.fINSVelHorX_8 = getScaledValue(rosMsg.INSVelHorX_8, 0.005);
        rosMsg.fINSVelHorY_8 = getScaledValue(rosMsg.INSVelHorY_8, 0.005);
        rosMsg.fINSVelHorZ_8 = getScaledValue(rosMsg.INSVelHorZ_8, 0.005);

        rosMsg.fINSStddevLat = getScaledValue(rosMsg.INSStddevLat, 0.01);
        rosMsg.fINSStddevLong = getScaledValue(rosMsg.INSStddevLong, 0.01);
        rosMsg.fINSStddevHeight = getScaledValue(rosMsg.INSStddevHeight, 0.01);

        rosMsg.fINSStddevVelX = getScaledValue(rosMsg.INSStddevVelX, 0.01);
        rosMsg.fINSStddevVelY = getScaledValue(rosMsg.INSStddevVelY, 0.01);
        rosMsg.fINSStddevVelZ = getScaledValue(rosMsg.INSStddevVelZ, 0.01);
        rosMsg.fINSStddevRoll = getScaledValue(rosMsg.INSStddevRoll, 0.01);
        rosMsg.fINSStddevPitch = getScaledValue(rosMsg.INSStddevPitch, 0.01);
        rosMsg.fINSStddevYaw = getScaledValue(rosMsg.INSStddevYaw, 0.01);

        rosMsg.fAN1 = getScaledValue(rosMsg.AN1, 0.0005);
        rosMsg.fAN2 = getScaledValue(rosMsg.AN2, 0.0005);
        rosMsg.fAN3 = getScaledValue(rosMsg.AN3, 0.0005);
        rosMsg.fAN4 = getScaledValue(rosMsg.AN4, 0.0005);

        // // only for >= v3.3.3
        rosMsg.fGPSDualAntHeading = getScaledValue(rosMsg.GPSDualAntHeading, 0.01);
        rosMsg.fGPSDualAntPitch = getScaledValue(rosMsg.GPSDualAntPitch, 0.01);
        rosMsg.fGPSDualAntStdDevHeading = getScaledValue(rosMsg.GPSDualAntStdDevHeading, 0.01);
        rosMsg.fGPSDualAntStdDevPitch = getScaledValue(rosMsg.GPSDualAntStdDevPitch, 0.01);
        rosMsg.fGPSDualAntStdDevHeading_HR = getScaledValue(rosMsg.GPSDualAntStdDevHeading_HR, 0.01);
        rosMsg.fGPSDualAntStdDevPitch_HR = getScaledValue(rosMsg.GPSDualAntStdDevPitch_HR, 0.01);
}

void ADMA2ROSParser::extractNavSatFix(adma_ros_driver_msgs::AdmaDataScaled& rosMsg, sensor_msgs::NavSatFix& navRosMsg)
{
        // fil status
        switch (rosMsg.status.status_gnss_mode)
        {
        case 1:
                // No GNSS Data
                navRosMsg.status.status = sensor_msgs::NavSatStatus::STATUS_NO_FIX; 
                break;
        case 2:
                // single GNSS
                navRosMsg.status.status = sensor_msgs::NavSatStatus::STATUS_FIX; 
                break;
        case 4:
                // actually DGNSS Coarse Mode, but used to distinguish here 
                navRosMsg.status.status = sensor_msgs::NavSatStatus::STATUS_SBAS_FIX;
                break;
        case 8:
                // DGNSS Precise Mode
                navRosMsg.status.status = sensor_msgs::NavSatStatus::STATUS_GBAS_FIX;
                break;
        default:
                break;
        }

        navRosMsg.altitude = rosMsg.ins_height;
        navRosMsg.latitude = rosMsg.ins_lat_abs;
        navRosMsg.longitude = rosMsg.ins_long_abs;
        navRosMsg.position_covariance[0] = std::pow(rosMsg.ins_stddev_lat, 2);
        navRosMsg.position_covariance[4] = std::pow(rosMsg.ins_stddev_long, 2);
        navRosMsg.position_covariance[8] = std::pow(rosMsg.ins_stddev_height, 2);

        navRosMsg.position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
}

void ADMA2ROSParser::extractIMU(adma_ros_driver_msgs::AdmaDataScaled& rosMsg, sensor_msgs::Imu& imuRosMsg)
{
        // imuRosMsg.linear_acceleration.x = rosMsg.acc_body_hr.x * 9.81;
        // imuRosMsg.linear_acceleration.y = rosMsg.acc_body_hr.y * 9.81;
        // imuRosMsg.linear_acceleration.z = rosMsg.acc_body_hr.z * 9.81;

        // imuRosMsg.angular_velocity.x = rosMsg.rate_body_hr.x * PI / 180.0;
        // imuRosMsg.angular_velocity.y = rosMsg.rate_body_hr.y * PI / 180.0;
        // imuRosMsg.angular_velocity.z = rosMsg.rate_body_hr.z * PI / 180.0;

        // tf2::Quaternion q;
        // double roll_rad = rosMsg.ins_roll * PI / 180.0;
        // double pitch_rad = rosMsg.ins_pitch * PI / 180.0;
        // double yaw_rad = rosMsg.ins_yaw * PI / 180.0;
        // q.setRPY(roll_rad, pitch_rad, yaw_rad);
        // imuRosMsg.orientation = tf2::toMsg(q);

        // imuRosMsg.orientation_covariance[0] = std::pow(rosMsg.ins_stddev_roll * PI / 180.0, 2);
        // imuRosMsg.orientation_covariance[4] = std::pow(rosMsg.ins_stddev_pitch * PI / 180.0, 2);
        // imuRosMsg.orientation_covariance[8] = std::pow(rosMsg.ins_stddev_yaw * PI / 180.0, 2);

        // // ADMA does not provide covariance for linear acceleration and angular velocity.
        // // These values need to be measured at standstill each ADMA model.
        // imuRosMsg.angular_velocity_covariance[0] = -1;
        // imuRosMsg.linear_acceleration_covariance[0] = -1;
}