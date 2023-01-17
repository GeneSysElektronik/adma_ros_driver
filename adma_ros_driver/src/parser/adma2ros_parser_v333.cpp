#include "adma_ros_driver/parser/adma2ros_parser_v333.hpp"
#include "adma_ros_driver/parser/parser_utils.hpp"

ADMA2ROSParserV333::ADMA2ROSParserV333()
{
}

ADMA2ROSParserV333::~ADMA2ROSParserV333()
{

}

void ADMA2ROSParserV333::mapAdmaMessageToROS(adma_msgs::Adma& rosMsg, AdmaDataV333& admaData)
{
        rosMsg.StatusCount = admaData.statuscount;

        //fill sensor bodies
        rosMsg.AccBodyHRX = admaData.sensorsBodyX.accHR;
        rosMsg.RateBodyHRX = admaData.sensorsBodyX.rateHR;
        rosMsg.AccBodyHRY = admaData.sensorsBodyY.accHR;
        rosMsg.RateBodyHRY = admaData.sensorsBodyY.rateHR;
        rosMsg.AccBodyHRZ = admaData.sensorsBodyZ.accHR;
        rosMsg.RateBodyHRZ = admaData.sensorsBodyZ.rateHR;

        //fill rates
        rosMsg.RateBodyX = admaData.ratesBody.x;
        rosMsg.RateBodyY = admaData.ratesBody.y;
        rosMsg.RateBodyZ = admaData.ratesBody.z;
        rosMsg.RateHorX = admaData.ratesHorizontal.x;
        rosMsg.RateHorY = admaData.ratesHorizontal.y;
        rosMsg.RateHorZ = admaData.ratesHorizontal.z;

        // //fill accelerations
        rosMsg.AccBodyX = admaData.accBody.x;
        rosMsg.AccBodyY = admaData.accBody.y;
        rosMsg.AccBodyZ = admaData.accBody.z;
        rosMsg.AccHorX = admaData.accHorizontal.x;
        rosMsg.AccHorY = admaData.accHorizontal.y;
        rosMsg.AccHorZ = admaData.accHorizontal.z;

        // //fill POI accelerations
        rosMsg.AccBodyX_1 = admaData.accBodyPOI1.x;
        rosMsg.AccBodyY_1 = admaData.accBodyPOI1.y;
        rosMsg.AccBodyZ_1 = admaData.accBodyPOI1.z;
        rosMsg.AccBodyX_2 = admaData.accBodyPOI2.x;
        rosMsg.AccBodyY_2 = admaData.accBodyPOI2.y;
        rosMsg.AccBodyZ_2 = admaData.accBodyPOI2.z;
        rosMsg.AccBodyX_3 = admaData.accBodyPOI3.x;
        rosMsg.AccBodyY_3 = admaData.accBodyPOI3.y;
        rosMsg.AccBodyZ_3 = admaData.accBodyPOI3.z;
        rosMsg.AccBodyX_4 = admaData.accBodyPOI4.x;
        rosMsg.AccBodyY_4 = admaData.accBodyPOI4.y;
        rosMsg.AccBodyZ_4 = admaData.accBodyPOI4.z;
        rosMsg.AccBodyX_5 = admaData.accBodyPOI5.x;
        rosMsg.AccBodyY_5 = admaData.accBodyPOI5.y;
        rosMsg.AccBodyZ_5 = admaData.accBodyPOI5.z;
        rosMsg.AccBodyX_6 = admaData.accBodyPOI6.x;
        rosMsg.AccBodyY_6 = admaData.accBodyPOI6.y;
        rosMsg.AccBodyZ_6 = admaData.accBodyPOI6.z;
        rosMsg.AccBodyX_7 = admaData.accBodyPOI7.x;
        rosMsg.AccBodyY_7 = admaData.accBodyPOI7.y;
        rosMsg.AccBodyZ_7 = admaData.accBodyPOI7.z;
        rosMsg.AccBodyX_8 = admaData.accBodyPOI8.x;
        rosMsg.AccBodyY_8 = admaData.accBodyPOI8.y;
        rosMsg.AccBodyZ_8 = admaData.accBodyPOI8.z;
        
        rosMsg.AccHorX_1 = admaData.accHorizontalPOI1.x;
        rosMsg.AccHorY_1 = admaData.accHorizontalPOI1.y;
        rosMsg.AccHorZ_1 = admaData.accHorizontalPOI1.z;
        rosMsg.AccHorX_2 = admaData.accHorizontalPOI2.x;
        rosMsg.AccHorY_2 = admaData.accHorizontalPOI2.y;
        rosMsg.AccHorZ_2 = admaData.accHorizontalPOI2.z;
        rosMsg.AccHorX_3 = admaData.accHorizontalPOI3.x;
        rosMsg.AccHorY_3 = admaData.accHorizontalPOI3.y;
        rosMsg.AccHorZ_3 = admaData.accHorizontalPOI3.z;
        rosMsg.AccHorX_4 = admaData.accHorizontalPOI4.x;
        rosMsg.AccHorY_4 = admaData.accHorizontalPOI4.y;
        rosMsg.AccHorZ_4 = admaData.accHorizontalPOI4.z;
        rosMsg.AccHorX_5 = admaData.accHorizontalPOI5.x;
        rosMsg.AccHorY_5 = admaData.accHorizontalPOI5.y;
        rosMsg.AccHorZ_5 = admaData.accHorizontalPOI5.z;
        rosMsg.AccHorX_6 = admaData.accHorizontalPOI6.x;
        rosMsg.AccHorY_6 = admaData.accHorizontalPOI6.y;
        rosMsg.AccHorZ_6 = admaData.accHorizontalPOI6.z;
        rosMsg.AccHorX_7 = admaData.accHorizontalPOI7.x;
        rosMsg.AccHorY_7 = admaData.accHorizontalPOI7.y;
        rosMsg.AccHorZ_7 = admaData.accHorizontalPOI7.z;
        rosMsg.AccHorX_8 = admaData.accHorizontalPOI8.x;
        rosMsg.AccHorY_8 = admaData.accHorizontalPOI8.y;
        rosMsg.AccHorZ_8 = admaData.accHorizontalPOI8.z;

        // //fill external velocity
        rosMsg.ExtVelAnX = admaData.extVelAnalog.x;
        rosMsg.ExtVelAnY = admaData.extVelAnalog.y;
        rosMsg.ExtVelDigX = admaData.extveldigx;
        rosMsg.ExtVelDigY = admaData.extveldigy;
        rosMsg.ExtVelDigPulsesX = admaData.extveldigpulsesx;
        rosMsg.ExtVelDigPulsesY = admaData.extveldigpulsesy;
        rosMsg.ExtVelXCorrected = admaData.extVelCorrected.x;
        rosMsg.ExtVelYCorrected = admaData.extVelCorrected.y;

        // Barometer values
        // rosMsg.ExtBaroPressure = admaData.extbaropressure;
        // rosMsg.ExtBaroHeight = admaData.extbaroheight;
        // rosMsg.ExtBaroHeightCorrected = admaData.extbaroheightcorrected;

        // //fill miscellaneous
        rosMsg.InvPathRadius = admaData.misc.invPathRadius;
        rosMsg.SideSlipAngle = admaData.misc.sideSlipAngle;
        rosMsg.DistTrav = admaData.misc.distanceTraveled;

        // //fill miscellaneous POI
        rosMsg.InvPathRadius_1 = admaData.miscPOI1.invPathRadius;
        rosMsg.SideSlipAngle_1 = admaData.miscPOI1.sideSlipAngle;
        rosMsg.DistTrav_1 = admaData.miscPOI1.distanceTraveled;
        rosMsg.InvPathRadius_2 = admaData.miscPOI2.invPathRadius;
        rosMsg.SideSlipAngle_2 = admaData.miscPOI2.sideSlipAngle;
        rosMsg.DistTrav_2 = admaData.miscPOI2.distanceTraveled;
        rosMsg.InvPathRadius_3 = admaData.miscPOI3.invPathRadius;
        rosMsg.SideSlipAngle_3 = admaData.miscPOI3.sideSlipAngle;
        rosMsg.DistTrav_3 = admaData.miscPOI3.distanceTraveled;
        rosMsg.InvPathRadius_4 = admaData.miscPOI4.invPathRadius;
        rosMsg.SideSlipAngle_4 = admaData.miscPOI4.sideSlipAngle;
        rosMsg.DistTrav_4 = admaData.miscPOI4.distanceTraveled;
        rosMsg.InvPathRadius_5 = admaData.miscPOI5.invPathRadius;
        rosMsg.SideSlipAngle_5 = admaData.miscPOI5.sideSlipAngle;
        rosMsg.DistTrav_5 = admaData.miscPOI5.distanceTraveled;
        rosMsg.InvPathRadius_6 = admaData.miscPOI6.invPathRadius;
        rosMsg.SideSlipAngle_6 = admaData.miscPOI6.sideSlipAngle;
        rosMsg.DistTrav_6 = admaData.miscPOI6.distanceTraveled;
        rosMsg.InvPathRadius_7 = admaData.miscPOI7.invPathRadius;
        rosMsg.SideSlipAngle_7 = admaData.miscPOI7.sideSlipAngle;
        rosMsg.DistTrav_7 = admaData.miscPOI7.distanceTraveled;
        rosMsg.InvPathRadius_8 = admaData.miscPOI8.invPathRadius;
        rosMsg.SideSlipAngle_8 = admaData.miscPOI8.sideSlipAngle;
        rosMsg.DistTrav_8 = admaData.miscPOI8.distanceTraveled;

        
        // fill triggers
        rosMsg.TrigRising1 = admaData.trigrising1;
        rosMsg.TrigFalling1 = admaData.trigfalling1;
        rosMsg.TrigRising2 = admaData.trigrising2;
        rosMsg.TrigFalling2 = admaData.trigfalling2;
        rosMsg.TrigRising3 = admaData.trigrising3;
        rosMsg.TrigFalling3 = admaData.trigfalling3;
        rosMsg.TrigRising4 = admaData.trigrising4;
        rosMsg.TrigFalling4 = admaData.trigfalling4;

        // //fill system data
        rosMsg.SystemTa = admaData.systemta;
        rosMsg.SystemTemp = admaData.systemtemp;
        rosMsg.SystemTimeSinceInit = admaData.systemtimesinceinit;
        rosMsg.SystemDSPLoad = admaData.systemdspload;
        
        // //fill GPS position
        rosMsg.GPSLatAbs = admaData.posAbs.latitude;
        rosMsg.GPSLonAbs = admaData.posAbs.longitude;
        rosMsg.GPSLatRel = admaData.posRel.longitude;
        rosMsg.GPSLonRel = admaData.posRel.latitude;

        // // fill GPS Expected Position Error
        rosMsg.GPSStddevLat = admaData.gnssstddevlat;
        rosMsg.GPSStddevLon = admaData.gnssstddevlon;
        rosMsg.GPSStddevHeight = admaData.gnssstddevheight;

        // //fill GPS Velocity
        rosMsg.GPSVelFrameX = admaData.gnssvelframex;
        rosMsg.GPSVelFrameY = admaData.gnssvelframey;
        rosMsg.GPSVelFrameZ = admaData.gnssvelframez;
        rosMsg.GPSVelLatency = admaData.gnssvellatency;

        // //fill GPS Expected Velocity error
        rosMsg.GPSStddevVelX = admaData.gnssStdDevVel.x;
        rosMsg.GPSStddevVelY = admaData.gnssStdDevVel.y;
        rosMsg.GPSStddevVelZ = admaData.gnssStdDevVel.z;
        
        // //fill GPS Time
        rosMsg.GPSTimemsec = admaData.gnsstimemsec;
        rosMsg.GPSTimeWeek = admaData.gnsstimeweek;
        rosMsg.GPSTrigger = admaData.gnsstrigger;

        // //fill GPS AUX data
        rosMsg.GPSDiffAge = admaData.gnssdiffage;
        rosMsg.GPSStatsUsed = admaData.gnsssatsused;
        rosMsg.GPSStatsVisible = admaData.gnsssatsvisible;
        rosMsg.GPSSatsDualAntUsed = admaData.gnsssatsdualantused;
        rosMsg.GPSSatsDualAntVisible = admaData.gnsssatsdualantvisible;
        rosMsg.GPSLogDelay = admaData.gnsslogdelay;
        rosMsg.GPSReceiverLoad = admaData.gnssreceiverload;
        //TODO: check if basenr is string or INT
        // std::stringstream ss;
        // ss <<  admaData.gnssbasenr;
        // rosMsg.GPSBaseNr = ss.str();

        // //fill INS Angle and GPS COG
        rosMsg.INSRoll = admaData.insroll;
        rosMsg.INSPitch = admaData.inspitch;
        rosMsg.INSYaw = admaData.insyaw;
        rosMsg.GPSCOG = admaData.gnsscog;
        
        // //fill GPS Height MSL
        rosMsg.GPSHeight = admaData.gnssheight;
        rosMsg.Undulation = admaData.undulation;

        // // GNSS DUal ant information 
        rosMsg.GPSDualAntTimemsec = admaData.gnssDualAntTimeMsec;
        rosMsg.GPSDualAntTimeWeek = admaData.gnssDualAntTimeWeek;
        rosMsg.GPSDualAntHeading = admaData.gnssDualAntHeading;
        rosMsg.GPSDualAntPitch = admaData.gnssDualAntPitch;

        // //GNSS Dualant ETE
        rosMsg.GPSDualAntStdDevHeading = admaData.gnssdualantstdevheading;
        rosMsg.GPSDualAntStdDevPitch = admaData.gnssdualantstddevpitch;
        rosMsg.GPSDualAntStdDevHeading_HR = admaData.gnssdualantstdevheadinghr;
        rosMsg.GPSDualAntStdDevPitch_HR = admaData.gnssdualantstddevpitchhr;

        // //fill INS height MSL (+ POI)
        rosMsg.INSHeight = admaData.insHeight;
        rosMsg.INSHeight_1 = admaData.insHeightPOI1;
        rosMsg.INSHeight_2 = admaData.insHeightPOI2;
        rosMsg.INSHeight_3 = admaData.insHeightPOI3;
        rosMsg.INSHeight_4 = admaData.insHeightPOI4;
        rosMsg.INSHeight_5 = admaData.insHeightPOI5;
        rosMsg.INSHeight_6 = admaData.insHeightPOI6;
        rosMsg.INSHeight_7 = admaData.insHeightPOI7;
        rosMsg.INSHeight_8 = admaData.insHeightPOI8;

        // //fill INS time UTC
        rosMsg.INSTimemsec = admaData.instimemsec;
        rosMsg.INSTimeWeek = admaData.instimeweek;
        rosMsg.LeapSeconds = admaData.leapseconds;

        // //fill INS Position (+POI)
        rosMsg.INSLatAbs = admaData.insPosAbs.latitude;
        rosMsg.INSLonAbs = admaData.insPosAbs.longitude;
        rosMsg.INSLatRel = admaData.insPosRel.longitude;
        rosMsg.INSLonRel = admaData.insPosRel.latitude;
        rosMsg.INSLatAbs_1 = admaData.insPosAbsPOI1.latitude;
        rosMsg.INSLonAbs_1 = admaData.insPosAbsPOI1.longitude;
        rosMsg.INSLatRel_1 = admaData.insPosRelPOI1.longitude;
        rosMsg.INSLonRel_1 = admaData.insPosRelPOI1.latitude;
        rosMsg.INSLatAbs_2 = admaData.insPosAbsPOI2.latitude;
        rosMsg.INSLonAbs_2 = admaData.insPosAbsPOI2.longitude;
        rosMsg.INSLatRel_2 = admaData.insPosRelPOI2.longitude;
        rosMsg.INSLonRel_2 = admaData.insPosRelPOI2.latitude;
        rosMsg.INSLatAbs_3 = admaData.insPosAbsPOI3.latitude;
        rosMsg.INSLonAbs_3 = admaData.insPosAbsPOI3.longitude;
        rosMsg.INSLatRel_3 = admaData.insPosRelPOI3.longitude;
        rosMsg.INSLonRel_3 = admaData.insPosRelPOI3.latitude;
        rosMsg.INSLatAbs_4 = admaData.insPosAbsPOI4.latitude;
        rosMsg.INSLonAbs_4 = admaData.insPosAbsPOI4.longitude;
        rosMsg.INSLatRel_4 = admaData.insPosRelPOI4.longitude;
        rosMsg.INSLonRel_4 = admaData.insPosRelPOI4.latitude;
        rosMsg.INSLatAbs_5 = admaData.insPosAbsPOI5.latitude;
        rosMsg.INSLonAbs_5 = admaData.insPosAbsPOI5.longitude;
        rosMsg.INSLatRel_5 = admaData.insPosRelPOI5.longitude;
        rosMsg.INSLonRel_5 = admaData.insPosRelPOI5.latitude;
        rosMsg.INSLatAbs_6 = admaData.insPosAbsPOI6.latitude;
        rosMsg.INSLonAbs_6 = admaData.insPosAbsPOI6.longitude;
        rosMsg.INSLatRel_6 = admaData.insPosRelPOI6.longitude;
        rosMsg.INSLonRel_6 = admaData.insPosRelPOI6.latitude;
        rosMsg.INSLatAbs_7 = admaData.insPosAbsPOI7.latitude;
        rosMsg.INSLonAbs_7 = admaData.insPosAbsPOI7.longitude;
        rosMsg.INSLatRel_7 = admaData.insPosRelPOI7.longitude;
        rosMsg.INSLonRel_7 = admaData.insPosRelPOI7.latitude;
        rosMsg.INSLatAbs_8 = admaData.insPosAbsPOI8.latitude;
        rosMsg.INSLonAbs_8 = admaData.insPosAbsPOI8.longitude;
        rosMsg.INSLatRel_8 = admaData.insPosRelPOI8.longitude;
        rosMsg.INSLonRel_8 = admaData.insPosRelPOI8.latitude;
        
        // //fill ins velocity (horizontal + frame)
        rosMsg.INSVelHorX = admaData.insVelHor.x;
        rosMsg.INSVelHorY = admaData.insVelHor.y;
        rosMsg.INSVelHorZ = admaData.insVelHor.z;
        rosMsg.INSVelFrameX = admaData.insVelFrame.x;
        rosMsg.INSVelFrameY = admaData.insVelFrame.y;
        rosMsg.INSVelFrameZ = admaData.insVelFrame.z;

        // //fill INS velocity (POI)
        rosMsg.INSVelHorX_1 = admaData.insVelHorPOI1.x;
        rosMsg.INSVelHorY_1 = admaData.insVelHorPOI1.y;
        rosMsg.INSVelHorZ_1 = admaData.insVelHorPOI1.z;
        rosMsg.INSVelHorX_2 = admaData.insVelHorPOI2.x;
        rosMsg.INSVelHorY_2 = admaData.insVelHorPOI2.y;
        rosMsg.INSVelHorZ_2 = admaData.insVelHorPOI2.z;
        rosMsg.INSVelHorX_3 = admaData.insVelHorPOI3.x;
        rosMsg.INSVelHorY_3 = admaData.insVelHorPOI3.y;
        rosMsg.INSVelHorZ_3 = admaData.insVelHorPOI3.z;
        rosMsg.INSVelHorX_4 = admaData.insVelHorPOI4.x;
        rosMsg.INSVelHorY_4 = admaData.insVelHorPOI4.y;
        rosMsg.INSVelHorZ_4 = admaData.insVelHorPOI4.z;
        rosMsg.INSVelHorX_5 = admaData.insVelHorPOI5.x;
        rosMsg.INSVelHorY_5 = admaData.insVelHorPOI5.y;
        rosMsg.INSVelHorZ_5 = admaData.insVelHorPOI5.z;
        rosMsg.INSVelHorX_6 = admaData.insVelHorPOI6.x;
        rosMsg.INSVelHorY_6 = admaData.insVelHorPOI6.y;
        rosMsg.INSVelHorZ_6 = admaData.insVelHorPOI6.z;
        rosMsg.INSVelHorX_7 = admaData.insVelHorPOI7.x;
        rosMsg.INSVelHorY_7 = admaData.insVelHorPOI7.y;
        rosMsg.INSVelHorZ_7 = admaData.insVelHorPOI7.z;
        rosMsg.INSVelHorX_8 = admaData.insVelHorPOI8.x;
        rosMsg.INSVelHorY_8 = admaData.insVelHorPOI8.y;
        rosMsg.INSVelHorZ_8 = admaData.insVelHorPOI8.z;

        // //fill INS Expected Position Error
        rosMsg.INSStddevLat = admaData.insstddevlat;
        rosMsg.INSStddevLong = admaData.insstddevlong;
        rosMsg.INSStddevHeight = admaData.insstddevheight;
        
        // //fill INS EVE and INS ETE
        rosMsg.INSStddevVelX = admaData.insstddevvelx;
        rosMsg.INSStddevVelY = admaData.insstddevvely;
        rosMsg.INSStddevVelZ = admaData.insstddevvelz;
        rosMsg.INSStddevRoll = admaData.insstddevroll;
        rosMsg.INSStddevPitch = admaData.insstddevpitch;
        rosMsg.INSStddevYaw = admaData.insstddevyaw;

        // //fill Analog in 1
        rosMsg.AN1 = admaData.an1;
        rosMsg.AN2 = admaData.an2;
        rosMsg.AN3 = admaData.an3;
        rosMsg.AN4 = admaData.an4;

        // // kalman filter status
        rosMsg.KFLatStimulated = admaData.kflatstimulated;
        rosMsg.KFLongStimulated = admaData.kflongstimulated;
        rosMsg.KFSteadyState = admaData.kfsteadystate;

        // // gnss receiver status and error
        rosMsg.GPSReceiverError = admaData.gnssreceivererror;
        rosMsg.GPSReceiverStatus = admaData.gnssreceiverstatus;
}

void ADMA2ROSParserV333::getKFStatus(adma_msgs::Adma& rosMsg, unsigned char kfStatus)
{
        bool status_speed_b2 = getbit(kfStatus,5);
        bool status_speed_b1 = getbit(kfStatus,4);
        bool status_kf_steady_state = getbit(kfStatus,3);
        bool status_kf_long_stimulated = getbit(kfStatus,2);
        bool status_kf_lat_stimulated = getbit(kfStatus,1);
        bool status_kalmanfilter_settled = getbit(kfStatus,0);
        rosMsg.StatusKalmanFilterSetteled = status_kalmanfilter_settled;
        rosMsg.StatusKFLatStimulated = status_kf_lat_stimulated;
        rosMsg.StatusKFLongStimulated = status_kf_long_stimulated;
        rosMsg.StatusKFSteadyState = status_kf_steady_state;
        if(status_speed_b1==0 && status_speed_b2==0)
        {
                rosMsg.StatusSpeed = 0;
        }
        else if(status_speed_b1==0 && status_speed_b2==1)
        {
                rosMsg.StatusSpeed = 1;
        }
        else if(status_speed_b1==1 && status_speed_b2==0)
        {
                rosMsg.StatusSpeed = 2;
        }
}