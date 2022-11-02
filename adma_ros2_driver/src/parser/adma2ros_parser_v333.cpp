#include "adma_ros2_driver/parser/adma2ros_parser_v333.hpp"
#include "adma_ros2_driver/parser/parser_utils.hpp"

ADMA2ROSParserV333::ADMA2ROSParserV333()
{
}

ADMA2ROSParserV333::~ADMA2ROSParserV333()
{

}

void ADMA2ROSParserV333::mapAdmaMessageToROS(adma_msgs::msg::AdmaData& rosMsg, AdmaDataV333& admaData)
{
        rosMsg.statuscount = admaData.statuscount;

        //fill sensor bodies
        rosMsg.accbodyhrx = admaData.sensorsBodyX.accHR;
        rosMsg.ratebodyhrx = admaData.sensorsBodyX.rateHR;
        rosMsg.accbodyhry = admaData.sensorsBodyY.accHR;
        rosMsg.ratebodyhry = admaData.sensorsBodyY.rateHR;
        rosMsg.accbodyhrz = admaData.sensorsBodyZ.accHR;
        rosMsg.ratebodyhrz = admaData.sensorsBodyZ.rateHR;

        //fill rates
        rosMsg.ratebodyx = admaData.ratesBody.x;
        rosMsg.ratebodyy = admaData.ratesBody.y;
        rosMsg.ratebodyz = admaData.ratesBody.z;
        rosMsg.ratehorx = admaData.ratesHorizontal.x;
        rosMsg.ratehory = admaData.ratesHorizontal.y;
        rosMsg.ratehorz = admaData.ratesHorizontal.z;

        //fill accelerations
        rosMsg.accbodyx = admaData.accBody.x;
        rosMsg.accbodyy = admaData.accBody.y;
        rosMsg.accbodyz = admaData.accBody.z;
        rosMsg.acchorx = admaData.accHorizontal.x;
        rosMsg.acchory = admaData.accHorizontal.y;
        rosMsg.acchorz = admaData.accHorizontal.z;

        //fill POI accelerations
        rosMsg.accbodyx_1 = admaData.accBodyPOI1.x;
        rosMsg.accbodyy_1 = admaData.accBodyPOI1.y;
        rosMsg.accbodyz_1 = admaData.accBodyPOI1.z;
        rosMsg.accbodyx_2 = admaData.accBodyPOI2.x;
        rosMsg.accbodyy_2 = admaData.accBodyPOI2.y;
        rosMsg.accbodyz_2 = admaData.accBodyPOI2.z;
        rosMsg.accbodyx_3 = admaData.accBodyPOI3.x;
        rosMsg.accbodyy_3 = admaData.accBodyPOI3.y;
        rosMsg.accbodyz_3 = admaData.accBodyPOI3.z;
        rosMsg.accbodyx_4 = admaData.accBodyPOI4.x;
        rosMsg.accbodyy_4 = admaData.accBodyPOI4.y;
        rosMsg.accbodyz_4 = admaData.accBodyPOI4.z;
        rosMsg.accbodyx_5 = admaData.accBodyPOI5.x;
        rosMsg.accbodyy_5 = admaData.accBodyPOI5.y;
        rosMsg.accbodyz_5 = admaData.accBodyPOI5.z;
        rosMsg.accbodyx_6 = admaData.accBodyPOI6.x;
        rosMsg.accbodyy_6 = admaData.accBodyPOI6.y;
        rosMsg.accbodyz_6 = admaData.accBodyPOI6.z;
        rosMsg.accbodyx_7 = admaData.accBodyPOI7.x;
        rosMsg.accbodyy_7 = admaData.accBodyPOI7.y;
        rosMsg.accbodyz_7 = admaData.accBodyPOI7.z;
        rosMsg.accbodyx_8 = admaData.accBodyPOI8.x;
        rosMsg.accbodyy_8 = admaData.accBodyPOI8.y;
        rosMsg.accbodyz_8 = admaData.accBodyPOI8.z;
        
        rosMsg.acchorx_1 = admaData.accHorizontalPOI1.x;
        rosMsg.acchory_1 = admaData.accHorizontalPOI1.y;
        rosMsg.acchorz_1 = admaData.accHorizontalPOI1.z;
        rosMsg.acchorx_2 = admaData.accHorizontalPOI2.x;
        rosMsg.acchory_2 = admaData.accHorizontalPOI2.y;
        rosMsg.acchorz_2 = admaData.accHorizontalPOI2.z;
        rosMsg.acchorx_3 = admaData.accHorizontalPOI3.x;
        rosMsg.acchory_3 = admaData.accHorizontalPOI3.y;
        rosMsg.acchorz_3 = admaData.accHorizontalPOI3.z;
        rosMsg.acchorx_4 = admaData.accHorizontalPOI4.x;
        rosMsg.acchory_4 = admaData.accHorizontalPOI4.y;
        rosMsg.acchorz_4 = admaData.accHorizontalPOI4.z;
        rosMsg.acchorx_5 = admaData.accHorizontalPOI5.x;
        rosMsg.acchory_5 = admaData.accHorizontalPOI5.y;
        rosMsg.acchorz_5 = admaData.accHorizontalPOI5.z;
        rosMsg.acchorx_6 = admaData.accHorizontalPOI6.x;
        rosMsg.acchory_6 = admaData.accHorizontalPOI6.y;
        rosMsg.acchorz_6 = admaData.accHorizontalPOI6.z;
        rosMsg.acchorx_7 = admaData.accHorizontalPOI7.x;
        rosMsg.acchory_7 = admaData.accHorizontalPOI7.y;
        rosMsg.acchorz_7 = admaData.accHorizontalPOI7.z;
        rosMsg.acchorx_8 = admaData.accHorizontalPOI8.x;
        rosMsg.acchory_8 = admaData.accHorizontalPOI8.y;
        rosMsg.acchorz_8 = admaData.accHorizontalPOI8.z;

        //fill external velocity
        rosMsg.extvelanx = admaData.extVelAnalog.x;
        rosMsg.extvelany = admaData.extVelAnalog.y;
        rosMsg.extveldigx = admaData.extveldigx;
        rosMsg.extveldigy = admaData.extveldigy;
        rosMsg.extveldigpulsesx = admaData.extveldigpulsesx;
        rosMsg.extveldigpulsesy = admaData.extveldigpulsesy;
        rosMsg.extvelxcorrected = admaData.extVelCorrected.x;
        rosMsg.extvelycorrected = admaData.extVelCorrected.y;

        //fill miscellaneous
        rosMsg.invpathradius = admaData.misc.invPathRadius;
        rosMsg.sideslipangle = admaData.misc.sideSlipAngle;
        rosMsg.disttrav = admaData.misc.distanceTraveled;

        //fill miscellaneous POI
        rosMsg.invpathradius_1 = admaData.miscPOI1.invPathRadius;
        rosMsg.sideslipangle_1 = admaData.miscPOI1.sideSlipAngle;
        rosMsg.disttrav_1 = admaData.miscPOI1.distanceTraveled;
        rosMsg.invpathradius_2 = admaData.miscPOI2.invPathRadius;
        rosMsg.sideslipangle_2 = admaData.miscPOI2.sideSlipAngle;
        rosMsg.disttrav_2 = admaData.miscPOI2.distanceTraveled;
        rosMsg.invpathradius_3 = admaData.miscPOI3.invPathRadius;
        rosMsg.sideslipangle_3 = admaData.miscPOI3.sideSlipAngle;
        rosMsg.disttrav_3 = admaData.miscPOI3.distanceTraveled;
        rosMsg.invpathradius_4 = admaData.miscPOI4.invPathRadius;
        rosMsg.sideslipangle_4 = admaData.miscPOI4.sideSlipAngle;
        rosMsg.disttrav_4 = admaData.miscPOI4.distanceTraveled;
        rosMsg.invpathradius_5 = admaData.miscPOI5.invPathRadius;
        rosMsg.sideslipangle_5 = admaData.miscPOI5.sideSlipAngle;
        rosMsg.disttrav_5 = admaData.miscPOI5.distanceTraveled;
        rosMsg.invpathradius_6 = admaData.miscPOI6.invPathRadius;
        rosMsg.sideslipangle_6 = admaData.miscPOI6.sideSlipAngle;
        rosMsg.disttrav_6 = admaData.miscPOI6.distanceTraveled;
        rosMsg.invpathradius_7 = admaData.miscPOI7.invPathRadius;
        rosMsg.sideslipangle_7 = admaData.miscPOI7.sideSlipAngle;
        rosMsg.disttrav_7 = admaData.miscPOI7.distanceTraveled;
        rosMsg.invpathradius_8 = admaData.miscPOI8.invPathRadius;
        rosMsg.sideslipangle_8 = admaData.miscPOI8.sideSlipAngle;
        rosMsg.disttrav_8 = admaData.miscPOI8.distanceTraveled;

        // fill triggers
        rosMsg.trigrising1 = admaData.trigrising1;
        rosMsg.trigfalling1 = admaData.trigfalling1;
        rosMsg.trigrising2 = admaData.trigrising2;
        rosMsg.trigfalling2 = admaData.trigfalling2;
        rosMsg.trigrising3 = admaData.trigrising3;
        rosMsg.trigfalling3 = admaData.trigfalling3;
        rosMsg.trigrising4 = admaData.trigrising4;
        rosMsg.trigfalling4 = admaData.trigfalling4;

        //fill system data
        rosMsg.systemta = admaData.systemta;
        rosMsg.systemtemp = admaData.systemtemp;
        rosMsg.systemtimesinceinit = admaData.systemtimesinceinit;
        rosMsg.systemdspload = admaData.systemdspload;
        
        //fill GPS position
        rosMsg.gpslatabs = admaData.posAbs.latitude;
        rosMsg.gpslonabs = admaData.posAbs.longitude;
        rosMsg.gpslatrel = admaData.posRel.longitude;
        rosMsg.gpslonrel = admaData.posRel.latitude;

        // fill GPS Expected Position Error
        rosMsg.gpsstddevlat = admaData.gnssstddevlat;
        rosMsg.gpsstddevlon = admaData.gnssstddevlon;
        rosMsg.gpsstddevheight = admaData.gnssstddevheight;

        //fill GPS Velocity
        rosMsg.gpsvelframex = admaData.gnssvelframex;
        rosMsg.gpsvelframey = admaData.gnssvelframey;
        rosMsg.gpsvelframez = admaData.gnssvelframez;
        rosMsg.gpsvellatency = admaData.gnssvellatency;

        //fill GPS Expected Velocity error
        rosMsg.gpsstddevvelx = admaData.gnssStdDevVel.x;
        rosMsg.gpsstddevvely = admaData.gnssStdDevVel.y;
        rosMsg.gpsstddevvelz = admaData.gnssStdDevVel.z;
        
        //fill GPS Time
        rosMsg.gpstimemsec = admaData.gnsstimemsec;
        rosMsg.gpstimeweek = admaData.gnsstimeweek;
        rosMsg.gpstrigger = admaData.gnsstrigger;

        //fill GPS AUX data
        rosMsg.gpsdiffage = admaData.gnssdiffage;
        rosMsg.gpssatsused = admaData.gnsssatsused;
        rosMsg.gpssatsvisible = admaData.gnsssatsvisible;
        rosMsg.gpssatsdualantused = admaData.gnsssatsdualantused;
        rosMsg.gpssatsdualantvisible = admaData.gnsssatsdualantvisible;
        rosMsg.gpslogdelay = admaData.gnsslogdelay;
        rosMsg.gpsreceiverload = admaData.gnssreceiverload;
        std::stringstream ss;
        ss <<  admaData.gnssbasenr;
        rosMsg.gpsbasenr = ss.str();

        //fill INS Angle and GPS COG
        rosMsg.insroll = admaData.insroll;
        rosMsg.inspitch = admaData.inspitch;
        rosMsg.insyaw = admaData.insyaw;
        rosMsg.gpscog = admaData.gnsscog;
        
        //fill GPS Height MSL
        rosMsg.gpsheight = admaData.gnssheight;
        rosMsg.undulation = admaData.undulation;

        // GNSS DUal ant information 
        rosMsg.gpsdualanttimemsec = admaData.gnssDualAntTimeMsec;
        rosMsg.gpsdualanttimeweek = admaData.gnssDualAntTimeWeek;
        rosMsg.gpsdualantheading = admaData.gnssDualAntHeading;
        rosMsg.gpsdualantpitch = admaData.gnssDualAntPitch;

        //GNSS Dualant ETE
        rosMsg.gpsdualantstddevheading = admaData.gnssdualantstdevheading;
        rosMsg.gpsdualantstddevpitch = admaData.gnssdualantstddevpitch;
        rosMsg.gpsdualantstddevheading_hr = admaData.gnssdualantstdevheadinghr;
        rosMsg.gpsdualantstddevpitch_hr = admaData.gnssdualantstddevpitchhr;

        //fill INS height MSL (+ POI)
        rosMsg.insheight = admaData.insHeight;
        rosMsg.insheight_1 = admaData.insHeightPOI1;
        rosMsg.insheight_2 = admaData.insHeightPOI2;
        rosMsg.insheight_3 = admaData.insHeightPOI3;
        rosMsg.insheight_4 = admaData.insHeightPOI4;
        rosMsg.insheight_5 = admaData.insHeightPOI5;
        rosMsg.insheight_6 = admaData.insHeightPOI6;
        rosMsg.insheight_7 = admaData.insHeightPOI7;
        rosMsg.insheight_8 = admaData.insHeightPOI8;

        //fill INS time UTC
        rosMsg.instimemsec = admaData.instimemsec;
        rosMsg.instimeweek = admaData.instimeweek;
        rosMsg.leapseconds = admaData.leapseconds;

        //fill INS Position (+POI)
        rosMsg.inslatabs = admaData.insPosAbs.latitude;
        rosMsg.inslonabs = admaData.insPosAbs.longitude;
        rosMsg.inslatrel = admaData.insPosRel.longitude;
        rosMsg.inslonrel = admaData.insPosRel.latitude;
        rosMsg.inslatabs_1 = admaData.insPosAbsPOI1.latitude;
        rosMsg.inslonabs_1 = admaData.insPosAbsPOI1.longitude;
        rosMsg.inslatrel_1 = admaData.insPosRelPOI1.longitude;
        rosMsg.inslonrel_1 = admaData.insPosRelPOI1.latitude;
        rosMsg.inslatabs_2 = admaData.insPosAbsPOI2.latitude;
        rosMsg.inslonabs_2 = admaData.insPosAbsPOI2.longitude;
        rosMsg.inslatrel_2 = admaData.insPosRelPOI2.longitude;
        rosMsg.inslonrel_2 = admaData.insPosRelPOI2.latitude;
        rosMsg.inslatabs_3 = admaData.insPosAbsPOI3.latitude;
        rosMsg.inslonabs_3 = admaData.insPosAbsPOI3.longitude;
        rosMsg.inslatrel_3 = admaData.insPosRelPOI3.longitude;
        rosMsg.inslonrel_3 = admaData.insPosRelPOI3.latitude;
        rosMsg.inslatabs_4 = admaData.insPosAbsPOI4.latitude;
        rosMsg.inslonabs_4 = admaData.insPosAbsPOI4.longitude;
        rosMsg.inslatrel_4 = admaData.insPosRelPOI4.longitude;
        rosMsg.inslonrel_4 = admaData.insPosRelPOI4.latitude;
        rosMsg.inslatabs_5 = admaData.insPosAbsPOI5.latitude;
        rosMsg.inslonabs_5 = admaData.insPosAbsPOI5.longitude;
        rosMsg.inslatrel_5 = admaData.insPosRelPOI5.longitude;
        rosMsg.inslonrel_5 = admaData.insPosRelPOI5.latitude;
        rosMsg.inslatabs_6 = admaData.insPosAbsPOI6.latitude;
        rosMsg.inslonabs_6 = admaData.insPosAbsPOI6.longitude;
        rosMsg.inslatrel_6 = admaData.insPosRelPOI6.longitude;
        rosMsg.inslonrel_6 = admaData.insPosRelPOI6.latitude;
        rosMsg.inslatabs_7 = admaData.insPosAbsPOI7.latitude;
        rosMsg.inslonabs_7 = admaData.insPosAbsPOI7.longitude;
        rosMsg.inslatrel_7 = admaData.insPosRelPOI7.longitude;
        rosMsg.inslonrel_7 = admaData.insPosRelPOI7.latitude;
        rosMsg.inslatabs_8 = admaData.insPosAbsPOI8.latitude;
        rosMsg.inslonabs_8 = admaData.insPosAbsPOI8.longitude;
        rosMsg.inslatrel_8 = admaData.insPosRelPOI8.longitude;
        rosMsg.inslonrel_8 = admaData.insPosRelPOI8.latitude;
        
        //fill ins velocity (horizontal + frame)
        rosMsg.insvelhorx = admaData.insVelHor.x;
        rosMsg.insvelhory = admaData.insVelHor.y;
        rosMsg.insvelhorz = admaData.insVelHor.z;
        rosMsg.insvelframex = admaData.insVelFrame.x;
        rosMsg.insvelframey = admaData.insVelFrame.y;
        rosMsg.insvelframez = admaData.insVelFrame.z;

        //fill INS velocity (POI)
        rosMsg.insvelhorx_1 = admaData.insVelHorPOI1.x;
        rosMsg.insvelhory_1 = admaData.insVelHorPOI1.y;
        rosMsg.insvelhorz_1 = admaData.insVelHorPOI1.z;
        rosMsg.insvelhorx_2 = admaData.insVelHorPOI2.x;
        rosMsg.insvelhory_2 = admaData.insVelHorPOI2.y;
        rosMsg.insvelhorz_2 = admaData.insVelHorPOI2.z;
        rosMsg.insvelhorx_3 = admaData.insVelHorPOI3.x;
        rosMsg.insvelhory_3 = admaData.insVelHorPOI3.y;
        rosMsg.insvelhorz_3 = admaData.insVelHorPOI3.z;
        rosMsg.insvelhorx_4 = admaData.insVelHorPOI4.x;
        rosMsg.insvelhory_4 = admaData.insVelHorPOI4.y;
        rosMsg.insvelhorz_4 = admaData.insVelHorPOI4.z;
        rosMsg.insvelhorx_5 = admaData.insVelHorPOI5.x;
        rosMsg.insvelhory_5 = admaData.insVelHorPOI5.y;
        rosMsg.insvelhorz_5 = admaData.insVelHorPOI5.z;
        rosMsg.insvelhorx_6 = admaData.insVelHorPOI6.x;
        rosMsg.insvelhory_6 = admaData.insVelHorPOI6.y;
        rosMsg.insvelhorz_6 = admaData.insVelHorPOI6.z;
        rosMsg.insvelhorx_7 = admaData.insVelHorPOI7.x;
        rosMsg.insvelhory_7 = admaData.insVelHorPOI7.y;
        rosMsg.insvelhorz_7 = admaData.insVelHorPOI7.z;
        rosMsg.insvelhorx_8 = admaData.insVelHorPOI8.x;
        rosMsg.insvelhory_8 = admaData.insVelHorPOI8.y;
        rosMsg.insvelhorz_8 = admaData.insVelHorPOI8.z;

        //fill INS Expected Position Error
        rosMsg.insstddevlat = admaData.insstddevlat;
        rosMsg.insstddevlong = admaData.insstddevlong;
        rosMsg.insstddevheight = admaData.insstddevheight;
        
        //fill INS EVE and INS ETE
        rosMsg.insstddevvelx = admaData.insstddevvelx;
        rosMsg.insstddevvely = admaData.insstddevvely;
        rosMsg.insstddevvelz = admaData.insstddevvelz;
        rosMsg.insstddevroll = admaData.insstddevroll;
        rosMsg.insstddevpitch = admaData.insstddevpitch;
        rosMsg.insstddevyaw = admaData.insstddevyaw;

        //fill Analog in 1
        rosMsg.an1 = admaData.an1;
        rosMsg.an2 = admaData.an2;
        rosMsg.an3 = admaData.an3;
        rosMsg.an4 = admaData.an4;

        // kalman filter status
        rosMsg.kflatstimulated = admaData.kflatstimulated;
        rosMsg.kflongstimulated = admaData.kflongstimulated;
        rosMsg.kfsteadystate = admaData.kfsteadystate;

        // gnss receiver status and error
        rosMsg.gpsreceivererror = admaData.gnssreceivererror;
        rosMsg.gpsreceiverstatus = admaData.gnssreceiverstatus;
}

void ADMA2ROSParserV333::getKFStatus(adma_msgs::msg::AdmaData& rosMsg, unsigned char kfStatus)
{
        bool status_speed_b2 = getbit(kfStatus,5);
        bool status_speed_b1 = getbit(kfStatus,4);
        bool status_kf_steady_state = getbit(kfStatus,3);
        bool status_kf_long_stimulated = getbit(kfStatus,2);
        bool status_kf_lat_stimulated = getbit(kfStatus,1);
        bool status_kalmanfilter_settled = getbit(kfStatus,0);
        rosMsg.statuskalmanfiltersetteled = status_kalmanfilter_settled;
        rosMsg.statuskflatstimulated = status_kf_lat_stimulated;
        rosMsg.statuskflongstimulated = status_kf_long_stimulated;
        rosMsg.statuskfsteadystate = status_kf_steady_state;
        if(status_speed_b1==0 && status_speed_b2==0)
        {
                rosMsg.statusspeed = 0;
        }
        else if(status_speed_b1==0 && status_speed_b2==1)
        {
                rosMsg.statusspeed = 1;
        }
        else if(status_speed_b1==1 && status_speed_b2==0)
        {
                rosMsg.statusspeed = 2;
        }
}