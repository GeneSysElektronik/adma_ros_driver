#include "adma_ros2_driver/parser/adma2ros_parser_v32.hpp"
#include "adma_ros2_driver/parser/parser_utils.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

ADMA2ROSParserV32::ADMA2ROSParserV32()
{
}

void ADMA2ROSParserV32::mapAdmaMessageToROS(adma_msgs::msg::AdmaData& rosMsg, AdmaDataV32& admaData)
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
        rosMsg.ratebodyx = admaData.rateBody.x;
        rosMsg.ratebodyy = admaData.rateBody.y;
        rosMsg.ratebodyz = admaData.rateBody.z;
        rosMsg.ratehorx = admaData.rateHorizontal.x;
        rosMsg.ratehory = admaData.rateHorizontal.y;
        rosMsg.ratehorz = admaData.rateHorizontal.z;

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

        //fill external velocity
        rosMsg.extvelanx = admaData.extVelAnalog.x;
        rosMsg.extvelany = admaData.extVelAnalog.y;
        rosMsg.extveldigx = admaData.extveldigx;
        rosMsg.extveldigy = admaData.extveldigy;
        rosMsg.extveldigpulsesx = admaData.extveldigpulsesx;
        rosMsg.extveldigpulsesy = admaData.extveldigpulsesy;
        rosMsg.extvelxcorrected = admaData.extVelCorrected.x;
        rosMsg.extvelycorrected = admaData.extVelCorrected.y;

        //fill barometer values
        rosMsg.extbaropressure = admaData.extbaropressure;
        rosMsg.extbaroheight = admaData.extbaroheight;
        rosMsg.extbaroheightcorrected = admaData.extbaroheightcorrected;

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
        rosMsg.gpslatabs = admaData.gpsPosAbs.latitude;
        rosMsg.gpslonabs = admaData.gpsPosAbs.longitude;
        rosMsg.gpslatrel = admaData.gpsPosRel.longitude;
        rosMsg.gpslonrel = admaData.gpsPosRel.latitude;

        // fill GPS Expected Position Error
        rosMsg.gpsstddevlat = admaData.gpsstddevlat;
        rosMsg.gpsstddevlon = admaData.gpsstddevlon;
        rosMsg.gpsstddevheight = admaData.gpsstddevheight;

        //fill GPS Velocity
        rosMsg.gpsvelframex = admaData.gpsvelframex;
        rosMsg.gpsvelframey = admaData.gpsvelframey;
        rosMsg.gpsvelframez = admaData.gpsvelframez;
        rosMsg.gpsvellatency = admaData.gpsvellatency;

        //fill GPS Expected Velocity error
        rosMsg.gpsstddevvelx = admaData.gpsStdDevVel.x;
        rosMsg.gpsstddevvely = admaData.gpsStdDevVel.y;
        rosMsg.gpsstddevvelz = admaData.gpsStdDevVel.z;
        

        //fill GPS Time
        rosMsg.gpstimemsec = admaData.gpstimemsec;
        rosMsg.gpstimeweek = admaData.gpstimeweek;
        rosMsg.gpstrigger = admaData.gpstrigger;

        //fill GPS AUX data
        rosMsg.gpsdiffage = admaData.gpsdiffage;
        rosMsg.gpssatsused = admaData.gpssatsused;
        rosMsg.gpssatsvisible = admaData.gpssatsvisible;
        rosMsg.gpslogdelay = admaData.gpslogdelay;
        rosMsg.gpsreceiverload = admaData.gpsreceiverload;
        std::stringstream ss;
        ss <<  admaData.gpsbasenr;
        rosMsg.gpsbasenr = ss.str();

        //fill INS Angle and GPS COG
        rosMsg.insroll = admaData.insroll;
        rosMsg.inspitch = admaData.inspitch;
        rosMsg.insyaw = admaData.insyaw;
        rosMsg.gpscog = admaData.gpscog;
        
        //fill GPS Height MSL
        rosMsg.gpsheight = admaData.gpsheight;
        rosMsg.undulation = admaData.undulation;

        //fill INS height MSL (+ POI)
        rosMsg.insheight = admaData.insHeight;
        rosMsg.insheight_1 = admaData.insHeightPOI1;
        rosMsg.insheight_2 = admaData.insHeightPOI2;
        rosMsg.insheight_3 = admaData.insHeightPOI3;
        rosMsg.insheight_4 = admaData.insHeightPOI4;
        rosMsg.insheight_5 = admaData.insHeightPOI5;
        rosMsg.insheight_6 = admaData.insHeightPOI6;
        rosMsg.insheight_7 = admaData.insHeightPOI7;

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
}