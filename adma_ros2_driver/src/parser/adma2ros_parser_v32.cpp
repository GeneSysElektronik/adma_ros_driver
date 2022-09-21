#include "adma_ros2_driver/parser/adma2ros_parser_v32.hpp"
#include "adma_ros2_driver/parser/parser_utils.hpp"

void mapAdmaMessageToROS(adma_msgs::msg::AdmaData& rosMsg, AdmaDataV32& admaMsg){
        // fill static header information
        rosMsg.genesysid = admaMsg.staticHeader.genesysid;
        std::stringstream ss;
        ss <<  int(admaMsg.staticHeader.headerversion[0]) << int(admaMsg.staticHeader.headerversion[1]) << int(admaMsg.staticHeader.headerversion[2]) << int(admaMsg.staticHeader.headerversion[3]);
        rosMsg.headerversion = ss.str();
        ss.clear();
        ss.str("");
        rosMsg.formatid = admaMsg.staticHeader.formatid;
        //TODO: this value is parsed wrong?!        
        ss <<  int(admaMsg.staticHeader.formatversion[0]) << int(admaMsg.staticHeader.formatversion[1]) << int(admaMsg.staticHeader.formatversion[2]) << int(admaMsg.staticHeader.formatversion[3]);
        rosMsg.formatversion = ss.str();
        rosMsg.serialno = admaMsg.staticHeader.serialno;

        // fill dynamic header information
        rosMsg.configid = admaMsg.dynamicHeader.configid;
        rosMsg.configformat = admaMsg.dynamicHeader.configformat;
        rosMsg.configversion = admaMsg.dynamicHeader.configversion;
        rosMsg.configsize = admaMsg.dynamicHeader.configsize;
        rosMsg.byteoffset = admaMsg.dynamicHeader.byteoffset;
        rosMsg.slicesize = admaMsg.dynamicHeader.slicesize;
        rosMsg.slicedata = admaMsg.dynamicHeader.slicedata;

        getstatusgps(rosMsg, admaMsg);
        getstatustrigger(rosMsg, admaMsg);
        getevkstatus(rosMsg, admaMsg);
        rosMsg.statuscount = admaMsg.statuscount;
        geterrorandwarning(rosMsg, admaMsg);

        //fill sensor bodies
        rosMsg.accbodyhrx = admaMsg.sensorsBodyX.accHR;
        rosMsg.faccbodyhrx = getScaledValue(admaMsg.sensorsBodyX.accHR, 0.0001);
        rosMsg.ratebodyhrx = admaMsg.sensorsBodyX.rateHR;
        rosMsg.ratebodyhrx = getScaledValue(admaMsg.sensorsBodyX.rateHR, 0.0001);
        rosMsg.accbodyhry = admaMsg.sensorsBodyY.accHR;
        rosMsg.faccbodyhry = getScaledValue(admaMsg.sensorsBodyY.accHR, 0.0001);
        rosMsg.ratebodyhry = admaMsg.sensorsBodyY.rateHR;
        rosMsg.ratebodyhry = getScaledValue(admaMsg.sensorsBodyY.rateHR, 0.0001);
        rosMsg.accbodyhrz = admaMsg.sensorsBodyZ.accHR;
        rosMsg.faccbodyhrz = getScaledValue(admaMsg.sensorsBodyZ.accHR, 0.0001);
        rosMsg.ratebodyhrz = admaMsg.sensorsBodyZ.rateHR;
        rosMsg.ratebodyhrz = getScaledValue(admaMsg.sensorsBodyZ.rateHR, 0.0001);

        //fill rates
        rosMsg.ratebodyx = admaMsg.rateBody.x;
        rosMsg.fratebodyx = getScaledValue(admaMsg.rateBody.x, 0.01);
        rosMsg.ratebodyy = admaMsg.rateBody.y;
        rosMsg.fratebodyy = getScaledValue(admaMsg.rateBody.y, 0.01);
        rosMsg.ratebodyz = admaMsg.rateBody.z;
        rosMsg.fratebodyz = getScaledValue(admaMsg.rateBody.z, 0.01);
        rosMsg.ratehorx = admaMsg.rateHorizontal.x;
        rosMsg.fratehorx = getScaledValue(admaMsg.rateHorizontal.x, 0.01);
        rosMsg.ratehory = admaMsg.rateHorizontal.y;
        rosMsg.fratehory = getScaledValue(admaMsg.rateHorizontal.y, 0.01);
        rosMsg.ratehorz = admaMsg.rateHorizontal.z;
        rosMsg.fratehorz = getScaledValue(admaMsg.rateHorizontal.z, 0.01);

        //fill accelerations
        rosMsg.accbodyx = admaMsg.accBody.x;
        rosMsg.faccbodyx = getScaledValue(admaMsg.accBody.x, 0.0004);
        rosMsg.accbodyy = admaMsg.accBody.y;
        rosMsg.faccbodyy = getScaledValue(admaMsg.accBody.y, 0.0004);
        rosMsg.accbodyz = admaMsg.accBody.z;
        rosMsg.faccbodyz = getScaledValue(admaMsg.accBody.z, 0.0004);
        rosMsg.acchorx = admaMsg.accHorizontal.x;
        rosMsg.facchorx = getScaledValue(admaMsg.accHorizontal.x, 0.0004);
        rosMsg.acchory = admaMsg.accHorizontal.y;
        rosMsg.facchory = getScaledValue(admaMsg.accHorizontal.y, 0.0004);
        rosMsg.acchorz = admaMsg.accHorizontal.z;
        rosMsg.facchorz = getScaledValue(admaMsg.accHorizontal.z, 0.0004);

        //fill POI accelerations
        rosMsg.accbodyx_1 = admaMsg.accBodyPOI1.x;
        rosMsg.faccbodyx_1 = getScaledValue(admaMsg.accBodyPOI1.x, 0.0004);
        rosMsg.accbodyy_1 = admaMsg.accBodyPOI1.y;
        rosMsg.faccbodyy_1 = getScaledValue(admaMsg.accBodyPOI1.y, 0.0004);
        rosMsg.accbodyz_1 = admaMsg.accBodyPOI1.z;
        rosMsg.faccbodyz_1 = getScaledValue(admaMsg.accBodyPOI1.z, 0.0004);
        rosMsg.accbodyx_2 = admaMsg.accBodyPOI2.x;
        rosMsg.faccbodyx_2 = getScaledValue(admaMsg.accBodyPOI2.x, 0.0004);
        rosMsg.accbodyy_2 = admaMsg.accBodyPOI2.y;
        rosMsg.faccbodyy_2 = getScaledValue(admaMsg.accBodyPOI2.y, 0.0004);
        rosMsg.accbodyz_2 = admaMsg.accBodyPOI2.z;
        rosMsg.faccbodyz_2 = getScaledValue(admaMsg.accBodyPOI2.z, 0.0004);
        rosMsg.accbodyx_3 = admaMsg.accBodyPOI3.x;
        rosMsg.faccbodyx_3 = getScaledValue(admaMsg.accBodyPOI3.x, 0.0004);
        rosMsg.accbodyy_3 = admaMsg.accBodyPOI3.y;
        rosMsg.faccbodyy_3 = getScaledValue(admaMsg.accBodyPOI3.y, 0.0004);
        rosMsg.accbodyz_3 = admaMsg.accBodyPOI3.z;
        rosMsg.faccbodyz_3 = getScaledValue(admaMsg.accBodyPOI3.z, 0.0004);
        rosMsg.accbodyx_4 = admaMsg.accBodyPOI4.x;
        rosMsg.faccbodyx_4 = getScaledValue(admaMsg.accBodyPOI4.x, 0.0004);
        rosMsg.accbodyy_4 = admaMsg.accBodyPOI4.y;
        rosMsg.faccbodyy_4 = getScaledValue(admaMsg.accBodyPOI4.y, 0.0004);
        rosMsg.accbodyz_4 = admaMsg.accBodyPOI4.z;
        rosMsg.faccbodyz_4 = getScaledValue(admaMsg.accBodyPOI4.z, 0.0004);
        rosMsg.accbodyx_5 = admaMsg.accBodyPOI5.x;
        rosMsg.faccbodyx_5 = getScaledValue(admaMsg.accBodyPOI5.x, 0.0004);
        rosMsg.accbodyy_5 = admaMsg.accBodyPOI5.y;
        rosMsg.faccbodyy_5 = getScaledValue(admaMsg.accBodyPOI5.y, 0.0004);
        rosMsg.accbodyz_5 = admaMsg.accBodyPOI5.z;
        rosMsg.faccbodyz_5 = getScaledValue(admaMsg.accBodyPOI5.z, 0.0004);
        rosMsg.accbodyx_6 = admaMsg.accBodyPOI6.x;
        rosMsg.faccbodyx_6 = getScaledValue(admaMsg.accBodyPOI6.x, 0.0004);
        rosMsg.accbodyy_6 = admaMsg.accBodyPOI6.y;
        rosMsg.faccbodyy_6 = getScaledValue(admaMsg.accBodyPOI6.y, 0.0004);
        rosMsg.accbodyz_6 = admaMsg.accBodyPOI6.z;
        rosMsg.faccbodyz_6 = getScaledValue(admaMsg.accBodyPOI6.z, 0.0004);
        rosMsg.accbodyx_7 = admaMsg.accBodyPOI7.x;
        rosMsg.faccbodyx_7 = getScaledValue(admaMsg.accBodyPOI7.x, 0.0004);
        rosMsg.accbodyy_7 = admaMsg.accBodyPOI7.y;
        rosMsg.faccbodyy_7 = getScaledValue(admaMsg.accBodyPOI7.y, 0.0004);
        rosMsg.accbodyz_7 = admaMsg.accBodyPOI7.z;
        rosMsg.faccbodyz_7 = getScaledValue(admaMsg.accBodyPOI7.z, 0.0004);
        rosMsg.acchorx_1 = admaMsg.accHorizontalPOI1.x;
        rosMsg.facchorx_1 = getScaledValue(admaMsg.accHorizontalPOI1.x, 0.0004);
        rosMsg.acchory_1 = admaMsg.accHorizontalPOI1.y;
        rosMsg.facchory_1 = getScaledValue(admaMsg.accHorizontalPOI1.y, 0.0004);
        rosMsg.acchorz_1 = admaMsg.accHorizontalPOI1.z;
        rosMsg.facchorz_1 = getScaledValue(admaMsg.accHorizontalPOI1.z, 0.0004);
        rosMsg.acchorx_2 = admaMsg.accHorizontalPOI2.x;
        rosMsg.facchorx_2 = getScaledValue(admaMsg.accHorizontalPOI2.x, 0.0004);
        rosMsg.acchory_2 = admaMsg.accHorizontalPOI2.y;
        rosMsg.facchory_2 = getScaledValue(admaMsg.accHorizontalPOI2.y, 0.0004);
        rosMsg.acchorz_2 = admaMsg.accHorizontalPOI2.z;
        rosMsg.facchorz_2 = getScaledValue(admaMsg.accHorizontalPOI2.z, 0.0004);
        rosMsg.acchorx_3 = admaMsg.accHorizontalPOI3.x;
        rosMsg.facchorx_3 = getScaledValue(admaMsg.accHorizontalPOI3.x, 0.0004);
        rosMsg.acchory_3 = admaMsg.accHorizontalPOI3.y;
        rosMsg.facchory_3 = getScaledValue(admaMsg.accHorizontalPOI3.y, 0.0004);
        rosMsg.acchorz_3 = admaMsg.accHorizontalPOI3.z;
        rosMsg.facchorz_3 = getScaledValue(admaMsg.accHorizontalPOI3.z, 0.0004);
        rosMsg.acchorx_4 = admaMsg.accHorizontalPOI4.x;
        rosMsg.facchorx_4 = getScaledValue(admaMsg.accHorizontalPOI4.x, 0.0004);
        rosMsg.acchory_4 = admaMsg.accHorizontalPOI4.y;
        rosMsg.facchory_4 = getScaledValue(admaMsg.accHorizontalPOI4.y, 0.0004);
        rosMsg.acchorz_4 = admaMsg.accHorizontalPOI4.z;
        rosMsg.facchorz_4 = getScaledValue(admaMsg.accHorizontalPOI4.z, 0.0004);
        rosMsg.acchorx_5 = admaMsg.accHorizontalPOI5.x;
        rosMsg.facchorx_5 = getScaledValue(admaMsg.accHorizontalPOI5.x, 0.0004);
        rosMsg.acchory_5 = admaMsg.accHorizontalPOI5.y;
        rosMsg.facchory_5 = getScaledValue(admaMsg.accHorizontalPOI5.y, 0.0004);
        rosMsg.acchorz_5 = admaMsg.accHorizontalPOI5.z;
        rosMsg.facchorz_5 = getScaledValue(admaMsg.accHorizontalPOI5.z, 0.0004);
        rosMsg.acchorx_6 = admaMsg.accHorizontalPOI6.x;
        rosMsg.facchorx_6 = getScaledValue(admaMsg.accHorizontalPOI6.x, 0.0004);
        rosMsg.acchory_6 = admaMsg.accHorizontalPOI6.y;
        rosMsg.facchory_6 = getScaledValue(admaMsg.accHorizontalPOI6.y, 0.0004);
        rosMsg.acchorz_6 = admaMsg.accHorizontalPOI6.z;
        rosMsg.facchorz_6 = getScaledValue(admaMsg.accHorizontalPOI6.z, 0.0004);
        rosMsg.acchorx_7 = admaMsg.accHorizontalPOI7.x;
        rosMsg.facchorx_7 = getScaledValue(admaMsg.accHorizontalPOI7.x, 0.0004);
        rosMsg.acchory_7 = admaMsg.accHorizontalPOI7.y;
        rosMsg.facchory_7 = getScaledValue(admaMsg.accHorizontalPOI7.y, 0.0004);
        rosMsg.acchorz_7 = admaMsg.accHorizontalPOI7.z;
        rosMsg.facchorz_7 = getScaledValue(admaMsg.accHorizontalPOI7.z, 0.0004);

        //fill external velocity
        rosMsg.extvelanx = admaMsg.extVelAnalog.x;
        rosMsg.fextvelanx = getScaledValue(admaMsg.extVelAnalog.x, 0.005);
        rosMsg.extvelany = admaMsg.extVelAnalog.y;
        rosMsg.fextvelany = getScaledValue(admaMsg.extVelAnalog.y, 0.005);
        rosMsg.extveldigx = admaMsg.extveldigx;
        rosMsg.fextveldigx = getScaledValue(admaMsg.extveldigx, 0.005);
        rosMsg.extveldigy = admaMsg.extveldigy;
        rosMsg.fextveldigy = getScaledValue(admaMsg.extveldigy, 0.005);
        rosMsg.extveldigpulsesx = admaMsg.extveldigpulsesx;
        rosMsg.extveldigpulsesy = admaMsg.extveldigpulsesy;
        rosMsg.extvelxcorrected = admaMsg.extVelCorrected.x;
        rosMsg.fextvelxcorrected = getScaledValue(admaMsg.extVelCorrected.x, 0.005);
        rosMsg.extvelycorrected = admaMsg.extVelCorrected.y;
        rosMsg.fextvelycorrected = getScaledValue(admaMsg.extVelCorrected.y, 0.005);

        //fill barometer values
        rosMsg.extbaropressure = admaMsg.extbaropressure;
        rosMsg.fextbaropressure = getScaledValue(admaMsg.extbaropressure, 0.01);
        rosMsg.extbaroheight = admaMsg.extbaroheight;
        rosMsg.fextbaroheight = getScaledValue(admaMsg.extbaroheight, 0.01);
        rosMsg.extbaroheightcorrected = admaMsg.extbaroheightcorrected;
        rosMsg.fextbaroheightcorrected = getScaledValue(admaMsg.extbaroheightcorrected, 0.01);

        //fill miscellaneous
        rosMsg.invpathradius = admaMsg.misc.invPathRadius;
        rosMsg.finvpathradius = getScaledValue(admaMsg.misc.invPathRadius, 0.0001);
        rosMsg.sideslipangle = admaMsg.misc.sideSlipAngle;
        rosMsg.fsideslipangle = getScaledValue(admaMsg.misc.sideSlipAngle, 0.01);
        rosMsg.disttrav = admaMsg.misc.distanceTraveled;
        rosMsg.fdisttrav = getScaledValue(admaMsg.misc.distanceTraveled, 0.01);

        //fill miscellaneous POI
        rosMsg.invpathradius_1 = admaMsg.miscPOI1.invPathRadius;
        rosMsg.finvpathradius_1 = getScaledValue(admaMsg.miscPOI1.invPathRadius, 0.0001);
        rosMsg.sideslipangle_1 = admaMsg.miscPOI1.sideSlipAngle;
        rosMsg.fsideslipangle_1 = getScaledValue(admaMsg.miscPOI1.sideSlipAngle, 0.01);
        rosMsg.disttrav_1 = admaMsg.miscPOI1.distanceTraveled;
        rosMsg.fdisttrav_1 = getScaledValue(admaMsg.miscPOI1.distanceTraveled, 0.01);
        rosMsg.invpathradius_2 = admaMsg.miscPOI2.invPathRadius;
        rosMsg.finvpathradius_2 = getScaledValue(admaMsg.miscPOI2.invPathRadius, 0.0001);
        rosMsg.sideslipangle_2 = admaMsg.miscPOI2.sideSlipAngle;
        rosMsg.fsideslipangle_2 = getScaledValue(admaMsg.miscPOI2.sideSlipAngle, 0.01);
        rosMsg.disttrav_2 = admaMsg.miscPOI2.distanceTraveled;
        rosMsg.fdisttrav_2 = getScaledValue(admaMsg.miscPOI2.distanceTraveled, 0.01);
        rosMsg.invpathradius_3 = admaMsg.miscPOI3.invPathRadius;
        rosMsg.finvpathradius_3 = getScaledValue(admaMsg.miscPOI3.invPathRadius, 0.0001);
        rosMsg.sideslipangle_3 = admaMsg.miscPOI3.sideSlipAngle;
        rosMsg.fsideslipangle_3 = getScaledValue(admaMsg.miscPOI3.sideSlipAngle, 0.01);
        rosMsg.disttrav_3 = admaMsg.miscPOI3.distanceTraveled;
        rosMsg.fdisttrav_3 = getScaledValue(admaMsg.miscPOI3.distanceTraveled, 0.01);
        rosMsg.invpathradius_4 = admaMsg.miscPOI4.invPathRadius;
        rosMsg.finvpathradius_4 = getScaledValue(admaMsg.miscPOI4.invPathRadius, 0.0001);
        rosMsg.sideslipangle_4 = admaMsg.miscPOI4.sideSlipAngle;
        rosMsg.fsideslipangle_4 = getScaledValue(admaMsg.miscPOI4.sideSlipAngle, 0.01);
        rosMsg.disttrav_4 = admaMsg.miscPOI4.distanceTraveled;
        rosMsg.fdisttrav_4 = getScaledValue(admaMsg.miscPOI4.distanceTraveled, 0.01);
        rosMsg.invpathradius_5 = admaMsg.miscPOI5.invPathRadius;
        rosMsg.finvpathradius_5 = getScaledValue(admaMsg.miscPOI5.invPathRadius, 0.0001);
        rosMsg.sideslipangle_5 = admaMsg.miscPOI5.sideSlipAngle;
        rosMsg.fsideslipangle_5 = getScaledValue(admaMsg.miscPOI5.sideSlipAngle, 0.01);
        rosMsg.disttrav_5 = admaMsg.miscPOI5.distanceTraveled;
        rosMsg.fdisttrav_5 = getScaledValue(admaMsg.miscPOI5.distanceTraveled, 0.01);
        rosMsg.invpathradius_6 = admaMsg.miscPOI6.invPathRadius;
        rosMsg.finvpathradius_6 = getScaledValue(admaMsg.miscPOI6.invPathRadius, 0.0001);
        rosMsg.sideslipangle_6 = admaMsg.miscPOI6.sideSlipAngle;
        rosMsg.fsideslipangle_6 = getScaledValue(admaMsg.miscPOI6.sideSlipAngle, 0.01);
        rosMsg.disttrav_6 = admaMsg.miscPOI6.distanceTraveled;
        rosMsg.fdisttrav_6 = getScaledValue(admaMsg.miscPOI6.distanceTraveled, 0.01);
        rosMsg.invpathradius_7 = admaMsg.miscPOI7.invPathRadius;
        rosMsg.finvpathradius_7 = getScaledValue(admaMsg.miscPOI7.invPathRadius, 0.0001);
        rosMsg.sideslipangle_7 = admaMsg.miscPOI7.sideSlipAngle;
        rosMsg.fsideslipangle_7 = getScaledValue(admaMsg.miscPOI7.sideSlipAngle, 0.01);
        rosMsg.disttrav_7 = admaMsg.miscPOI7.distanceTraveled;
        rosMsg.fdisttrav_7 = getScaledValue(admaMsg.miscPOI7.distanceTraveled, 0.01);

        // fill triggers
        rosMsg.trigrising1 = admaMsg.trigrising1;
        rosMsg.trigfalling1 = admaMsg.trigfalling1;
        rosMsg.trigrising2 = admaMsg.trigrising2;
        rosMsg.trigfalling2 = admaMsg.trigfalling2;
        rosMsg.trigrising3 = admaMsg.trigrising3;
        rosMsg.trigfalling3 = admaMsg.trigfalling3;
        rosMsg.trigrising4 = admaMsg.trigrising4;
        rosMsg.trigfalling4 = admaMsg.trigfalling4;

        //fill system data
        rosMsg.systemta = admaMsg.systemta;
        rosMsg.systemtemp = admaMsg.systemtemp;
        rosMsg.fsystemtemp = getScaledValue(admaMsg.systemtemp, 0.1);
        rosMsg.systemtimesinceinit = admaMsg.systemtimesinceinit;
        rosMsg.systemdspload = admaMsg.systemdspload;
        rosMsg.fsystemdspload = getScaledValue(admaMsg.systemdspload, 0.1);

        //fill GPS position
        rosMsg.gpslatabs = admaMsg.posAbs.latitude;
        rosMsg.fgpslatabs = getScaledValue(admaMsg.posAbs.latitude, 0.0000001);
        rosMsg.gpslonabs = admaMsg.posAbs.longitude;
        rosMsg.fgpslonabs = getScaledValue(admaMsg.posAbs.longitude, 0.0000001);
        rosMsg.gpslatrel = admaMsg.posRel.latitude;
        rosMsg.fgpslatrel = getScaledValue(admaMsg.posRel.latitude, 0.01);
        rosMsg.gpslonrel = admaMsg.posRel.longitude;
        rosMsg.fgpslonrel = getScaledValue(admaMsg.posRel.longitude, 0.01);

        // fill GPS Expected Position Error
        rosMsg.gpsstddevlat = admaMsg.gpsstddevlat;
        rosMsg.fgpsstddevlat = getScaledValue(admaMsg.gpsstddevlat, 0.001);
        rosMsg.gpsstddevlon = admaMsg.gpsstddevlon;
        rosMsg.fgpsstddevlon = getScaledValue(admaMsg.gpsstddevlon, 0.001);
        rosMsg.gpsstddevheight = admaMsg.gpsstddevheight;
        rosMsg.fgpsstddevheight = getScaledValue(admaMsg.gpsstddevheight, 0.001);

        //fill GPS Velocity
        rosMsg.gpsvelframex = admaMsg.gpsvelframex;
        rosMsg.fgpsvelframex = getScaledValue(admaMsg.gpsvelframex, 0.005);
        rosMsg.gpsvelframey = admaMsg.gpsvelframey;
        rosMsg.fgpsvelframey = getScaledValue(admaMsg.gpsvelframey, 0.005);
        rosMsg.gpsvelframez = admaMsg.gpsvelframez;
        rosMsg.fgpsvelframez = getScaledValue(admaMsg.gpsvelframez, 0.005);
        rosMsg.gpsvellatency = admaMsg.gpsvellatency;
        rosMsg.fgpsvellatency = getScaledValue(admaMsg.gpsvellatency, 0.001);

        //fill GPS Expected Velocity error
        rosMsg.gpsstddevvelx = admaMsg.gpsStdDevVel.x;
        rosMsg.fgpsstddevvelx = getScaledValue(admaMsg.gpsStdDevVel.x, 0.001);
        rosMsg.gpsstddevvely = admaMsg.gpsStdDevVel.y;
        rosMsg.fgpsstddevvely = getScaledValue(admaMsg.gpsStdDevVel.y, 0.001);
        rosMsg.gpsstddevvelz = admaMsg.gpsStdDevVel.z;
        rosMsg.fgpsstddevvelz = getScaledValue(admaMsg.gpsStdDevVel.z, 0.001);

        //fill GPS Time
        rosMsg.gpstimemsec = admaMsg.gpstimemsec;
        rosMsg.gpstimeweek = admaMsg.gpstimeweek;
        rosMsg.gpstrigger = admaMsg.gpstrigger;

        //fill GPS AUX data
        rosMsg.gpsdiffage = admaMsg.gpsdiffage;
        rosMsg.fgpsdiffage = getScaledValue(admaMsg.gpsdiffage, 0.1);
        rosMsg.gpssatsused = admaMsg.gpssatsused;
        rosMsg.gpssatsvisible = admaMsg.gpssatsvisible;
        rosMsg.gpslogdelay = admaMsg.gpslogdelay;
        rosMsg.gpsreceiverload = admaMsg.gpsreceiverload;
        rosMsg.fgpsreceiverload = getScaledValue(admaMsg.gpsreceiverload, 0.5);
        // rosMsg.gpsbasenr = admaMsg.gpsbasenr; //TODO: incompatible..

        //fill INS Angle and GPS COG
        rosMsg.insroll = admaMsg.insroll;
        rosMsg.finsroll = getScaledValue(admaMsg.insroll, 0.01);
        rosMsg.inspitch = admaMsg.inspitch;
        rosMsg.finspitch = getScaledValue(admaMsg.inspitch, 0.01);
        rosMsg.insyaw = admaMsg.insyaw;
        rosMsg.finsyaw = getScaledValue(admaMsg.insyaw, 0.01);
        rosMsg.gpscog = admaMsg.gpscog;
        rosMsg.fgpscog = getScaledValue(admaMsg.gpscog, 0.01);

        //fill GPS Height MSL
        rosMsg.gpsheight = admaMsg.gpsheight;
        rosMsg.fgpsheight = getScaledValue(admaMsg.gpsheight, 0.01);
        rosMsg.undulation = admaMsg.undulation;
        rosMsg.fundulation = getScaledValue(admaMsg.undulation, 0.01);

        //fill INS height MSL (+ POI)
        rosMsg.insheight = admaMsg.insHeight;
        rosMsg.finsheight = getScaledValue(admaMsg.insHeight, 0.01);
        rosMsg.insheight_1 = admaMsg.insHeightPOI1;
        rosMsg.finsheight_1 = getScaledValue(admaMsg.insHeightPOI1, 0.01);
        rosMsg.insheight_2 = admaMsg.insHeightPOI2;
        rosMsg.finsheight_2 = getScaledValue(admaMsg.insHeightPOI2, 0.01);
        rosMsg.insheight_3 = admaMsg.insHeightPOI3;
        rosMsg.finsheight_3 = getScaledValue(admaMsg.insHeightPOI3, 0.01);
        rosMsg.insheight_4 = admaMsg.insHeightPOI4;
        rosMsg.finsheight_4 = getScaledValue(admaMsg.insHeightPOI4, 0.01);
        rosMsg.insheight_5 = admaMsg.insHeightPOI5;
        rosMsg.finsheight_5 = getScaledValue(admaMsg.insHeightPOI5, 0.01);
        rosMsg.insheight_6 = admaMsg.insHeightPOI6;
        rosMsg.finsheight_6 = getScaledValue(admaMsg.insHeightPOI6, 0.01);
        rosMsg.insheight_7 = admaMsg.insHeightPOI7;
        rosMsg.finsheight_7 = getScaledValue(admaMsg.insHeightPOI7, 0.01);

        //fill INS time UTC
        rosMsg.instimemsec = admaMsg.instimemsec;
        rosMsg.instimeweek = admaMsg.instimeweek;
        rosMsg.leapseconds = admaMsg.leapseconds;

        //fill INS Position (+POI)
        rosMsg.inslatabs = admaMsg.insPosAbs.latitude;
        rosMsg.finslatabs = getScaledValue(admaMsg.insPosAbs.latitude, 0.0000001);
        rosMsg.inslonabs = admaMsg.insPosAbs.longitude;
        rosMsg.finslonabs = getScaledValue(admaMsg.insPosAbs.longitude, 0.0000001);
        rosMsg.inslatrel = admaMsg.insPosRel.latitude;
        rosMsg.finslatrel = getScaledValue(admaMsg.insPosRel.latitude, 0.01);
        rosMsg.inslonrel = admaMsg.insPosRel.longitude;
        rosMsg.finslonrel = getScaledValue(admaMsg.insPosRel.longitude, 0.01);
        rosMsg.inslatabs_1 = admaMsg.insPosAbsPOI1.latitude;
        rosMsg.finslatabs_1 = getScaledValue(admaMsg.insPosAbsPOI1.latitude, 0.0000001);
        rosMsg.inslonabs_1 = admaMsg.insPosAbsPOI1.longitude;
        rosMsg.finslonabs_1 = getScaledValue(admaMsg.insPosAbsPOI1.longitude, 0.0000001);
        rosMsg.inslatrel_1 = admaMsg.insPosRelPOI1.latitude;
        rosMsg.finslatrel_1 = getScaledValue(admaMsg.insPosRelPOI1.latitude, 0.01);
        rosMsg.inslonrel_1 = admaMsg.insPosRelPOI1.longitude;
        rosMsg.finslonrel_1 = getScaledValue(admaMsg.insPosRelPOI1.longitude, 0.01);
        rosMsg.inslatabs_2 = admaMsg.insPosAbsPOI2.latitude;
        rosMsg.finslatabs_2 = getScaledValue(admaMsg.insPosAbsPOI2.latitude, 0.0000001);
        rosMsg.inslonabs_2 = admaMsg.insPosAbsPOI2.longitude;
        rosMsg.finslonabs_2 = getScaledValue(admaMsg.insPosAbsPOI2.longitude, 0.0000001);
        rosMsg.inslatrel_2 = admaMsg.insPosRelPOI2.latitude;
        rosMsg.finslatrel_2 = getScaledValue(admaMsg.insPosRelPOI2.latitude, 0.01);
        rosMsg.inslonrel_2 = admaMsg.insPosRelPOI2.longitude;
        rosMsg.finslonrel_2 = getScaledValue(admaMsg.insPosRelPOI2.longitude, 0.01);
        rosMsg.inslatabs_3 = admaMsg.insPosAbsPOI3.latitude;
        rosMsg.finslatabs_3 = getScaledValue(admaMsg.insPosAbsPOI3.latitude, 0.0000001);
        rosMsg.inslonabs_3 = admaMsg.insPosAbsPOI3.longitude;
        rosMsg.finslonabs_3 = getScaledValue(admaMsg.insPosAbsPOI3.longitude, 0.0000001);
        rosMsg.inslatrel_3 = admaMsg.insPosRelPOI3.latitude;
        rosMsg.finslatrel_3 = getScaledValue(admaMsg.insPosRelPOI3.latitude, 0.01);
        rosMsg.inslonrel_3 = admaMsg.insPosRelPOI3.longitude;
        rosMsg.finslonrel_3 = getScaledValue(admaMsg.insPosRelPOI3.longitude, 0.01);
        rosMsg.inslatabs_4 = admaMsg.insPosAbsPOI4.latitude;
        rosMsg.finslatabs_4 = getScaledValue(admaMsg.insPosAbsPOI4.latitude, 0.0000001);
        rosMsg.inslonabs_4 = admaMsg.insPosAbsPOI4.longitude;
        rosMsg.finslonabs_4 = getScaledValue(admaMsg.insPosAbsPOI4.longitude, 0.0000001);
        rosMsg.inslatrel_4 = admaMsg.insPosRelPOI4.latitude;
        rosMsg.finslatrel_4 = getScaledValue(admaMsg.insPosRelPOI4.latitude, 0.01);
        rosMsg.inslonrel_4 = admaMsg.insPosRelPOI4.longitude;
        rosMsg.finslonrel_4 = getScaledValue(admaMsg.insPosRelPOI4.longitude, 0.01);
        rosMsg.inslatabs_5 = admaMsg.insPosAbsPOI5.latitude;
        rosMsg.finslatabs_5 = getScaledValue(admaMsg.insPosAbsPOI5.latitude, 0.0000001);
        rosMsg.inslonabs_5 = admaMsg.insPosAbsPOI5.longitude;
        rosMsg.finslonabs_5 = getScaledValue(admaMsg.insPosAbsPOI5.longitude, 0.0000001);
        rosMsg.inslatrel_5 = admaMsg.insPosRelPOI5.latitude;
        rosMsg.finslatrel_5 = getScaledValue(admaMsg.insPosRelPOI5.latitude, 0.01);
        rosMsg.inslonrel_5 = admaMsg.insPosRelPOI5.longitude;
        rosMsg.finslonrel_5 = getScaledValue(admaMsg.insPosRelPOI5.longitude, 0.01);
        rosMsg.inslatabs_6 = admaMsg.insPosAbsPOI6.latitude;
        rosMsg.finslatabs_6 = getScaledValue(admaMsg.insPosAbsPOI6.latitude, 0.0000001);
        rosMsg.inslonabs_6 = admaMsg.insPosAbsPOI6.longitude;
        rosMsg.finslonabs_6 = getScaledValue(admaMsg.insPosAbsPOI6.longitude, 0.0000001);
        rosMsg.inslatrel_6 = admaMsg.insPosRelPOI6.latitude;
        rosMsg.finslatrel_6 = getScaledValue(admaMsg.insPosRelPOI6.latitude, 0.01);
        rosMsg.inslonrel_6 = admaMsg.insPosRelPOI6.longitude;
        rosMsg.finslonrel_6 = getScaledValue(admaMsg.insPosRelPOI6.longitude, 0.01);
        rosMsg.inslatabs_7 = admaMsg.insPosAbsPOI7.latitude;
        rosMsg.finslatabs_7 = getScaledValue(admaMsg.insPosAbsPOI7.latitude, 0.0000001);
        rosMsg.inslonabs_7 = admaMsg.insPosAbsPOI7.longitude;
        rosMsg.finslonabs_7 = getScaledValue(admaMsg.insPosAbsPOI7.longitude, 0.0000001);
        rosMsg.inslatrel_7 = admaMsg.insPosRelPOI7.latitude;
        rosMsg.finslatrel_7 = getScaledValue(admaMsg.insPosRelPOI7.latitude, 0.01);
        rosMsg.inslonrel_7 = admaMsg.insPosRelPOI7.longitude;
        rosMsg.finslonrel_7 = getScaledValue(admaMsg.insPosRelPOI7.longitude, 0.01);

        //fill ins velocity (horizontal + frame)
        rosMsg.insvelhorx = admaMsg.insVelHor.x;
        rosMsg.finsvelhorx = getScaledValue(admaMsg.insVelHor.x, 0.005);
        rosMsg.insvelhory = admaMsg.insVelHor.y;
        rosMsg.finsvelhory = getScaledValue(admaMsg.insVelHor.y, 0.005);
        rosMsg.insvelhorz = admaMsg.insVelHor.z;
        rosMsg.finsvelhorz = getScaledValue(admaMsg.insVelHor.z, 0.005);
        rosMsg.insvelframex = admaMsg.insVelFrame.x;
        rosMsg.finsvelframex = getScaledValue(admaMsg.insVelFrame.x, 0.005);
        rosMsg.insvelframey = admaMsg.insVelFrame.y;
        rosMsg.finsvelframey = getScaledValue(admaMsg.insVelFrame.y, 0.005);
        rosMsg.insvelframez = admaMsg.insVelFrame.z;
        rosMsg.finsvelframez = getScaledValue(admaMsg.insVelFrame.z, 0.005);

        //fill INS velocity (POI)
        rosMsg.insvelhorx_1 = admaMsg.insVelHorPOI1.x;
        rosMsg.finsvelhorx_1 = getScaledValue(admaMsg.insVelHorPOI1.x, 0.005);
        rosMsg.insvelhory_1 = admaMsg.insVelHorPOI1.y;
        rosMsg.finsvelhory_1 = getScaledValue(admaMsg.insVelHorPOI1.y, 0.005);
        rosMsg.insvelhorz_1 = admaMsg.insVelHorPOI1.z;
        rosMsg.finsvelhorz_1 = getScaledValue(admaMsg.insVelHorPOI1.z, 0.005);
        rosMsg.insvelhorx_2 = admaMsg.insVelHorPOI2.x;
        rosMsg.finsvelhorx_2 = getScaledValue(admaMsg.insVelHorPOI2.x, 0.005);
        rosMsg.insvelhory_2 = admaMsg.insVelHorPOI2.y;
        rosMsg.finsvelhory_2 = getScaledValue(admaMsg.insVelHorPOI2.y, 0.005);
        rosMsg.insvelhorz_2 = admaMsg.insVelHorPOI2.z;
        rosMsg.finsvelhorz_2 = getScaledValue(admaMsg.insVelHorPOI2.z, 0.005);
        rosMsg.insvelhorx_3 = admaMsg.insVelHorPOI3.x;
        rosMsg.finsvelhorx_3 = getScaledValue(admaMsg.insVelHorPOI3.x, 0.005);
        rosMsg.insvelhory_3 = admaMsg.insVelHorPOI3.y;
        rosMsg.finsvelhory_3 = getScaledValue(admaMsg.insVelHorPOI3.y, 0.005);
        rosMsg.insvelhorz_3 = admaMsg.insVelHorPOI3.z;
        rosMsg.finsvelhorz_3 = getScaledValue(admaMsg.insVelHorPOI3.z, 0.005);
        rosMsg.insvelhorx_4 = admaMsg.insVelHorPOI4.x;
        rosMsg.finsvelhorx_4 = getScaledValue(admaMsg.insVelHorPOI4.x, 0.005);
        rosMsg.insvelhory_4 = admaMsg.insVelHorPOI4.y;
        rosMsg.finsvelhory_4 = getScaledValue(admaMsg.insVelHorPOI4.y, 0.005);
        rosMsg.insvelhorz_4 = admaMsg.insVelHorPOI4.z;
        rosMsg.finsvelhorz_4 = getScaledValue(admaMsg.insVelHorPOI4.z, 0.005);
        rosMsg.insvelhorx_5 = admaMsg.insVelHorPOI5.x;
        rosMsg.finsvelhorx_5 = getScaledValue(admaMsg.insVelHorPOI5.x, 0.005);
        rosMsg.insvelhory_5 = admaMsg.insVelHorPOI5.y;
        rosMsg.finsvelhory_5 = getScaledValue(admaMsg.insVelHorPOI5.y, 0.005);
        rosMsg.insvelhorz_5 = admaMsg.insVelHorPOI5.z;
        rosMsg.finsvelhorz_5 = getScaledValue(admaMsg.insVelHorPOI5.z, 0.005);
        rosMsg.insvelhorx_6 = admaMsg.insVelHorPOI6.x;
        rosMsg.finsvelhorx_6 = getScaledValue(admaMsg.insVelHorPOI6.x, 0.005);
        rosMsg.insvelhory_6 = admaMsg.insVelHorPOI6.y;
        rosMsg.finsvelhory_6 = getScaledValue(admaMsg.insVelHorPOI6.y, 0.005);
        rosMsg.insvelhorz_6 = admaMsg.insVelHorPOI6.z;
        rosMsg.finsvelhorz_6 = getScaledValue(admaMsg.insVelHorPOI6.z, 0.005);
        rosMsg.insvelhorx_7 = admaMsg.insVelHorPOI7.x;
        rosMsg.finsvelhorx_7 = getScaledValue(admaMsg.insVelHorPOI7.x, 0.005);
        rosMsg.insvelhory_7 = admaMsg.insVelHorPOI7.y;
        rosMsg.finsvelhory_7 = getScaledValue(admaMsg.insVelHorPOI7.y, 0.005);
        rosMsg.insvelhorz_7 = admaMsg.insVelHorPOI7.z;
        rosMsg.finsvelhorz_7 = getScaledValue(admaMsg.insVelHorPOI7.z, 0.005);

        //fill INS Expected Position Error
        rosMsg.insstddevlat = admaMsg.insstddevlat;
        rosMsg.finsstddevlat = getScaledValue(admaMsg.insstddevlat, 0.01);
        rosMsg.insstddevlong = admaMsg.insstddevlong;
        rosMsg.finsstddevlong = getScaledValue(admaMsg.insstddevlong, 0.01);
        rosMsg.insstddevheight = admaMsg.insstddevheight;

        //fill INS EVE and INS ETE
        rosMsg.insstddevvelx = admaMsg.insstddevvelx;
        rosMsg.finsstddevvelx = getScaledValue(admaMsg.insstddevvelx, 0.01);
        rosMsg.insstddevvely = admaMsg.insstddevvely;
        rosMsg.finsstddevvely = getScaledValue(admaMsg.insstddevvely, 0.01);
        rosMsg.insstddevvelz = admaMsg.insstddevvelz;
        rosMsg.finsstddevvelz = getScaledValue(admaMsg.insstddevvelz, 0.01);
        rosMsg.insstddevroll = admaMsg.insstddevroll;
        rosMsg.finsstddevroll = getScaledValue(admaMsg.insstddevroll, 0.01);
        rosMsg.insstddevpitch = admaMsg.insstddevpitch;
        rosMsg.finsstddevpitch = getScaledValue(admaMsg.insstddevpitch, 0.01);
        rosMsg.insstddevyaw = admaMsg.insstddevyaw;
        rosMsg.finsstddevyaw = getScaledValue(admaMsg.insstddevyaw, 0.01);

        //fill Analog in 1
        rosMsg.an1 = admaMsg.an1;
        rosMsg.fan1 = getScaledValue(admaMsg.an1, 0.0005);
        rosMsg.an2 = admaMsg.an2;
        rosMsg.fan2 = getScaledValue(admaMsg.an2, 0.0005);
        rosMsg.an3 = admaMsg.an3;
        rosMsg.fan3 = getScaledValue(admaMsg.an3, 0.0005);
        rosMsg.an4 = admaMsg.an4;
        rosMsg.fan4 = getScaledValue(admaMsg.an4, 0.0005);
}

/// \file
/// \brief  getstatusgps function - adma status information
/// \param  local_data adma string
/// \param  message adma message to be loaded
void getstatusgps(adma_msgs::msg::AdmaData& rosMsg, AdmaDataV32& admaMsg)
{
    bool status_external_vel = getbit(admaMsg.gpsStatus,7);
    bool status_skidding = getbit(admaMsg.gpsStatus,5);
    bool standstill_c = getbit(admaMsg.gpsStatus,4);
    bool rtk_precise = getbit(admaMsg.gpsStatus,3);
    bool rtk_coarse = getbit(admaMsg.gpsStatus,2);
    bool gps_mode = getbit(admaMsg.gpsStatus,1);
    bool gps_out = getbit(admaMsg.gpsStatus,0);

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
/// \param  local_data adma string
/// \param  message adma message to be loaded
void getstatustrigger(adma_msgs::msg::AdmaData& rosMsg, AdmaDataV32& admaMsg)
{
    bool status_synclock = getbit(admaMsg.gpsTriggerStatus,7);
    bool status_dead_reckoning = getbit(admaMsg.gpsTriggerStatus,6);
    bool status_ahrs_ins = getbit(admaMsg.gpsTriggerStatus,5);
    bool status_alignment = getbit(admaMsg.gpsTriggerStatus,4);
    bool status_signal_in1 = getbit(admaMsg.gpsTriggerStatus,3);
    bool status_signal_in2 = getbit(admaMsg.gpsTriggerStatus,2);
    bool status_signal_in3 = getbit(admaMsg.gpsTriggerStatus,1);
    bool status_trig_gps = getbit(admaMsg.gpsTriggerStatus,0);
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
/// \param  local_data adma string
/// \param  message adma message to be loaded
void getevkstatus(adma_msgs::msg::AdmaData& rosMsg, AdmaDataV32& admaMsg)
{
    bool status_pos_b2 = getbit(admaMsg.evkStatus,7);
    bool status_pos_b1 = getbit(admaMsg.evkStatus,6);
    bool status_tilt_b2 = getbit(admaMsg.evkStatus,5);
    bool status_tilt_b1 = getbit(admaMsg.evkStatus,4);
    bool status_configuration_changed = getbit(admaMsg.evkStatus,3);
    bool status_heading_executed = getbit(admaMsg.evkStatus,2);
    bool status_evk_estimates = getbit(admaMsg.evkStatus,1);
    bool status_evk_activ = getbit(admaMsg.evkStatus,0);
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
/// \param  local_data adma string
/// \param  message adma message to be loaded
void geterrorandwarning(adma_msgs::msg::AdmaData& rosMsg, AdmaDataV32& admaMsg)
{
    std::bitset<8> bitdataerror1 = admaMsg.dataError1;
    std::bitset<8> bitdataerror2 = admaMsg.dataError2;
    std::bitset<8> bitdatawarn3 = admaMsg.dataWarn1;
    std::bitset<8> errorhw = admaMsg.dataErrorHW;
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