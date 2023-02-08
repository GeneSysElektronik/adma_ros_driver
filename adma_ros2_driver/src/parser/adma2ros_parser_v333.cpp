#include "adma_ros2_driver/parser/adma2ros_parser_v333.hpp"

#include "adma_ros2_driver/parser/parser_utils.hpp"

ADMA2ROSParserV333::ADMA2ROSParserV333() {}

ADMA2ROSParserV333::~ADMA2ROSParserV333() {}

void ADMA2ROSParserV333::mapAdmaMessageToROS(
  adma_ros_driver_msgs::msg::AdmaData & ros_msg, AdmaDataV333 & adma_data)
{
  ros_msg.statuscount = adma_data.statuscount;

  //fill sensor bodies
  ros_msg.accbodyhrx = adma_data.sensorsBodyX.accHR;
  ros_msg.ratebodyhrx = adma_data.sensorsBodyX.rateHR;
  ros_msg.accbodyhry = adma_data.sensorsBodyY.accHR;
  ros_msg.ratebodyhry = adma_data.sensorsBodyY.rateHR;
  ros_msg.accbodyhrz = adma_data.sensorsBodyZ.accHR;
  ros_msg.ratebodyhrz = adma_data.sensorsBodyZ.rateHR;

  //fill rates
  ros_msg.ratebodyx = adma_data.ratesBody.x;
  ros_msg.ratebodyy = adma_data.ratesBody.y;
  ros_msg.ratebodyz = adma_data.ratesBody.z;
  ros_msg.ratehorx = adma_data.ratesHorizontal.x;
  ros_msg.ratehory = adma_data.ratesHorizontal.y;
  ros_msg.ratehorz = adma_data.ratesHorizontal.z;

  //fill accelerations
  ros_msg.accbodyx = adma_data.accBody.x;
  ros_msg.accbodyy = adma_data.accBody.y;
  ros_msg.accbodyz = adma_data.accBody.z;
  ros_msg.acchorx = adma_data.accHorizontal.x;
  ros_msg.acchory = adma_data.accHorizontal.y;
  ros_msg.acchorz = adma_data.accHorizontal.z;

  //fill POI accelerations
  ros_msg.accbodyx_1 = adma_data.accBodyPOI1.x;
  ros_msg.accbodyy_1 = adma_data.accBodyPOI1.y;
  ros_msg.accbodyz_1 = adma_data.accBodyPOI1.z;
  ros_msg.accbodyx_2 = adma_data.accBodyPOI2.x;
  ros_msg.accbodyy_2 = adma_data.accBodyPOI2.y;
  ros_msg.accbodyz_2 = adma_data.accBodyPOI2.z;
  ros_msg.accbodyx_3 = adma_data.accBodyPOI3.x;
  ros_msg.accbodyy_3 = adma_data.accBodyPOI3.y;
  ros_msg.accbodyz_3 = adma_data.accBodyPOI3.z;
  ros_msg.accbodyx_4 = adma_data.accBodyPOI4.x;
  ros_msg.accbodyy_4 = adma_data.accBodyPOI4.y;
  ros_msg.accbodyz_4 = adma_data.accBodyPOI4.z;
  ros_msg.accbodyx_5 = adma_data.accBodyPOI5.x;
  ros_msg.accbodyy_5 = adma_data.accBodyPOI5.y;
  ros_msg.accbodyz_5 = adma_data.accBodyPOI5.z;
  ros_msg.accbodyx_6 = adma_data.accBodyPOI6.x;
  ros_msg.accbodyy_6 = adma_data.accBodyPOI6.y;
  ros_msg.accbodyz_6 = adma_data.accBodyPOI6.z;
  ros_msg.accbodyx_7 = adma_data.accBodyPOI7.x;
  ros_msg.accbodyy_7 = adma_data.accBodyPOI7.y;
  ros_msg.accbodyz_7 = adma_data.accBodyPOI7.z;
  ros_msg.accbodyx_8 = adma_data.accBodyPOI8.x;
  ros_msg.accbodyy_8 = adma_data.accBodyPOI8.y;
  ros_msg.accbodyz_8 = adma_data.accBodyPOI8.z;

  ros_msg.acchorx_1 = adma_data.accHorizontalPOI1.x;
  ros_msg.acchory_1 = adma_data.accHorizontalPOI1.y;
  ros_msg.acchorz_1 = adma_data.accHorizontalPOI1.z;
  ros_msg.acchorx_2 = adma_data.accHorizontalPOI2.x;
  ros_msg.acchory_2 = adma_data.accHorizontalPOI2.y;
  ros_msg.acchorz_2 = adma_data.accHorizontalPOI2.z;
  ros_msg.acchorx_3 = adma_data.accHorizontalPOI3.x;
  ros_msg.acchory_3 = adma_data.accHorizontalPOI3.y;
  ros_msg.acchorz_3 = adma_data.accHorizontalPOI3.z;
  ros_msg.acchorx_4 = adma_data.accHorizontalPOI4.x;
  ros_msg.acchory_4 = adma_data.accHorizontalPOI4.y;
  ros_msg.acchorz_4 = adma_data.accHorizontalPOI4.z;
  ros_msg.acchorx_5 = adma_data.accHorizontalPOI5.x;
  ros_msg.acchory_5 = adma_data.accHorizontalPOI5.y;
  ros_msg.acchorz_5 = adma_data.accHorizontalPOI5.z;
  ros_msg.acchorx_6 = adma_data.accHorizontalPOI6.x;
  ros_msg.acchory_6 = adma_data.accHorizontalPOI6.y;
  ros_msg.acchorz_6 = adma_data.accHorizontalPOI6.z;
  ros_msg.acchorx_7 = adma_data.accHorizontalPOI7.x;
  ros_msg.acchory_7 = adma_data.accHorizontalPOI7.y;
  ros_msg.acchorz_7 = adma_data.accHorizontalPOI7.z;
  ros_msg.acchorx_8 = adma_data.accHorizontalPOI8.x;
  ros_msg.acchory_8 = adma_data.accHorizontalPOI8.y;
  ros_msg.acchorz_8 = adma_data.accHorizontalPOI8.z;

  //fill external velocity
  ros_msg.extvelanx = adma_data.extVelAnalog.x;
  ros_msg.extvelany = adma_data.extVelAnalog.y;
  ros_msg.extveldigx = adma_data.extveldigx;
  ros_msg.extveldigy = adma_data.extveldigy;
  ros_msg.extveldigpulsesx = adma_data.extveldigpulsesx;
  ros_msg.extveldigpulsesy = adma_data.extveldigpulsesy;
  ros_msg.extvelxcorrected = adma_data.extVelCorrected.x;
  ros_msg.extvelycorrected = adma_data.extVelCorrected.y;

  //fill miscellaneous
  ros_msg.invpathradius = adma_data.misc.invPathRadius;
  ros_msg.sideslipangle = adma_data.misc.sideSlipAngle;
  ros_msg.disttrav = adma_data.misc.distanceTraveled;

  //fill miscellaneous POI
  ros_msg.invpathradius_1 = adma_data.miscPOI1.invPathRadius;
  ros_msg.sideslipangle_1 = adma_data.miscPOI1.sideSlipAngle;
  ros_msg.disttrav_1 = adma_data.miscPOI1.distanceTraveled;
  ros_msg.invpathradius_2 = adma_data.miscPOI2.invPathRadius;
  ros_msg.sideslipangle_2 = adma_data.miscPOI2.sideSlipAngle;
  ros_msg.disttrav_2 = adma_data.miscPOI2.distanceTraveled;
  ros_msg.invpathradius_3 = adma_data.miscPOI3.invPathRadius;
  ros_msg.sideslipangle_3 = adma_data.miscPOI3.sideSlipAngle;
  ros_msg.disttrav_3 = adma_data.miscPOI3.distanceTraveled;
  ros_msg.invpathradius_4 = adma_data.miscPOI4.invPathRadius;
  ros_msg.sideslipangle_4 = adma_data.miscPOI4.sideSlipAngle;
  ros_msg.disttrav_4 = adma_data.miscPOI4.distanceTraveled;
  ros_msg.invpathradius_5 = adma_data.miscPOI5.invPathRadius;
  ros_msg.sideslipangle_5 = adma_data.miscPOI5.sideSlipAngle;
  ros_msg.disttrav_5 = adma_data.miscPOI5.distanceTraveled;
  ros_msg.invpathradius_6 = adma_data.miscPOI6.invPathRadius;
  ros_msg.sideslipangle_6 = adma_data.miscPOI6.sideSlipAngle;
  ros_msg.disttrav_6 = adma_data.miscPOI6.distanceTraveled;
  ros_msg.invpathradius_7 = adma_data.miscPOI7.invPathRadius;
  ros_msg.sideslipangle_7 = adma_data.miscPOI7.sideSlipAngle;
  ros_msg.disttrav_7 = adma_data.miscPOI7.distanceTraveled;
  ros_msg.invpathradius_8 = adma_data.miscPOI8.invPathRadius;
  ros_msg.sideslipangle_8 = adma_data.miscPOI8.sideSlipAngle;
  ros_msg.disttrav_8 = adma_data.miscPOI8.distanceTraveled;

  // fill triggers
  ros_msg.trigrising1 = adma_data.trigrising1;
  ros_msg.trigfalling1 = adma_data.trigfalling1;
  ros_msg.trigrising2 = adma_data.trigrising2;
  ros_msg.trigfalling2 = adma_data.trigfalling2;
  ros_msg.trigrising3 = adma_data.trigrising3;
  ros_msg.trigfalling3 = adma_data.trigfalling3;
  ros_msg.trigrising4 = adma_data.trigrising4;
  ros_msg.trigfalling4 = adma_data.trigfalling4;

  //fill system data
  ros_msg.systemta = adma_data.systemta;
  ros_msg.systemtemp = adma_data.systemtemp;
  ros_msg.systemtimesinceinit = adma_data.systemtimesinceinit;
  ros_msg.systemdspload = adma_data.systemdspload;

  //fill GPS position
  ros_msg.gpslatabs = adma_data.posAbs.latitude;
  ros_msg.gpslonabs = adma_data.posAbs.longitude;
  ros_msg.gpslatrel = adma_data.posRel.longitude;
  ros_msg.gpslonrel = adma_data.posRel.latitude;

  // fill GPS Expected Position Error
  ros_msg.gpsstddevlat = adma_data.gnssstddevlat;
  ros_msg.gpsstddevlon = adma_data.gnssstddevlon;
  ros_msg.gpsstddevheight = adma_data.gnssstddevheight;

  //fill GPS Velocity
  ros_msg.gpsvelframex = adma_data.gnssvelframex;
  ros_msg.gpsvelframey = adma_data.gnssvelframey;
  ros_msg.gpsvelframez = adma_data.gnssvelframez;
  ros_msg.gpsvellatency = adma_data.gnssvellatency;

  //fill GPS Expected Velocity error
  ros_msg.gpsstddevvelx = adma_data.gnssStdDevVel.x;
  ros_msg.gpsstddevvely = adma_data.gnssStdDevVel.y;
  ros_msg.gpsstddevvelz = adma_data.gnssStdDevVel.z;

  //fill GPS Time
  ros_msg.gpstimemsec = adma_data.gnsstimemsec;
  ros_msg.gpstimeweek = adma_data.gnsstimeweek;
  ros_msg.gpstrigger = adma_data.gnsstrigger;

  //fill GPS AUX data
  ros_msg.gpsdiffage = adma_data.gnssdiffage;
  ros_msg.gpssatsused = adma_data.gnsssatsused;
  ros_msg.gpssatsvisible = adma_data.gnsssatsvisible;
  ros_msg.gpssatsdualantused = adma_data.gnsssatsdualantused;
  ros_msg.gpssatsdualantvisible = adma_data.gnsssatsdualantvisible;
  ros_msg.gpslogdelay = adma_data.gnsslogdelay;
  ros_msg.gpsreceiverload = adma_data.gnssreceiverload;
  std::stringstream ss;
  ss << adma_data.gnssbasenr;
  ros_msg.gpsbasenr = ss.str();

  //fill INS Angle and GPS COG
  ros_msg.insroll = adma_data.insroll;
  ros_msg.inspitch = adma_data.inspitch;
  ros_msg.insyaw = adma_data.insyaw;
  ros_msg.gpscog = adma_data.gnsscog;

  //fill GPS Height MSL
  ros_msg.gpsheight = adma_data.gnssheight;
  ros_msg.undulation = adma_data.undulation;

  // GNSS DUal ant information
  ros_msg.gpsdualanttimemsec = adma_data.gnssDualAntTimeMsec;
  ros_msg.gpsdualanttimeweek = adma_data.gnssDualAntTimeWeek;
  ros_msg.gpsdualantheading = adma_data.gnssDualAntHeading;
  ros_msg.gpsdualantpitch = adma_data.gnssDualAntPitch;

  //GNSS Dualant ETE
  ros_msg.gpsdualantstddevheading = adma_data.gnssdualantstdevheading;
  ros_msg.gpsdualantstddevpitch = adma_data.gnssdualantstddevpitch;
  ros_msg.gpsdualantstddevheading_hr = adma_data.gnssdualantstdevheadinghr;
  ros_msg.gpsdualantstddevpitch_hr = adma_data.gnssdualantstddevpitchhr;

  //fill INS height MSL (+ POI)
  ros_msg.insheight = adma_data.insHeight;
  ros_msg.insheight_1 = adma_data.insHeightPOI1;
  ros_msg.insheight_2 = adma_data.insHeightPOI2;
  ros_msg.insheight_3 = adma_data.insHeightPOI3;
  ros_msg.insheight_4 = adma_data.insHeightPOI4;
  ros_msg.insheight_5 = adma_data.insHeightPOI5;
  ros_msg.insheight_6 = adma_data.insHeightPOI6;
  ros_msg.insheight_7 = adma_data.insHeightPOI7;
  ros_msg.insheight_8 = adma_data.insHeightPOI8;

  //fill INS time UTC
  ros_msg.instimemsec = adma_data.instimemsec;
  ros_msg.instimeweek = adma_data.instimeweek;
  ros_msg.leapseconds = adma_data.leapseconds;

  //fill INS Position (+POI)
  ros_msg.inslatabs = adma_data.insPosAbs.latitude;
  ros_msg.inslonabs = adma_data.insPosAbs.longitude;
  ros_msg.inslatrel = adma_data.insPosRel.longitude;
  ros_msg.inslonrel = adma_data.insPosRel.latitude;
  ros_msg.inslatabs_1 = adma_data.insPosAbsPOI1.latitude;
  ros_msg.inslonabs_1 = adma_data.insPosAbsPOI1.longitude;
  ros_msg.inslatrel_1 = adma_data.insPosRelPOI1.longitude;
  ros_msg.inslonrel_1 = adma_data.insPosRelPOI1.latitude;
  ros_msg.inslatabs_2 = adma_data.insPosAbsPOI2.latitude;
  ros_msg.inslonabs_2 = adma_data.insPosAbsPOI2.longitude;
  ros_msg.inslatrel_2 = adma_data.insPosRelPOI2.longitude;
  ros_msg.inslonrel_2 = adma_data.insPosRelPOI2.latitude;
  ros_msg.inslatabs_3 = adma_data.insPosAbsPOI3.latitude;
  ros_msg.inslonabs_3 = adma_data.insPosAbsPOI3.longitude;
  ros_msg.inslatrel_3 = adma_data.insPosRelPOI3.longitude;
  ros_msg.inslonrel_3 = adma_data.insPosRelPOI3.latitude;
  ros_msg.inslatabs_4 = adma_data.insPosAbsPOI4.latitude;
  ros_msg.inslonabs_4 = adma_data.insPosAbsPOI4.longitude;
  ros_msg.inslatrel_4 = adma_data.insPosRelPOI4.longitude;
  ros_msg.inslonrel_4 = adma_data.insPosRelPOI4.latitude;
  ros_msg.inslatabs_5 = adma_data.insPosAbsPOI5.latitude;
  ros_msg.inslonabs_5 = adma_data.insPosAbsPOI5.longitude;
  ros_msg.inslatrel_5 = adma_data.insPosRelPOI5.longitude;
  ros_msg.inslonrel_5 = adma_data.insPosRelPOI5.latitude;
  ros_msg.inslatabs_6 = adma_data.insPosAbsPOI6.latitude;
  ros_msg.inslonabs_6 = adma_data.insPosAbsPOI6.longitude;
  ros_msg.inslatrel_6 = adma_data.insPosRelPOI6.longitude;
  ros_msg.inslonrel_6 = adma_data.insPosRelPOI6.latitude;
  ros_msg.inslatabs_7 = adma_data.insPosAbsPOI7.latitude;
  ros_msg.inslonabs_7 = adma_data.insPosAbsPOI7.longitude;
  ros_msg.inslatrel_7 = adma_data.insPosRelPOI7.longitude;
  ros_msg.inslonrel_7 = adma_data.insPosRelPOI7.latitude;
  ros_msg.inslatabs_8 = adma_data.insPosAbsPOI8.latitude;
  ros_msg.inslonabs_8 = adma_data.insPosAbsPOI8.longitude;
  ros_msg.inslatrel_8 = adma_data.insPosRelPOI8.longitude;
  ros_msg.inslonrel_8 = adma_data.insPosRelPOI8.latitude;

  //fill ins velocity (horizontal + frame)
  ros_msg.insvelhorx = adma_data.insVelHor.x;
  ros_msg.insvelhory = adma_data.insVelHor.y;
  ros_msg.insvelhorz = adma_data.insVelHor.z;
  ros_msg.insvelframex = adma_data.insVelFrame.x;
  ros_msg.insvelframey = adma_data.insVelFrame.y;
  ros_msg.insvelframez = adma_data.insVelFrame.z;

  //fill INS velocity (POI)
  ros_msg.insvelhorx_1 = adma_data.insVelHorPOI1.x;
  ros_msg.insvelhory_1 = adma_data.insVelHorPOI1.y;
  ros_msg.insvelhorz_1 = adma_data.insVelHorPOI1.z;
  ros_msg.insvelhorx_2 = adma_data.insVelHorPOI2.x;
  ros_msg.insvelhory_2 = adma_data.insVelHorPOI2.y;
  ros_msg.insvelhorz_2 = adma_data.insVelHorPOI2.z;
  ros_msg.insvelhorx_3 = adma_data.insVelHorPOI3.x;
  ros_msg.insvelhory_3 = adma_data.insVelHorPOI3.y;
  ros_msg.insvelhorz_3 = adma_data.insVelHorPOI3.z;
  ros_msg.insvelhorx_4 = adma_data.insVelHorPOI4.x;
  ros_msg.insvelhory_4 = adma_data.insVelHorPOI4.y;
  ros_msg.insvelhorz_4 = adma_data.insVelHorPOI4.z;
  ros_msg.insvelhorx_5 = adma_data.insVelHorPOI5.x;
  ros_msg.insvelhory_5 = adma_data.insVelHorPOI5.y;
  ros_msg.insvelhorz_5 = adma_data.insVelHorPOI5.z;
  ros_msg.insvelhorx_6 = adma_data.insVelHorPOI6.x;
  ros_msg.insvelhory_6 = adma_data.insVelHorPOI6.y;
  ros_msg.insvelhorz_6 = adma_data.insVelHorPOI6.z;
  ros_msg.insvelhorx_7 = adma_data.insVelHorPOI7.x;
  ros_msg.insvelhory_7 = adma_data.insVelHorPOI7.y;
  ros_msg.insvelhorz_7 = adma_data.insVelHorPOI7.z;
  ros_msg.insvelhorx_8 = adma_data.insVelHorPOI8.x;
  ros_msg.insvelhory_8 = adma_data.insVelHorPOI8.y;
  ros_msg.insvelhorz_8 = adma_data.insVelHorPOI8.z;

  //fill INS Expected Position Error
  ros_msg.insstddevlat = adma_data.insstddevlat;
  ros_msg.insstddevlong = adma_data.insstddevlong;
  ros_msg.insstddevheight = adma_data.insstddevheight;

  //fill INS EVE and INS ETE
  ros_msg.insstddevvelx = adma_data.insstddevvelx;
  ros_msg.insstddevvely = adma_data.insstddevvely;
  ros_msg.insstddevvelz = adma_data.insstddevvelz;
  ros_msg.insstddevroll = adma_data.insstddevroll;
  ros_msg.insstddevpitch = adma_data.insstddevpitch;
  ros_msg.insstddevyaw = adma_data.insstddevyaw;

  //fill Analog in 1
  ros_msg.an1 = adma_data.an1;
  ros_msg.an2 = adma_data.an2;
  ros_msg.an3 = adma_data.an3;
  ros_msg.an4 = adma_data.an4;

  // kalman filter status
  ros_msg.kflatstimulated = adma_data.kflatstimulated;
  ros_msg.kflongstimulated = adma_data.kflongstimulated;
  ros_msg.kfsteadystate = adma_data.kfsteadystate;

  // gnss receiver status and error
  ros_msg.gpsreceivererror = adma_data.gnssreceivererror;
  ros_msg.gpsreceiverstatus = adma_data.gnssreceiverstatus;
}

void ADMA2ROSParserV333::getKFStatus(
  adma_ros_driver_msgs::msg::AdmaData & ros_msg, unsigned char kf_status)
{
  bool status_speed_b2 = getbit(kf_status, 5);
  bool status_speed_b1 = getbit(kf_status, 4);
  bool status_kf_steady_state = getbit(kf_status, 3);
  bool status_kf_long_stimulated = getbit(kf_status, 2);
  bool status_kf_lat_stimulated = getbit(kf_status, 1);
  bool status_kalmanfilter_settled = getbit(kf_status, 0);
  ros_msg.statuskalmanfiltersetteled = status_kalmanfilter_settled;
  ros_msg.statuskflatstimulated = status_kf_lat_stimulated;
  ros_msg.statuskflongstimulated = status_kf_long_stimulated;
  ros_msg.statuskfsteadystate = status_kf_steady_state;
  if (status_speed_b1 == 0 && status_speed_b2 == 0) {
    ros_msg.statusspeed = 0;
  } else if (status_speed_b1 == 0 && status_speed_b2 == 1) {
    ros_msg.statusspeed = 1;
  } else if (status_speed_b1 == 1 && status_speed_b2 == 0) {
    ros_msg.statusspeed = 2;
  }
}
