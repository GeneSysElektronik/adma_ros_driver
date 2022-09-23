#include "adma_ros2_driver/data/data_structs.hpp"

#pragma once



// definition of the whole data packet of the UDP protocol (V 3.2)
struct AdmaDataV32
{
        AdmaStaticHeader staticHeader;
        AdmaDynamicHeader dynamicHeader;
        //contains several states in 1 byte
        unsigned char gpsStatus;
        unsigned char gpsTriggerStatus;
        unsigned char evkStatus;
        
        int8_t statuscount;
        Reserved4 reservedStatus1;

        //contains several errors/warning in 1 byte
        unsigned char dataError1;
        unsigned char dataError2;
        unsigned char dataWarn1;
        unsigned char dataErrorHW;
        Reserved4 reservedError1;

        SensorBody sensorsBodyX;
        SensorBody sensorsBodyY;
        SensorBody sensorsBodyZ;
        Vector3 rateBody; // deg/s
        Vector3 rateHorizontal; //deg/s
        Vector3 accBody; // g
        Vector3 accHorizontal; // g

        //POI information block 1
        Vector3 accBodyPOI1; // g
        Vector3 accBodyPOI2; // g
        Vector3 accBodyPOI3; // g
        Vector3 accBodyPOI4; // g
        Vector3 accBodyPOI5; // g
        Vector3 accBodyPOI6; // g
        Vector3 accBodyPOI7; // g
        Vector3 accHorizontalPOI1; // g
        Vector3 accHorizontalPOI2; // g
        Vector3 accHorizontalPOI3; // g
        Vector3 accHorizontalPOI4; // g
        Vector3 accHorizontalPOI5; // g
        Vector3 accHorizontalPOI6; // g
        Vector3 accHorizontalPOI7; // g

        //  vel in m/s
        Vector2 extVelAnalog;
        // external velocity digital pulses
        int16_t extveldigx;
        int16_t extveldigy;
        uint16_t extveldigpulsesx;
        int16_t extveldigpulsesy;
        Vector2 extVelCorrected;

        // barometer
        uint32_t extbaropressure; // mbar
        Reserved4 reservedSpace1;
        int32_t extbaroheight; //m
        int32_t extbaroheightcorrected; //m

        Reserved4 reservedSpace2;
        Reserved4 reservedSpace3;

        // miscellaneous
        Miscellaneous misc;
        Miscellaneous miscPOI1;
        Miscellaneous miscPOI2;
        Miscellaneous miscPOI3;
        Miscellaneous miscPOI4;
        Miscellaneous miscPOI5;
        Miscellaneous miscPOI6;
        Miscellaneous miscPOI7;

        // triggers 1 and 2
        // triggers in mu.s
        uint16_t trigrising1;
        uint16_t trigfalling1;
        uint16_t trigrising2;
        uint16_t trigfalling2;

        // triggers 3 and 4
        // triggers in mu.s
        uint16_t trigrising3;
        uint16_t trigfalling3;
        uint16_t trigrising4;
        uint16_t trigfalling4;

        // system data
        // systemta in mu.s
        uint16_t systemta;
        // systemtemp in deg. c
        int16_t systemtemp;
        // systemtimesinceinit in s
        uint16_t systemtimesinceinit;
        // system dsp load in %
        uint16_t systemdspload;

        // gps position absolute
        // position in deg
        GNSSPosition gpsPosAbs;

        // gps position relative
        // position in m
        GNSSPosition gpsPosRel;

        // gps epe
        //  error in m
        uint16_t gpsstddevlat;
        uint16_t gpsstddevlon;
        uint16_t gpsstddevheight;
        Reserved2 reservedSpace4;

        // gps velocity frame
        // vel in m/s
        int16_t gpsvelframex;
        int16_t gpsvelframey;
        int16_t gpsvelframez;
        // latency in s
        uint16_t gpsvellatency;

        // gps eve
        // error in m/s
        Vector3 gpsStdDevVel;

        // gps time utc
        // gpstimemsec in ms
        uint32_t gpstimemsec;
        // gpstimeweek in week
        uint16_t gpstimeweek;
        // gpstrigger in mu.s
        uint16_t gpstrigger;

        // gps auxdata 1
        // gpsdiffage in s
        uint16_t gpsdiffage;
        unsigned char gpssatsused;
        unsigned char gpssatsvisible;
        Reserved4 reservedSpace5;

        // gps auxdata 2
        // gpslogdelay in ta
        unsigned char gpslogdelay;
        // gpsreceiverload in %
        unsigned char gpsreceiverload;
        unsigned char gpsbasenr[4];
        Reserved2 reservedSpace6;

        // ins angle and gps cog
        // all units in deg
        int16_t insroll;
        int16_t inspitch;
        uint16_t insyaw;
        uint16_t gpscog;

        // gps height(msl)
        int32_t gpsheight;
        int16_t undulation;
        Reserved2 reservedSpace7;

        // ins position height (msl)
        int32_t ingHeight;
        Reserved4 reservedSpace8;
        int32_t ingHeightPOI1;
        int32_t ingHeightPOI2;
        int32_t ingHeightPOI3;
        int32_t ingHeightPOI4;
        int32_t ingHeightPOI5;
        int32_t ingHeightPOI6;
        int32_t ingHeightPOI7;
        Reserved4 reservedSpace9;

        // ins time utc
        // instimemsec in ms
        uint32_t instimemsec;
        // instimeweek in week
        uint16_t instimeweek;
        // leapseconds in s
        int16_t leapseconds;

        // ins positions
        // absolute position in deg
        // relative position in m
        GNSSPosition insPosAbs;
        GNSSPosition insPosRel;
        GNSSPosition insPosAbsPOI1;
        GNSSPosition insPosRelPOI1;
        GNSSPosition insPosAbsPOI2;
        GNSSPosition insPosRelPOI2;
        GNSSPosition insPosAbsPOI3;
        GNSSPosition insPosRelPOI3;
        GNSSPosition insPosAbsPOI4;
        GNSSPosition insPosRelPOI4;
        GNSSPosition insPosAbsPOI5;
        GNSSPosition insPosRelPOI5;
        GNSSPosition insPosAbsPOI6;
        GNSSPosition insPosRelPOI6;
        GNSSPosition insPosAbsPOI7;
        GNSSPosition insPosRelPOI7;
        

        // ins velocity horizontal
        // velocity in m/s
        Vector3 insVelHor;

        // ins velocity frame
        // velocity in m/s
        Vector3 insVelFrame;

        // ins velocity POI
        Vector3 insVelHorPOI1;
        Vector3 insVelHorPOI2;
        Vector3 insVelHorPOI3;
        Vector3 insVelHorPOI4;
        Vector3 insVelHorPOI5; 
        Vector3 insVelHorPOI6;
        Vector3 insVelHorPOI7;

        // ins epe
        // error in m
        uint16_t insstddevlat;
        uint16_t insstddevlong;
        uint16_t insstddevheight;
        Reserved2 reservedSpace10;

        // ins eve and ins ete
        //  std vel in m/s
        int8_t insstddevvelx;
        int8_t insstddevvely;
        int8_t insstddevvelz;
        // std ete in deg
        int8_t insstddevroll;
        int8_t insstddevpitch;
        int8_t insstddevyaw;
        Reserved2 reservedSpace11;

        // analog in1
        // analog in v
        int16_t an1;
        int16_t an2;
        int16_t an3;
        int16_t an4;
};