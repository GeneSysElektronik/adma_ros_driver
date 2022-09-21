#include "adma_ros2_driver/data/data_structs.hpp"

#pragma once



// definition of the whole data packet of the UDP protocol (V 3.3.3)
struct AdmaDataV333
{
        AdmaStaticHeader staticHeader;
        AdmaDynamicHeader dynamicHeader;
        //contains several states in 1 byte
        unsigned char gnssStatus;
        unsigned char signalInStatus;
        unsigned char miscStatus;
        
        int8_t statuscount;
        unsigned char kfStatus;
        Reserved3 reservedStatus1;

        //contains several errors/warning in 1 byte
        unsigned char dataError1;
        unsigned char dataError2;
        unsigned char dataWarn1;
        unsigned char dataError3;
        Reserved4 reservedError1;

        SensorBody sensorsBodyX;
        SensorBody sensorsBodyY;
        SensorBody sensorsBodyZ;
        Vector3 ratesBody; // deg/s
        Vector3 ratesHorizontal; //deg/s
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
        Vector3 accBodyPOI8; // g
        Vector3 accHorizontalPOI1; // g
        Vector3 accHorizontalPOI2; // g
        Vector3 accHorizontalPOI3; // g
        Vector3 accHorizontalPOI4; // g
        Vector3 accHorizontalPOI5; // g
        Vector3 accHorizontalPOI6; // g
        Vector3 accHorizontalPOI7; // g
        Vector3 accHorizontalPOI8; // g

        //  vel in m/s
        Vector2 extVelAnalog;
        // external velocity digital pulses
        int16_t extveldigx;
        int16_t extveldigy;
        uint16_t extveldigpulsesx;
        int16_t extveldigpulsesy;
        Vector2 extVelCorrected;
        
        // Reserved Block
        Reserved4 reservedData1;
        Reserved4 reservedData2;
        Reserved4 reservedData3;
        Reserved4 reservedData4;
        Reserved4 reservedData5;
        Reserved4 reservedData6;

        // miscellaneous
        Miscellaneous misc;
        Miscellaneous miscPOI1;
        Miscellaneous miscPOI2;
        Miscellaneous miscPOI3;
        Miscellaneous miscPOI4;
        Miscellaneous miscPOI5;
        Miscellaneous miscPOI6;
        Miscellaneous miscPOI7;
        Miscellaneous miscPOI8;

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

        // gnss position absolute
        // position in deg
        GNSSPosition posAbs; 

        // gnss position relative
        // position in m
        GNSSPosition posRel;

        // gnss epe
        //  error in m
        uint16_t gnssstddevlat;
        uint16_t gnssstddevlon;
        uint16_t gnssstddevheight;
        Reserved2 reservedSpace4;

        // gnss velocity frame
        // vel in m/s
        int16_t gnssvelframex;
        int16_t gnssvelframey;
        int16_t gnssvelframez;
        // latency in s
        uint16_t gnssvellatency;

        // gnss eve
        // error in m/s
        Vector3 gnssStdDevVel;

        // gnss time utc
        // gnsstimemsec in ms
        uint32_t gnsstimemsec;
        // gnsstimeweek in week
        uint16_t gnsstimeweek;
        // gnsstrigger in mu.s
        uint16_t gnsstrigger;

        // gnss auxdata 1
        // gnssdiffage in s
        uint16_t gnssdiffage;
        unsigned char gnsssatsused;
        unsigned char gnsssatsvisible;
        unsigned char gnsssatsdualantused;
        unsigned char gnsssatsdualantvisible;
        Reserved2 reservedSpace5;

        // gnss auxdata 2
        // gnsslogdelay in ta
        unsigned char gnsslogdelay;
        // gnssreceiverload in %
        unsigned char gnssreceiverload;
        unsigned char gnssbasenr[4];
        Reserved2 reservedSpace6;

        // ins angle and gnss cog
        // all units in deg
        int16_t insroll;
        int16_t inspitch;
        uint16_t insyaw;
        uint16_t gnsscog;

        // gnss height(msl)
        int32_t gnssheight;
        int16_t undulation;
        Reserved2 reservedSpace7;
        
        // gnss dualant angle ete
        // all units in °
        unsigned char gnssdualantsttdevheading;
        unsigned char gnssdualantstddevpitch;
        uint16_t gnssdualantsttdevheadinghr;
        uint16_t gnssdualantstddevpitchhr;
        Reserved2 reservedSpace8;

        // ins position height (msl)
        // all units in m
        int32_t insHeight;
        Reserved4 reservedSpace9;
        int32_t insHeightPOI1;
        int32_t insHeightPOI2;
        int32_t insHeightPOI3;
        int32_t insHeightPOI4;
        int32_t insHeightPOI5;
        int32_t insHeightPOI6;
        int32_t insHeightPOI7;
        int32_t insHeightPOI8;

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
        GNSSPosition insPosAbsPOI8;
        GNSSPosition insPosRelPOI8;
        

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
        Vector3 insVelHorPOI8;

        // ins epe
        // error in m
        uint16_t insstddevlat;
        uint16_t insstddevlong;
        uint16_t insstddevheight;
        Reserved2 reservedSpace11;

        // ins eve and ins ete
        //  std vel in m/s
        int8_t insstddevvelx;
        int8_t insstddevvely;
        int8_t insstddevvelz;
        // std ete in deg
        int8_t insstddevroll;
        int8_t insstddevpitch;
        int8_t insstddevyaw;
        Reserved2 reservedSpace12;

        // analog in1
        // analog in v
        int16_t an1;
        int16_t an2;
        int16_t an3;
        int16_t an4;
        
        // kalmanfilter status
        // all units in %
        uint8_t kflatstimulated;
        uint8_t kflongstimulated;
        uint8_t kfsteadystate;
        Reserved1 reservedSpace13;
        
        // gnss receiver status and error
        uint32_t gnssreceivererror;
        uint32_t gnssreceiverstatus;
};