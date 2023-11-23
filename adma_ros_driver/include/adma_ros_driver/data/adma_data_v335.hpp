#include "adma_ros_driver/data/data_structs.hpp"

#pragma once



// definition of the whole data packet of the UDP protocol (V 3.3.5)
struct AdmaDataV335
{
        AdmaStaticHeader staticHeader;
        AdmaDynamicHeader dynamicHeader;
        //contains several states in 1 byte
        unsigned char gnssStatus;
        unsigned char signalInStatus;
        unsigned char miscStatus;
        
        int8_t statuscount;
        unsigned char kfStatus;
        unsigned char statusRobot;
        unsigned char reservedStatus1[2];

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
        Vector3 accBodyPOI[8];
        Vector3 accHorizontalPOI[8];

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
        Miscellaneous miscPOI[8];

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
        uint16_t gnssstddevcog;

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
        unsigned char gnsssatssinglefreq;
        unsigned char gnsssatsmultifreq;

        // gnss auxdata 2
        // gnsslogdelay in ta
        unsigned char gnsslogdelay;
        // gnssreceiverload in %
        unsigned char gnssreceiverload;
        unsigned char gnssbasenr[4];
        unsigned char reservedSpace5[1];
        unsigned char gnsssatsdualantmultifreq;

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
        
        //gnss dualant time UTC
        uint32_t gnssDualAntTimeMsec;
        uint16_t gnssDualAntTimeWeek;
        Reserved2 reservedSpace8;
        
        //gnss dualant angle
        uint16_t gnssDualAntHeading;
        int16_t gnssDualAntPitch;
        Reserved4 reservedSpace9;

        // gnss dualant angle ete
        // all units in °
        unsigned char gnssdualantstdevheading;
        unsigned char gnssdualantstddevpitch;
        uint16_t gnssdualantstdevheadinghr;
        uint16_t gnssdualantstddevpitchhr;
        Reserved2 reservedSpace10;

        // ins position height (msl)
        // all units in m
        int32_t insHeight;
        uint16_t insyawrel;
        Reserved2 reservedSpace11;
        int32_t insHeightPOI[8];

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
        INSPosition insPos;
        INSPosition insPosPOI[8];

        // ins velocity horizontal
        // velocity in m/s
        Vector3 insVelHor;

        // ins velocity frame
        // velocity in m/s
        Vector3 insVelFrame;

        // ins velocity POI
        Vector3 insVelHorPOI[8];

        // ins epe
        // error in m
        uint16_t insstddevlat;
        uint16_t insstddevlong;
        uint16_t insstddevheight;
        Reserved2 reservedSpace12;

        // ins eve and ins ete
        //  std vel in m/s
        int8_t insstddevvelx;
        int8_t insstddevvely;
        int8_t insstddevvelz;
        // std ete in deg
        int8_t insstddevroll;
        int8_t insstddevpitch;
        int8_t insstddevyaw;
        Reserved2 reservedSpace13;

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
        unsigned char reservedSpace14[5];
        
        // gnss receiver status and error
        uint32_t gnssreceivererror;
        uint32_t gnssreceiverstatus;
};