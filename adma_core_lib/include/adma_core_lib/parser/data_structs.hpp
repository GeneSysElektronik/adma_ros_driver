#include <iostream>

#pragma once

namespace genesys
{

// struct of the ADMA header
struct ADMAnetHeader
{
        // static part
        char genesysid[4];
        char headerversion[4];
        uint32_t formatid;
        char formatversion[4];
        char reserved[16];
        uint32_t serialno;
        char alias[32];
        //dynamic part
        uint32_t configid;
        uint32_t configformat;
        uint32_t configversion;
        uint32_t configsize;
        uint32_t byteoffset;
        uint32_t slicesize;
        int32_t slicedata;
};

enum class ADMADataType
{
        INT,
        UINT,
        BOOL,
        UNKNOWN
};

struct DataChannel
{
        std::string name;
        uint16_t channelID;
        float scale;
        std::string dataTypeName;
        ADMADataType dataType;
        float minRange;
        float maxRange;
        int decimalPlaces;
        int preDecimalPlaces;
        uint16_t byteOffset;
        int bitOffset;
        int lengthBits;
};



}