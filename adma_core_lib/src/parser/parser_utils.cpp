#include "adma_core_lib/parser/parser_utils.hpp"

/// \file
/// \brief  bit shift function
/// \param  byte byte information
/// \param  position message
/// \return an integer 0 upon exit success
bool getbit(unsigned char byte, int position)  // position in range 0-7
{
  return (byte >> position) & 0x1;
}

uint8_t getBits(unsigned char byte, int bitOffset, int length)
{
  if (bitOffset + length > 8)
    throw std::out_of_range("Bitbereich überschreitet 8 Bit");

  uint8_t mask = (1 << length) - 1;
  return (byte >> bitOffset) & mask;
}

double getScaledValue(int32_t raw_value, double lsb_factor)
{
  return double(raw_value) * lsb_factor;
}

bool isLittleEndian() {
  uint16_t test = 0x1;
  return *reinterpret_cast<uint8_t*>(&test) == 0x1;
}

void extractAdmanetHeader(adma_ros_driver_msgs::msg::AdmanetHeader &headerMsg, std::array<char, 856>& buffer)
{
  genesys::AdmaHeader admaHeaderStruct;
  memcpy(&admaHeaderStruct, &buffer, sizeof(admaHeaderStruct));
  // fill static header information
  headerMsg.genesys_id = admaHeaderStruct.genesysid;
  std::stringstream ss;
  ss << int(admaHeaderStruct.headerversion[3]) << int(admaHeaderStruct.headerversion[2])
     << int(admaHeaderStruct.headerversion[1]) << int(admaHeaderStruct.headerversion[0]);
  headerMsg.header_version = std::stoi(ss.str());
  ss.clear();
  ss.str("");
  headerMsg.format_id = admaHeaderStruct.formatid;
  ss << int(admaHeaderStruct.formatversion[3]) << int(admaHeaderStruct.formatversion[2])
     << int(admaHeaderStruct.formatversion[1]) << int(admaHeaderStruct.formatversion[0]);
  headerMsg.format_version = std::stoi(ss.str());
  ss.clear();
  headerMsg.serial_number = admaHeaderStruct.serialno;

  // fill dynamic header information
  headerMsg.config_id = admaHeaderStruct.configid;
  headerMsg.config_format = admaHeaderStruct.configformat;
  headerMsg.config_version = admaHeaderStruct.configversion;
  headerMsg.config_size = admaHeaderStruct.configsize;
  headerMsg.byte_offset = admaHeaderStruct.byteoffset;
  headerMsg.slice_size = admaHeaderStruct.slicesize;
  headerMsg.slice_data = admaHeaderStruct.slicedata;
}