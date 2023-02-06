#include "../../include/adma_ros_driver/parser/parser_utils.hpp"

/// \file
/// \brief  bit shift function
/// \param  byte byte information
/// \param  position message
/// \return an integer 0 upon exit success
bool getbit(unsigned char byte, int position) // position in range 0-7
{
    return (byte >> position) & 0x1;
}

double getScaledValue(int32_t rawValue, double lsbFactor)
{
    return rawValue * lsbFactor;
}