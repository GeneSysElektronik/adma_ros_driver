#pragma once

#include <iostream>

const double PI=3.1415926535897932384626433832795028841971;

bool getbit(unsigned char byte, int position);
double getScaledValue(int32_t rawValue, double lsbFactor);