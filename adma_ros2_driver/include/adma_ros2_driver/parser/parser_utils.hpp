#pragma once

#include <iostream>

const double PI = 3.1415926535897932384626433832795028841971;

bool getbit(unsigned char byte, int position);
double getScaledValue(int32_t raw_value, double lsb_factor);

template <typename T>
inline T deg2Rad(T deg){ return T(deg * PI / 180.0); }

template <typename T>
inline T rad2Deg(T rad){ return T(rad * 180 / PI); }
