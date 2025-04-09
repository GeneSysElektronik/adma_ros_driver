#include <iostream>

#pragma once

//predefined reserved slots
struct Reserved16
{
  char reservedSpace[16];
};

struct Reserved4
{
  char reservedSpace[4];
};

struct Reserved2
{
  char reservedSpace[2];
};

// struct to represent a sensor body
struct SensorBody
{
  int32_t accHR;   // g
  int32_t rateHR;  // deg/s
};

// struct to define a 3D (XYZ) object
struct Vector3
{
  int16_t x;
  int16_t y;
  int16_t z;
  char reservedSpace[2];
};

// struct to define a 2D (XY) object
struct Vector2
{
  int16_t x;
  int16_t y;
  char reservedSpace1[2];
  char reservedSpace2[2];
};

struct Miscellaneous
{
  int16_t invPathRadius;
  int16_t sideSlipAngle;
  uint32_t distanceTraveled;
};

// struct to define a GNSS position (unit-indepent)
struct GNSSPosition
{
  int32_t latitude;
  int32_t longitude;
};

struct INSPosition
{
  GNSSPosition pos_abs;  //deg
  int32_t pos_rel_x;     //m
  int32_t pos_rel_y;     //m
};