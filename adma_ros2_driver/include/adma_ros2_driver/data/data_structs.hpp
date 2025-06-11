// BSD 3-Clause License
// Copyright (c) 2023, GeneSys Elektronik
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// 1. Redistributions of source code must retain the above copyright notice, this
//    list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from
//    this software without specific prior written permission.
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include <iostream>

#pragma once

// predefined reserved slots
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
  GNSSPosition pos_abs;  // deg
  int32_t pos_rel_x;     // m
  int32_t pos_rel_y;     // m
};
