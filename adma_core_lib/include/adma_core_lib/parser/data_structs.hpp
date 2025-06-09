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
#include <string>

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
  // dynamic part
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

}  // namespace genesys
