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

#include "adma_core_lib/parser/xml_parser.hpp"
#include <tinyxml2.h>

std::map<int, genesys::DataChannel> parseXMLProtocol(const std::string & xmlFileName)
{
  std::map<int, genesys::DataChannel> channelMap;
  tinyxml2::XMLDocument xmlDoc;
  xmlDoc.LoadFile(xmlFileName.c_str());

  if (xmlDoc.LoadFile(xmlFileName.c_str()) != tinyxml2::XML_SUCCESS) {
    std::cerr << "Fehler beim Laden der XML-Datei: " << xmlFileName << std::endl;
    return channelMap;
  }
  //  NOLINTBEGIN
  auto * measurementData = xmlDoc.FirstChildElement("MessageFormatDefinition")->FirstChildElement(
    "MeasurementData");
  if (!measurementData) {
    std::cerr << "Keine Elemente in der XML gefunden! " << std::endl;
  }
  for (auto * package = measurementData->FirstChildElement("Package"); package;
    package = package->NextSiblingElement("Package"))
  {
    for (auto * channel = package->FirstChildElement("Channel"); channel;
      channel = channel->NextSiblingElement("Channel"))
    {
      //  NOLINTEND
      // first extract ChannelID since this is the key for lookup mapping
      int id = std::stoi(channel->Attribute("ChannelID"));
      genesys::DataChannel dataChannel;
      dataChannel.name = channel->Attribute("Name");
      dataChannel.channelID = id;

      if (auto * value = channel->FirstChildElement("Scale")) {
        dataChannel.scale = std::stof(value->GetText());
      }
      if (auto * value = channel->FirstChildElement("DataType")) {
        dataChannel.dataTypeName = value->GetText();
        dataChannel.dataType = getDataType(dataChannel.dataTypeName);
      }
      if (auto * value = channel->FirstChildElement("Min")) {
        dataChannel.minRange = std::stof(value->GetText());
      }
      if (auto * value = channel->FirstChildElement("Max")) {
        dataChannel.maxRange = std::stof(value->GetText());
      }
      if (auto * value = channel->FirstChildElement("DecimalPlaces")) {
        dataChannel.decimalPlaces = static_cast<int>(std::stoul(value->GetText()));
      }
      if (auto * value = channel->FirstChildElement("PreDecimalPlaces")) {
        dataChannel.preDecimalPlaces = static_cast<int>(std::stoul(value->GetText()));
      }
      if (auto * value = channel->FirstChildElement("ByteOffset")) {
        dataChannel.byteOffset = std::stoi(value->GetText());
      }
      if (auto * value = channel->FirstChildElement("BitOffset")) {
        dataChannel.bitOffset = static_cast<int>(std::stoul(value->GetText()));
      }
      if (auto * value = channel->FirstChildElement("LengthBits")) {
        dataChannel.lengthBits = static_cast<int>(std::stoul(value->GetText()));
      }
      channelMap[id] = dataChannel;
    }
  }

  // for(const auto &[id, data] : channelMap)
  // {
  //   std::cout << "ID: " << id
  //             << ", Name: " << data.name
  //             << ", DataType: " << data.dataType
  //             << ", Scale: " << data.scale
  //             << ", ByteOffset: " << data.byteOffset
  //             << ", BitOffset: " << data.bitOffset
  //             << ", lengthBits: " << data.lengthBits
  //             << std::endl;
  // }

  return channelMap;
}

genesys::ADMADataType getDataType(std::string datatypeName)
{
  if (datatypeName == "INT") {return genesys::ADMADataType::INT;}
  if (datatypeName == "UINT") {return genesys::ADMADataType::UINT;}
  if (datatypeName == "BOOL") {return genesys::ADMADataType::BOOL;}
  return genesys::ADMADataType::UNKNOWN;
}
