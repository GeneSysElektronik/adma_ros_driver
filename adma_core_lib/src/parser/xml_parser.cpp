#include "adma_core_lib/parser/xml_parser.hpp"
#include <tinyxml2.h>

std::map<int, genesys::DataChannel> parseXMLProtocol(const std::string &xmlFileName)
{
        std::map<int, genesys::DataChannel> channelMap;
        tinyxml2::XMLDocument xmlDoc;
        xmlDoc.LoadFile(xmlFileName.c_str());

        if (xmlDoc.LoadFile(xmlFileName.c_str()) != tinyxml2::XML_SUCCESS) {
                // std::cerr << "Fehler beim Laden der XML-Datei: " << xmlFileName << std::endl;
                return channelMap;
        }

        auto* measurementData = xmlDoc.FirstChildElement("MessageFormatDefinition")->FirstChildElement("MeasurementData");
        if(!measurementData)
        {
                std::cerr << "Keine Elemente in der XML gefunden! " << std::endl;
        }
        for (auto* package = measurementData->FirstChildElement("Package"); package; package = package->NextSiblingElement("Package")) 
        {
                for (auto* channel = package->FirstChildElement("Channel"); channel; channel = channel->NextSiblingElement("Channel")) 
                {
                        // first extract ChannelID since this is the key for lookup mapping
                        int id = std::stoi(channel->Attribute("ChannelID"));
                        genesys::DataChannel dataChannel;
                        dataChannel.name = channel->Attribute("Name");
                        dataChannel.channelID = id;

                        if (auto* value = channel->FirstChildElement("Scale")) {
                                dataChannel.scale = std::stof(value->GetText());
                        }
                        if (auto* value = channel->FirstChildElement("DataType")) {
                                dataChannel.dataTypeName = value->GetText();
                                dataChannel.dataType = getDataType(dataChannel.dataTypeName);
                        }
                        if (auto* value = channel->FirstChildElement("Min")) {
                                dataChannel.minRange = std::stof(value->GetText());
                        }
                        if (auto* value = channel->FirstChildElement("Max")) {
                                dataChannel.maxRange = std::stof(value->GetText());
                        }
                        if (auto* value = channel->FirstChildElement("DecimalPlaces")) {
                                dataChannel.decimalPlaces = static_cast<int>(std::stoul(value->GetText()));
                        }
                        if (auto* value = channel->FirstChildElement("PreDecimalPlaces")) {
                                dataChannel.preDecimalPlaces = static_cast<int>(std::stoul(value->GetText()));
                        }
                        if (auto* value = channel->FirstChildElement("ByteOffset")) {
                                dataChannel.byteOffset = std::stoi(value->GetText());
                        }
                        if (auto* value = channel->FirstChildElement("BitOffset")) {
                                dataChannel.bitOffset = static_cast<int>(std::stoul(value->GetText()));
                        }
                        if (auto* value = channel->FirstChildElement("LengthBits")) {
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
        if (datatypeName == "INT") return genesys::ADMADataType::INT;
        if (datatypeName == "UINT") return genesys::ADMADataType::UINT;
        if (datatypeName == "BOOL") return genesys::ADMADataType::BOOL;
        return genesys::ADMADataType::UNKNOWN;
}