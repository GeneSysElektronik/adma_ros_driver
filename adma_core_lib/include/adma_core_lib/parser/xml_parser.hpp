#include <map>
#include "adma_core_lib/parser/data_structs.hpp"

#pragma once

std::map<int, genesys::DataChannel> parseXMLProtocol( const std::string &xmlFileName);
genesys::ADMADataType getDataType(std::string datatypeName);