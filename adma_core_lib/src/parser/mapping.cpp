#include "adma_core_lib/parser/mapping.hpp"
#include <fstream>
#include <iostream>
#include <rclcpp/rclcpp.hpp>


namespace genesys
{
namespace parser
{

Mapping::Mapping(uint16_t protocolVersion){
        version_ = protocolVersion;
}

Mapping::~Mapping(){}

void Mapping::initialize(const std::string &protocolFileName, const std::string &glossarFileName)
{
        RCLCPP_INFO(rclcpp::get_logger("genesys::parser::Mapping"), "Loading XML: %s", protocolFileName.c_str());
        RCLCPP_INFO(rclcpp::get_logger("genesys::parser::Mapping"), "Loading Glossar: %s", glossarFileName.c_str());
        // load json glossar file
        std::ifstream glossarFile(glossarFileName);
        glossarFile >> glossar_;

        // load protocol XML
        channelMap_ = parseXMLProtocol(protocolFileName);
        validateProtocol();
}

void Mapping::extractEntries(const nlohmann::json& j, const std::string& parentKey, std::map<std::string, nlohmann::json>& result)
{
    if (j.is_array()) {
        for (const auto& entry : j) {
            if (entry.is_object() && entry.contains("version") && entry.contains("channelID")) {
                result[parentKey] = entry;  // Speichere das Objekt mit zusammengesetztem Key
            }
        }
    }
    else if (j.is_object()) {
        for (const auto& [key, value] : j.items()) {
            std::string newKey = parentKey.empty() ? key : parentKey + "." + key;
            extractEntries(value, newKey, result);
        }
    }
}

void Mapping::validateProtocol()
{
        extractEntries(glossar_, "", glossarMap_);
        std::vector<std::string> unknownCHannels;
        for (const auto& [key, value] : glossarMap_) 
        {
                nlohmann::json channelIDs = value["channelID"];
                if(channelIDs.is_array())
                {
                        for (const auto& id : channelIDs) {
                                int channelID = id.get<int>();  // Konvertiert das JSON-Element in einen Integer
                                if(channelMap_.count(channelID) == 0)
                                {
                                        unknownCHannels.push_back(key + "_" + std::to_string(channelID));
                                }
                        }
                }else{
                        int channelID = channelIDs.get<int>();
                        if(channelMap_.count(channelID) == 0)
                        {
                                unknownCHannels.push_back(key + "_" + std::to_string(channelID));
                        }
                }
        }

        if(!unknownCHannels.empty()){
                RCLCPP_WARN(rclcpp::get_logger("genesys::parser::Mapping"), "The following ROS channels are unsupported with ADMAnet v%d", version_);
                for(const auto& value: unknownCHannels)
                {
                        RCLCPP_WARN(rclcpp::get_logger("genesys::parser::Mapping"), "%s", value.c_str());
                }
        }
        
}

nlohmann::json Mapping::getChannelIDByROSName(const std::string &rosChannelName)
{
        nlohmann::json result;
        if(glossarMap_.count(rosChannelName) == 0)
        {
                RCLCPP_ERROR(rclcpp::get_logger("genesys::parser::Mapping"), "couldnt find key: %s", rosChannelName.c_str());
                return result;
        }else{
                return glossarMap_[rosChannelName];
        }
        std::vector<std::string> keys = splitKeys(rosChannelName);
        nlohmann::json value = glossar_;
        // search for channel IDs
        for (const auto& key : keys) 
        {
                if (value.contains(key)) {
                        value = value[key];
                } else {
                        RCLCPP_ERROR(rclcpp::get_logger("genesys::parser::Mapping"), "couldnt find key: %s", rosChannelName.c_str());
                        return 0;
                }
        }

        // sort results by version number
        // std::sort(value.begin(), value.end(), [](const nlohmann::json& a, const nlohmann::json& b) {
        //         return a["version"].get<int>() < b["version"].get<int>();
        //     });
        // find closest protocol version defined in glossar
        // int minDiff = std::numeric_limits<int>::max();
        nlohmann::json closest = nullptr;
        // for (const auto& entry : value) {
        //         int curVersion = entry["version"].get<int>();
        //         int diff = std::abs(curVersion - version_);

        //         if (diff < minDiff) {
        //         minDiff = diff;
        //         closest = entry;
        //         }
        // }
        // if(closest["version"].get<int>() > version_){
        //         // dont try to use channels from higher versions..
        //         std::cout << rosChannelName << " not supported in version " << version_ << ", first support at version: " << closest["version"].get<int>() << std::endl;
        //         nlohmann::json empty;
        //         return empty;
        // }

        // std::cout << rosChannelName << " : " << closest.dump() << std::endl;
        return closest;
}

std::vector<std::string> Mapping::splitKeys(const std::string &keys)
{
        std::vector<std::string> result;
        std::istringstream ss(keys);
        std::string token;
    
        while (std::getline(ss, token, '.')) {
                result.push_back(token);
        }
    
        return result;
}

genesys::DataChannel Mapping::getDataChannelByID(uint16_t channelID)
{
       return channelMap_[channelID];
}

void Mapping::loadVector3FromBuffer(const std::string &rosChannelName, std::array<char, 856> &buffer,  geometry_msgs::msg::Vector3 &source)
{
        arrayCounter_ = 0;
        //TODO: some attributes are 16 Bits instead of 32 Bit...
        source.x = loadDataFromBuffer<int32_t, double>(rosChannelName, buffer);
        source.y = loadDataFromBuffer<int32_t, double>(rosChannelName, buffer);
        source.z = loadDataFromBuffer<int32_t, double>(rosChannelName, buffer);
        arrayCounter_ = 0;

}

} // end namespace parser
} // end namespace genesys