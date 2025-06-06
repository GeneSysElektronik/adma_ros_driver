#include "adma_core_lib/parser/mapping.hpp"
#include <fstream>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

namespace genesys
{
namespace parser
{

Mapping::Mapping(uint16_t protocolVersion, std::string package_name){
        version_ = protocolVersion;
        std::string configPath = ament_index_cpp::get_package_share_directory(package_name);
        configPath += "/config/protocols/";
        std::string admanetXMLFile, mappingGlossarFile;
        mappingGlossarFile = configPath + "channel_mapping.json";
        RCLCPP_INFO(rclcpp::get_logger("genesys::parser::Mapping"), "Loading Glossar: %s", mappingGlossarFile.c_str());
        std::ifstream glossarFile(mappingGlossarFile);
        glossarFile >> glossar_;
        nlohmann::json xmlFiles = glossar_["xml_files"];
        for (const auto& entry : xmlFiles) {
                if(entry["version"] == version_){
                        std::string xmlFileName = entry["filename"];
                        admanetXMLFile = configPath + xmlFileName;
                }
        }
        RCLCPP_INFO(rclcpp::get_logger("genesys::parser::Mapping"), "Loading XML: %s", admanetXMLFile.c_str());
        // load protocol XML
        channelMap_ = parseXMLProtocol(admanetXMLFile);
        validateProtocol();
}

Mapping::~Mapping(){}

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
        nlohmann::json closest = nullptr;
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
