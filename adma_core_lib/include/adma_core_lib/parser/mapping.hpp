#include "adma_core_lib/thirdparty/json.hpp"
#include "adma_core_lib/parser/data_structs.hpp"
#include "adma_core_lib/parser/xml_parser.hpp"
#include "adma_core_lib/parser/parser_utils.hpp"
#include <bitset>
#include <type_traits>
#include <geometry_msgs/msg/vector3.hpp>

#pragma once

namespace genesys
{
namespace parser
{

class Mapping
{
        public:
                Mapping(uint16_t protocolVersion);
                ~Mapping();
        
                void initialize(const std::string &protocolFileName, const std::string &glossarFileName);
                nlohmann::json getChannelIDByROSName(const std::string &rosChannelName);
                std::vector<std::string> splitKeys(const std::string &keys);
                genesys::DataChannel getDataChannelByID(uint16_t channelID);
                void loadVector3FromBuffer(const std::string &rosChannelName, std::array<char, 856> &buffer, geometry_msgs::msg::Vector3 &source);
                template <typename T_source, typename T_dest>
                T_dest loadDataFromBuffer(const std::string &rosChannelName, std::array<char, 856> &buffer)
                {
                        

                        T_source raw_value{};
                        nlohmann::json channelIDs = getChannelIDByROSName(rosChannelName);
                        if(channelIDs.empty())
                        {
                                std::cout << "channel in this version not supported.." << rosChannelName << std::endl;
                                return raw_value;
                        }
                                
                        if(!channelIDs["channelID"].is_array())
                        {
                                genesys::DataChannel curChannel = getDataChannelByID(channelIDs["channelID"].get<int>());
                                if(curChannel.lengthBits >= 8)
                                {
                                        raw_value = extractValue<T_source>(buffer, curChannel.byteOffset, int(curChannel.lengthBits / 8));
                                        if(curChannel.scale != 1.0)
                                        {       
                                                return static_cast<T_dest>(getScaledValue(raw_value, curChannel.scale));
                                        }
                                }else{
                                        // first extract byte
                                        raw_value = extractValue<T_source>(buffer, curChannel.byteOffset, 1);
                                        if(curChannel.lengthBits == 1)
                                        {
                                                // directly extract single bit
                                                return static_cast<T_dest>(getbit(raw_value, curChannel.bitOffset));
                                        }else{
                                                // otherwise extract multiple bits
                                                return static_cast<T_dest>(getBits(raw_value, curChannel.bitOffset, curChannel.lengthBits));
                                        }
                                }
                                
                                
                                return static_cast<T_dest>(raw_value);
                        }else{
                                // std::cout << rosChannelName << " is an array of multiple channels: " << channelIDs.dump().c_str() << std::endl;
                                genesys::DataChannel curChannel = getDataChannelByID(channelIDs["channelID"].at(arrayCounter_).get<int>());
                                if(curChannel.lengthBits >= 8)
                                {
                                        if(curChannel.lengthBits == 16){
                                                raw_value = extractValue<int16_t>(buffer, curChannel.byteOffset, int(curChannel.lengthBits / 8));        
                                        }
                                        else{
                                                raw_value = extractValue<T_source>(buffer, curChannel.byteOffset, int(curChannel.lengthBits / 8));
                                        }
                                        // RCLCPP_INFO(rclcpp::get_logger("genesys::parser::Mapping"), "%s: raw: %d, Scale: %f", rosChannelName.c_str(), raw_value, curChannel.scale);
                                        if(curChannel.scale != 1.0)
                                        {       
                                                arrayCounter_++;
                                                return static_cast<T_dest>(getScaledValue(raw_value, curChannel.scale));
                                        }
                                }
                        }
                        return static_cast<T_dest>(raw_value);
                }

        private:
                uint16_t version_;
                std::map<int, genesys::DataChannel> channelMap_;
                std::map<std::string, nlohmann::json> glossarMap_;
                nlohmann::json glossar_;
                uint8_t arrayCounter_;

                void extractEntries(const nlohmann::json& j, const std::string& parentKey, std::map<std::string, nlohmann::json>& result);
                void validateProtocol();
};
}        
}
