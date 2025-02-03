#include "adma_tools_cpp/gsdb_server.hpp"

#include <arpa/inet.h>
#include <sys/socket.h>
#include <iostream>

#include <rclcpp_components/register_node_macro.hpp>
/**
 * @brief This helper class can replay raw ADMA data (GSDB file) and send it via UDP
 * to re-process the data with ROS
 */
namespace genesys
{
namespace tools
{
GSDBServer::GSDBServer(const rclcpp::NodeOptions & options)
: Node("gsdb_server", options), 
admanet_send_socket_fd_(-1), 
admanet_socket_address_(), 
admanet_address_length_(4),
admanet_msgCounter_(0),
addondelta_send_socket_fd_(-1), 
addondelta_socket_address_(), 
addondelta_address_length_(4),
addondelta_msgCounter_(0)
{
  // read ros parameters
  std::string admanet_ip_address = this->declare_parameter("admanet_ip_address", "localhost");
  admanet_port_ = this->declare_parameter("admanet_port", 1040);
  admanet_protocol_version_ = this->declare_parameter("admanet_protocol_version", "v3.3.5");

  contains_delta_ = this->declare_parameter("contains_addondelta", false);
  
  std::string addondelta_ip_address = this->declare_parameter("addondelta_ip_address", "localhost");
  addondelta_port_ = this->declare_parameter("addondelta_port", 1025);
  
  frequency_ = this->declare_parameter("frequency", 100);
  gsdbFilePath_ = declare_parameter("gsdb_file", "/home/$USER/$ROS2_WS/data/$FILENAME.gsdb");
  gsdbFile_ = std::fstream(gsdbFilePath_);
  if(gsdbFile_)
  {
    RCLCPP_INFO(get_logger(), "Loaded GSDB-File: %s", gsdbFilePath_.c_str());
  }else
  {
    RCLCPP_WARN(get_logger(), "Desired GSDB-File not found: %s", gsdbFilePath_.c_str());
  }

  RCLCPP_INFO(get_logger(), "(ADMANet) Working with: %s, publishing data at %d Hz", admanet_protocol_version_.c_str(), frequency_);
  if (admanet_protocol_version_ == "v3.2") {
    admanet_protocolLength_ = 768;
  }else{
    admanet_protocolLength_ = 856;
  }
  

  // setup socket for sending data (ADMANet)
  admanet_send_socket_fd_ = socket(AF_INET, SOCK_DGRAM | SOCK_CLOEXEC, IPPROTO_UDP);
  admanet_address_length_ = sizeof(admanet_socket_address_);
  memset((char *)&admanet_socket_address_, 0, admanet_address_length_);
  admanet_socket_address_.sin_family = AF_INET;
  admanet_socket_address_.sin_port = htons(admanet_port_);
  inet_aton(admanet_ip_address.c_str(), &(admanet_socket_address_.sin_addr));

  if(contains_delta_)
  {
    RCLCPP_INFO(get_logger(), "setup Delta channel..");
    // setup socket for sending data (AddOnDelta)
    addondelta_send_socket_fd_ = socket(AF_INET, SOCK_DGRAM | SOCK_CLOEXEC, IPPROTO_UDP);
    addondelta_address_length_ = sizeof(addondelta_socket_address_);
    memset((char *)&addondelta_socket_address_, 0, addondelta_address_length_);
    addondelta_socket_address_.sin_family = AF_INET;
    addondelta_socket_address_.sin_port = htons(addondelta_port_);
    inet_aton(addondelta_ip_address.c_str(), &(addondelta_socket_address_.sin_addr));
  }
  updateLoop();
}

GSDBServer::~GSDBServer() 
{ 
  ::shutdown(admanet_send_socket_fd_, SHUT_RDWR); 
  RCLCPP_INFO(get_logger(), "GSDB file streaming done.. Read %ld ADMANet messages from file", admanet_msgCounter_);
  if(contains_delta_)
  {
    ::shutdown(addondelta_send_socket_fd_, SHUT_RDWR);
    RCLCPP_INFO(get_logger(), "GSDB file streaming done.. Read %ld AddonDelta messages from file", addondelta_msgCounter_);
  }
}

void GSDBServer::updateLoop()
{
  //TODO: cleanup this logic (make it more efficient/C++ way..)
  char buffer[2048];
  // char temp_buffer[1024];
  while (rclcpp::ok()) {
      if (gsdbFile_.read(buffer, sizeof(buffer))) {
          size_t currentPos = 0;
          // convert UDP packet prefix to strings
          std::string delta_prefix(addondelta_start_pattern, addondelta_start_pattern + 4);
          std::string admanet_prefix(admanet_start_pattern, admanet_start_pattern + 4);
          std::string current_buffer(buffer, sizeof(buffer));

          // try to find the prefixs in the current received buffer
          std::size_t delta_index = current_buffer.find(delta_prefix);
          std::size_t admanet_index = current_buffer.find(admanet_prefix);

          if(delta_index == std::string::npos && admanet_index == std::string::npos)
          {
            RCLCPP_INFO(get_logger(), "nothing found");
          }
          if(delta_index != std::string::npos)
          {
            currentPos = delta_index;
            if(currentPos + 88 <= sizeof(buffer)){
              std::string current_delta_packet = current_buffer.substr(currentPos, currentPos + 88);
              char msg[88];
              for (size_t i = 0; i < current_delta_packet.size(); i++) {
                msg[i] = current_delta_packet[i];
              }
              ::sendto(addondelta_send_socket_fd_, (void *)(&msg), 88, 0, (struct sockaddr *)&addondelta_socket_address_, addondelta_address_length_);
              addondelta_msgCounter_++;
            }else{
              //TODO: pack buffer into temp_buffer
              RCLCPP_INFO(get_logger(), "found incomplete delta packet, using temp_buffer");
            }
          }
          if(admanet_index != std::string::npos)
          {
            currentPos = admanet_index;
            if(currentPos + admanet_protocolLength_ <= sizeof(buffer)){
              std::string current_admanet_packet = current_buffer.substr(currentPos, currentPos + admanet_protocolLength_);
              char admanet_msg[856];
              for (size_t i = 0; i < current_admanet_packet.size(); i++) {
                admanet_msg[i] = current_admanet_packet[i];
              }
              ::sendto(admanet_send_socket_fd_, (void *)(&admanet_msg), admanet_protocolLength_, 0, (struct sockaddr *)&admanet_socket_address_, admanet_address_length_);
              admanet_msgCounter_++;
            }else{
              //TODO: pack buffer into temp_buffer
              RCLCPP_INFO(get_logger(), "found incomplete admanet packet, using temp_buffer");
            }
          }
      }else{
        rclcpp::shutdown();
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(1000 / frequency_));
      gsdbFile_.seekg((admanet_msgCounter_ * admanet_protocolLength_) + (addondelta_msgCounter_ * 88));
  }

  
}
}  // end namespace tools
}  // end namespace genesys
RCLCPP_COMPONENTS_REGISTER_NODE(genesys::tools::GSDBServer)
