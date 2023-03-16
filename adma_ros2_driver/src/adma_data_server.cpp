#include "adma_ros2_driver/adma_data_server.hpp"

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
DataServer::DataServer(const rclcpp::NodeOptions & options)
: Node("data_server", options), 
send_socket_fd_(-1), 
socket_address_(), 
address_length_(4),
msgCounter_(0)
{
  // read ros parameters
  frequency_ = this->declare_parameter("frequency", 100);
  std::string ip_address = this->declare_parameter("ip_address", "localhost");
  port_ = this->declare_parameter("port", 1040);
  protocol_version_ = this->declare_parameter("protocol_version", "v3.3.4");
  gsdbFilePath_ = declare_parameter("gsdb_file", "/home/rschilli/Documents/GeneSys/rosbags_ros2/ROS2_Arbeitsplatz/raw_data.gsdb");
  gsdbFile_ = std::fstream(gsdbFilePath_);
  if(gsdbFile_)
  {
    RCLCPP_INFO(get_logger(), "Loaded GSDB-File: %s", gsdbFilePath_.c_str());
  }else
  {
    RCLCPP_WARN(get_logger(), "Desired GSDB-File not found: %s", gsdbFilePath_.c_str());
  }

  RCLCPP_INFO(get_logger(), "Working with: %s, publishing data at %d Hz", protocol_version_.c_str(), frequency_);
  if (protocol_version_ == "v3.2") {
    protocolLength_ = 768;
  }else{
    protocolLength_ = 856;
  }
  

  // setup socket for sending data
  send_socket_fd_ = socket(AF_INET, SOCK_DGRAM | SOCK_CLOEXEC, IPPROTO_UDP);
  address_length_ = sizeof(socket_address_);
  memset((char *)&socket_address_, 0, address_length_);
  socket_address_.sin_family = AF_INET;
  socket_address_.sin_port = htons(port_);
  inet_aton(ip_address.c_str(), &(socket_address_.sin_addr));

  updateLoop();
}

DataServer::~DataServer() 
{ 
  ::shutdown(send_socket_fd_, SHUT_RDWR); 
  RCLCPP_INFO(get_logger(), "GSDB file streaming done.. Read %ld messages from file", msgCounter_);
}

void DataServer::updateLoop()
{
  char buffer[856];
  while (rclcpp::ok()) {
    if(gsdbFile_.read(buffer, protocolLength_)){
      ::sendto(send_socket_fd_, (void *)(&buffer), protocolLength_, 0, (struct sockaddr *)&socket_address_, address_length_);
    }else{
      rclcpp::shutdown();
    }
      
    std::this_thread::sleep_for(std::chrono::milliseconds(1000 / frequency_));
    msgCounter_++;
    gsdbFile_.seekg(msgCounter_ * protocolLength_);
  }
  
}
}  // end namespace genesys
RCLCPP_COMPONENTS_REGISTER_NODE(genesys::DataServer)
