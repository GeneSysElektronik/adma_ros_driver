#include <netdb.h>
#include <fstream>
#include <memory>
#include <rclcpp/rclcpp.hpp>

namespace genesys
{
namespace tools
{
class GSDBServer : public rclcpp::Node
{
public:
  explicit GSDBServer(const rclcpp::NodeOptions & options);
  virtual ~GSDBServer();

private:
  void updateLoop();

  // ADMANet specific 
  int admanet_send_socket_fd_;
  struct sockaddr_in admanet_socket_address_;
  socklen_t admanet_address_length_;
  int admanet_port_;
  std::string admanet_protocol_version_;
  unsigned long admanet_msgCounter_;
  unsigned long admanet_protocolLength_;
  char admanet_start_pattern[4] = {0x47, 0x42, 0x49, 0x4E}; //GBIN

  // AddOnDelta specific 
  int addondelta_send_socket_fd_;
  struct sockaddr_in addondelta_socket_address_;
  socklen_t addondelta_address_length_;
  int addondelta_port_;
  unsigned long addondelta_msgCounter_;
  unsigned long addondelta_protocolLength_;
  unsigned char addondelta_start_pattern[4] = {0xC0, 0xB5, 0x3E, 0x70};
  
  bool contains_delta_;
  unsigned short frequency_;
  std::string gsdbFilePath_;
  std::fstream gsdbFile_;
  
};
}  // end namespace tools
}  // end namespace genesys
