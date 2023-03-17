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

  int send_socket_fd_;
  struct sockaddr_in socket_address_;
  socklen_t address_length_;
  int port_;
  unsigned short frequency_;
  std::string protocol_version_;

  std::string gsdbFilePath_;
  std::fstream gsdbFile_;
  unsigned long msgCounter_;
  unsigned long protocolLength_;
};
}  // end namespace tools
}  // end namespace genesys
