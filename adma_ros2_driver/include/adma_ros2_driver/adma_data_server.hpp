#include <netdb.h>

#include <memory>
#include <rclcpp/rclcpp.hpp>

namespace genesys
{
class DataServer : public rclcpp::Node
{
public:
  explicit DataServer(const rclcpp::NodeOptions & options);
  virtual ~DataServer();

private:
  void updateLoop();

  int send_socket_fd_;
  struct sockaddr_in socket_address_;
  socklen_t address_length_;
  int port_;
  unsigned short frequency_;
  std::string protocol_version_;
};
}  // end namespace genesys
