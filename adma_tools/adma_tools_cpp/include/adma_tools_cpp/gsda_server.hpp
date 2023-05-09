#include <netdb.h>
#include <fstream>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <adma_ros_driver_msgs/msg/adma_data_scaled.hpp>
#include <adma_ros_driver_msgs/msg/adma_status.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/float64.hpp>
#include <adma_ros2_driver/parser/adma2ros_parser.hpp>

namespace genesys
{
namespace tools
{
class GSDAServer : public rclcpp::Node
{
public:
  explicit GSDAServer(const rclcpp::NodeOptions & options);
  virtual ~GSDAServer();

private:
  void updateLoop();
  void fillDataScaledMsg(adma_ros_driver_msgs::msg::AdmaDataScaled& dataScaledMsg);
  void readLine();
  void extractHeader();
  double readValue(std::string dataName);
  void extractBytes(adma_ros_driver_msgs::msg::AdmaStatus &stateMsg, adma_ros_driver_msgs::msg::AdmaDataScaled& dataScaledMsg);

  unsigned short frequency_;

  std::string gsdaFilePath_;
  std::fstream gsdaFile_;
  unsigned long msgCounter_;

  std::vector<std::vector<std::string>> content;
	std::vector<std::string> row;
	std::string line, word;
  std::map<std::string, int> indexMap_;

  rclcpp::Publisher<adma_ros_driver_msgs::msg::AdmaDataScaled>::SharedPtr pub_adma_data_scaled_;
  rclcpp::Publisher<adma_ros_driver_msgs::msg::AdmaStatus>::SharedPtr pub_adma_status_;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr pub_navsat_fix_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_heading_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_velocity_;

  ADMA2ROSParser * parser_;
};
}  // end namespace tools
}  // end namespace genesys
