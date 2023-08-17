#include <netdb.h>
#include <fstream>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <adma_ros_driver_msgs/msg/adma_data_scaled.hpp>
#include <adma_ros_driver_msgs/msg/adma_status.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/float64.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <tf2_ros/transform_broadcaster.h>
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
  int readByteValue(std::string dataName);
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
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odometry_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_trajectory_;

  // frame_ids for the ros msgs
  std::string gnss_frame_;
  std::string imu_frame_;
  std::string adma_frame_;
  std::string adma_status_frame_;
  std::string raw_data_frame_;
  std::string odometry_pose_frame_;
  std::string odometry_child_frame_;

  ADMA2ROSParser * parser_;

  // yaw offset angle if the odometry should be rotated by a fixed angle
  double odometry_yaw_offset_;

  /// Tools for broadcasting TFs.
  bool publish_TF_;
  tf2_ros::TransformBroadcaster tfBroadcaster_;
  nav_msgs::msg::Path trajectory_msg_;

  // desired data sources per topic (POI_x or MRP)
  uint8_t navsatfix_id_;
  uint8_t imu_id_;
  uint8_t velocity_id_;
  uint8_t odometry_id_;
  std::array<adma_ros_driver_msgs::msg::POI, 8> pois;
};
}  // end namespace tools
}  // end namespace genesys
