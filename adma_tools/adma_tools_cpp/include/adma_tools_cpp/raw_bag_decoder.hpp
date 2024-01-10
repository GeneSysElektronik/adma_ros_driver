#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <adma_ros_driver_msgs/msg/adma_data_scaled.hpp>
#include <adma_ros_driver_msgs/msg/adma_data_raw.hpp>
#include <adma_ros_driver_msgs/msg/adma_status.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/float64.hpp>
#include <adma_ros2_driver/parser/adma2ros_parser.hpp>

namespace genesys
{
namespace tools
{
class RawBagDecoder : public rclcpp::Node
{
public:
  explicit RawBagDecoder(const rclcpp::NodeOptions & options);
  virtual ~RawBagDecoder();

private:
  void rawDataCallback(adma_ros_driver_msgs::msg::AdmaDataRaw::SharedPtr rawDataMsg);

  rclcpp::Subscription<adma_ros_driver_msgs::msg::AdmaDataRaw>::SharedPtr subRawData_;
  rclcpp::Publisher<adma_ros_driver_msgs::msg::AdmaDataScaled>::SharedPtr pub_adma_data_scaled_;
  rclcpp::Publisher<adma_ros_driver_msgs::msg::AdmaStatus>::SharedPtr pub_adma_status_;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr pub_navsat_fix_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_heading_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_velocity_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odometry_;

  ADMA2ROSParser * parser_;

  // frame_ids for the ros msgs
  std::string gnss_frame_;
  std::string imu_frame_;
  std::string adma_frame_;
  std::string adma_status_frame_;
  std::string raw_data_frame_;
  std::string odometry_pose_frame_;
  std::string odometry_child_frame_;

  // yaw offset angle if the odometry should be rotated by a fixed angle
  double odometry_yaw_offset_;

  // desired data sources per topic (POI_x or MRP)
  uint8_t navsatfix_id_;
  uint8_t imu_id_;
  uint8_t velocity_id_;
  uint8_t odometry_id_;
  std::array<adma_ros_driver_msgs::msg::POI, 8> pois;

  // parameter to set time source
  uint8_t time_mode_;
};
}  // end namespace tools
}  // end namespace genesys
