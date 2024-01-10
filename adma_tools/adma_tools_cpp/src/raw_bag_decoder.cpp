#include "adma_tools_cpp/raw_bag_decoder.hpp"

#include <string>

#include <rclcpp_components/register_node_macro.hpp>
#include <adma_ros2_driver/parser/parser_utils.hpp>

/**
 * @brief This helper class can replay post-processed ADMA data (GSDA file) and publish the data to ROS
 */
namespace genesys
{
namespace tools
{
RawBagDecoder::RawBagDecoder(const rclcpp::NodeOptions & options)
: Node("raw_bag_decoder", options)
{
  // read ros parameters
  gnss_frame_ = this->declare_parameter("frame_ids.navsatfix", "gnss_link");
  imu_frame_ = this->declare_parameter("frame_ids.imu", "imu_link");
  adma_frame_ = this->declare_parameter("frame_ids.adma", "adma");
  adma_status_frame_ = this->declare_parameter("frame_ids.adma_status", "adma_status");
  raw_data_frame_ = this->declare_parameter("frame_ids.raw_data", "data_raw");
  odometry_pose_frame_ = this->declare_parameter("frame_ids.odometry_pose_id", "adma");
  odometry_child_frame_ = this->declare_parameter("frame_ids.odometry_twist_id", "odometry");
  odometry_yaw_offset_ = this->declare_parameter("odometry_yaw_offset", 0.0);
  navsatfix_id_ = this->declare_parameter("topic_pois.navsatfix", 1);
  imu_id_ = this->declare_parameter("topic_pois.imu", 1);
  velocity_id_ = this->declare_parameter("topic_pois.velocity", 1);
  odometry_id_ = this->declare_parameter("topic_pois.odometry", 1);
  time_mode_ = this->declare_parameter("time_mode", 0); // 0 / 1 / 2

  subRawData_ = create_subscription<adma_ros_driver_msgs::msg::AdmaDataRaw>(
    "/genesys/adma/data_raw", 10, std::bind(&RawBagDecoder::rawDataCallback,
    this, std::placeholders::_1));

  pub_adma_data_scaled_ =
    this->create_publisher<adma_ros_driver_msgs::msg::AdmaDataScaled>("adma/data_scaled", 1);
  pub_adma_status_ =
    this->create_publisher<adma_ros_driver_msgs::msg::AdmaStatus>("adma/status", 1);
  pub_navsat_fix_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("adma/fix", 1);
  pub_imu_ = this->create_publisher<sensor_msgs::msg::Imu>("adma/imu", 1);
  pub_heading_ = this->create_publisher<std_msgs::msg::Float64>("adma/heading", 1);
  pub_velocity_ = this->create_publisher<std_msgs::msg::Float64>("adma/velocity", 1);
  pub_odometry_ =
      this->create_publisher<nav_msgs::msg::Odometry>("adma/odometry", 1);

  //TODO: may make this injectable for several protocol version support
  parser_ = new ADMA2ROSParser("v3.3.4");
}

RawBagDecoder::~RawBagDecoder() 
{ 
  
}

void RawBagDecoder::rawDataCallback(adma_ros_driver_msgs::msg::AdmaDataRaw::SharedPtr newMsg)
{
  // first convert the received raw ROS msg into byte array for easier parsing afterwards
  std::array<char, 856> recv_buf;
  for(size_t i = 0; i < newMsg->size; i++) 
  {
    recv_buf[i] = newMsg->raw_data[i];
  }

  // prepare several ros msgs
  sensor_msgs::msg::NavSatFix message_fix;
  message_fix.header.frame_id = imu_frame_;
  std_msgs::msg::Float64 message_heading;
  std_msgs::msg::Float64 message_velocity;
  sensor_msgs::msg::Imu message_imu;
  message_imu.header.frame_id = imu_frame_;

  builtin_interfaces::msg::Time timestampForMsgs;

  float weektime;
  //offset between UNIX and GNSS (in ms)
  unsigned long long offset_gps_unix = 315964800000;
  unsigned long long week_to_msec = 604800000;
  unsigned long long timestamp;

  AdmaDataV334 data_struct;
  memcpy(&data_struct, &recv_buf, sizeof(data_struct));
  adma_ros_driver_msgs::msg::AdmaDataScaled adma_data_scaled_msg;
  adma_data_scaled_msg.header.frame_id = adma_frame_;
  parser_->parseV334(adma_data_scaled_msg, data_struct);
  // define POI-list for publishing odometry
  pois = {
    adma_data_scaled_msg.poi_1,
    adma_data_scaled_msg.poi_2,
    adma_data_scaled_msg.poi_3,
    adma_data_scaled_msg.poi_4,
    adma_data_scaled_msg.poi_5,
    adma_data_scaled_msg.poi_6,
    adma_data_scaled_msg.poi_7,
    adma_data_scaled_msg.poi_8
  };
  timestamp = adma_data_scaled_msg.ins_time_msec + offset_gps_unix;
  timestamp += adma_data_scaled_msg.ins_time_week * week_to_msec;
  adma_data_scaled_msg.time_msec = timestamp;
  adma_data_scaled_msg.time_nsec = timestamp * 1E6;

  if(time_mode_ == 0)
  {
    // mode == 0 -> use ADMA time 
    timestampForMsgs.sec = timestamp / 1000;
    timestampForMsgs.nanosec = timestamp * 1E6;
  }else if(time_mode_ == 1)
  {
    // mode == 1 -> use current ROS system time
    timestampForMsgs = get_clock()->now();
  } else if(time_mode_ == 2)
  {
    // mode == 2 -> use time from external ROS time topic
    //TODO: subscribe to a ROS time topic to set timestampFosMsgs
  }
  adma_data_scaled_msg.header.stamp = timestampForMsgs;

  parser_->extractNavSatFix(adma_data_scaled_msg, message_fix, pois, navsatfix_id_);
  parser_->extractIMU(adma_data_scaled_msg, message_imu, pois, imu_id_);

  // fill odometry message
  nav_msgs::msg::Odometry odom_msg;
  odom_msg.header.frame_id = odometry_pose_frame_;
  odom_msg.child_frame_id = odometry_child_frame_;
  odom_msg.header.stamp = timestampForMsgs;
  parser_->extractOdometry(adma_data_scaled_msg, odom_msg, odometry_yaw_offset_, pois, odometry_id_);
  pub_odometry_->publish(odom_msg);

  // read heading and velocity
  message_heading.data = adma_data_scaled_msg.ins_yaw;
  geometry_msgs::msg::Vector3 insSource = velocity_id_ == 0 
    ? adma_data_scaled_msg.ins_vel_frame 
    : pois[velocity_id_ - 1].ins_vel_hor;
  message_velocity.data = std::sqrt(std::pow(insSource.x, 2) + std::pow(insSource.y, 2)) * 3.6;

  pub_adma_data_scaled_->publish(adma_data_scaled_msg);

  weektime = adma_data_scaled_msg.ins_time_week;

  adma_ros_driver_msgs::msg::AdmaStatus status_msg;
  status_msg.header.stamp = timestampForMsgs;
  status_msg.header.frame_id = adma_status_frame_;
  parser_->parseV334Status(status_msg, data_struct);
  pub_adma_status_->publish(status_msg);

  // publish the messages
  message_fix.header.stamp = timestampForMsgs;
  message_imu.header.stamp = timestampForMsgs;
  pub_navsat_fix_->publish(message_fix);
  pub_heading_->publish(message_heading);
  pub_velocity_->publish(message_velocity);
  pub_imu_->publish(message_imu);
}

}  // end namespace tools
}  // end namespace genesys
RCLCPP_COMPONENTS_REGISTER_NODE(genesys::tools::RawBagDecoder)
