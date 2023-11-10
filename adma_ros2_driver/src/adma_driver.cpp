#include "adma_ros2_driver/adma_driver.hpp"

#include <arpa/inet.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <sys/types.h>

#include <rclcpp_components/register_node_macro.hpp>

#include "adma_ros2_driver/parser/parser_utils.hpp"

namespace genesys
{
ADMADriver::ADMADriver(const rclcpp::NodeOptions & options)
: Node("adma_driver", options),
  rcv_sock_fd_(-1),
  rcv_addr_info_(NULL),
  adma_address_(),
  adma_address_length_(4),
  adma_port_(0)
{
  std::string param_address = this->declare_parameter("destination_ip", "0.0.0.0");
  adma_port_ = this->declare_parameter("destination_port", 1040);

  performance_check_ = this->declare_parameter("use_performance_check", false);
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
  
  // define protocol specific stuff
  protocol_version_ = this->declare_parameter("protocol_version", "v3.3.3");
  RCLCPP_INFO(get_logger(), "Working with: %s", protocol_version_.c_str());
  if (protocol_version_ == "v3.2") {
    len_ = 768;
    pub_adma_data_ = this->create_publisher<adma_ros_driver_msgs::msg::AdmaData>("adma/data", 1);
  } else if (protocol_version_ == "v3.3.3") {
    len_ = 856;
    pub_adma_data_ = this->create_publisher<adma_ros_driver_msgs::msg::AdmaData>("adma/data", 1);
    pub_adma_data_raw_ =
      this->create_publisher<adma_ros_driver_msgs::msg::AdmaDataRaw>("adma/data_raw", 1);
  } else if (protocol_version_ == "v3.3.4") {
    len_ = 856;
    pub_adma_data_raw_ =
      this->create_publisher<adma_ros_driver_msgs::msg::AdmaDataRaw>("adma/data_raw", 1);
    pub_adma_data_scaled_ =
      this->create_publisher<adma_ros_driver_msgs::msg::AdmaDataScaled>("adma/data_scaled", 1);
    pub_adma_status_ =
      this->create_publisher<adma_ros_driver_msgs::msg::AdmaStatus>("adma/status", 1);
    pub_odometry_ =
      this->create_publisher<nav_msgs::msg::Odometry>("adma/odometry", 1);

  }
  parser_ = new ADMA2ROSParser(protocol_version_);

  pub_navsat_fix_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("adma/fix", 1);
  pub_imu_ = this->create_publisher<sensor_msgs::msg::Imu>("adma/imu", 1);
  pub_heading_ = this->create_publisher<std_msgs::msg::Float64>("adma/heading", 1);
  pub_velocity_ = this->create_publisher<std_msgs::msg::Float64>("adma/velocity", 1);

  initializeUDP(param_address);
  updateLoop();
}

ADMADriver::~ADMADriver()
{
  freeaddrinfo(rcv_addr_info_);
  ::shutdown(rcv_sock_fd_, SHUT_RDWR);
  rcv_sock_fd_ = -1;
}

void ADMADriver::initializeUDP(std::string adma_address)
{
  struct addrinfo hints;
  memset(&hints, 0, sizeof(hints));
  hints.ai_family = AF_UNSPEC;
  hints.ai_socktype = SOCK_DGRAM;
  hints.ai_protocol = IPPROTO_UDP;
  std::string rcv_port_str = std::to_string(adma_port_);

  adma_address_length_ = sizeof(adma_address_);
  memset((char *)&adma_address_, 0, adma_address_length_);
  adma_address_.sin_family = AF_INET;
  adma_address_.sin_port = htons(adma_port_);
  inet_aton(adma_address.c_str(), &(adma_address_.sin_addr));

  int r = getaddrinfo(adma_address.c_str(), rcv_port_str.c_str(), &hints, &rcv_addr_info_);
  if (r != 0 || rcv_addr_info_ == NULL) {
    RCLCPP_FATAL(
      get_logger(), "Invalid port for UDP socket: \"%s:%s\"", adma_address.c_str(),
      rcv_port_str.c_str());
    throw rclcpp::exceptions::InvalidParameterValueException(
      "Invalid port for UDP socket: \"" + adma_address + ":" + rcv_port_str + "\"");
  }
  rcv_sock_fd_ = socket(rcv_addr_info_->ai_family, SOCK_DGRAM | SOCK_CLOEXEC, IPPROTO_UDP);
  if (rcv_sock_fd_ == -1) {
    freeaddrinfo(rcv_addr_info_);
    RCLCPP_FATAL(
      get_logger(), "Could not create UDP socket for: \"%s:%s", adma_address.c_str(),
      rcv_port_str.c_str());
    throw rclcpp::exceptions::InvalidParameterValueException(
      "Could not create UDP socket for: \"" + adma_address + ":" + rcv_port_str + "\"");
  }
  r = bind(rcv_sock_fd_, rcv_addr_info_->ai_addr, rcv_addr_info_->ai_addrlen);
  if (r != 0) {
    freeaddrinfo(rcv_addr_info_);
    ::shutdown(rcv_sock_fd_, SHUT_RDWR);
    RCLCPP_FATAL(
      get_logger(), "Could not bind UDP socket with: \"%s:%s", adma_address.c_str(),
      rcv_port_str.c_str());
    throw rclcpp::exceptions::InvalidParameterValueException(
      "Could not bind UDP socket with: \"" + adma_address + ":" + rcv_port_str + "\"");
  }
}

void ADMADriver::parseData(std::array<char, 856> recv_buf)
{
  // prepare several ros msgs
  sensor_msgs::msg::NavSatFix message_fix;
  message_fix.header.frame_id = imu_frame_;
  std_msgs::msg::Float64 message_heading;
  std_msgs::msg::Float64 message_velocity;
  sensor_msgs::msg::Imu message_imu;
  message_imu.header.frame_id = imu_frame_;
  float weektime;
  //offset between UNIX and GNSS (in ms)
  unsigned long long offset_gps_unix = 315964800000;
  unsigned long long week_to_msec = 604800000;
  unsigned long long timestamp;

  // read Adma msg from UDP data packet
  if (protocol_version_ == "v3.2" || protocol_version_ == "v3.3.3") {
    adma_ros_driver_msgs::msg::AdmaData admaData_ros_msg;
    parser_->mapAdmaMessageToROS(admaData_ros_msg, recv_buf);
    timestamp = admaData_ros_msg.instimemsec + offset_gps_unix;
    timestamp += admaData_ros_msg.instimeweek * week_to_msec;
    admaData_ros_msg.timemsec = timestamp;
    admaData_ros_msg.timensec = timestamp * 1E6;

    // read NavSatFix out of AdmaData
    parser_->extractNavSatFix(admaData_ros_msg, message_fix);

    // read heading and velocity
    message_heading.data = admaData_ros_msg.finsyaw;
    message_velocity.data =
      std::sqrt(
        std::pow(admaData_ros_msg.fgpsvelframex, 2) + std::pow(admaData_ros_msg.fgpsvelframey, 2)) *
      3.6;

    // read IMU
    parser_->extractIMU(admaData_ros_msg, message_imu);
    admaData_ros_msg.header.stamp.sec = timestamp / 1000;
    admaData_ros_msg.header.stamp.nanosec = timestamp * 1E6;
    pub_adma_data_->publish(admaData_ros_msg);
    weektime = admaData_ros_msg.instimeweek;
  } else if (protocol_version_ == "v3.3.4") {
    AdmaDataV334 data_struct;
    memcpy(&data_struct, &recv_buf, sizeof(data_struct));
    adma_ros_driver_msgs::msg::AdmaDataScaled adma_data_scaled_msg;
    adma_data_scaled_msg.header.frame_id = adma_frame_;
    parser_->parseV334(adma_data_scaled_msg, data_struct);
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
    adma_data_scaled_msg.header.stamp.sec = timestamp / 1000;
    adma_data_scaled_msg.header.stamp.nanosec = timestamp * 1E6;

    parser_->extractNavSatFix(adma_data_scaled_msg, message_fix, pois, navsatfix_id_);
    parser_->extractIMU(adma_data_scaled_msg, message_imu, pois, imu_id_);

    // fill odometry message
    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.frame_id = odometry_pose_frame_;
    odom_msg.child_frame_id = odometry_child_frame_;
    odom_msg.header.stamp.sec = timestamp / 1000;
    odom_msg.header.stamp.nanosec = timestamp * 1E6;
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
    status_msg.header.stamp.sec = timestamp / 1000;
    status_msg.header.stamp.nanosec = timestamp * 1E6;
    status_msg.header.frame_id = adma_status_frame_;
    parser_->parseV334Status(status_msg, data_struct);
    pub_adma_status_->publish(status_msg);
  }

  // publish raw data with >= v3.3.3
  if (protocol_version_ != "v3.2") {
    // publish raw data as byte array
    adma_ros_driver_msgs::msg::AdmaDataRaw raw_data_msg;
    raw_data_msg.size = len_;
    raw_data_msg.header.stamp.sec = timestamp / 1000;
    raw_data_msg.header.stamp.nanosec = timestamp * 1E6;
    raw_data_msg.header.frame_id = raw_data_frame_;

    for (int i = 0; i < len_; ++i) {
      raw_data_msg.raw_data.push_back(recv_buf[i]);
    }
    pub_adma_data_raw_->publish(raw_data_msg);
  }

  // publish the messages
  message_fix.header.stamp.sec = timestamp / 1000;
  message_fix.header.stamp.nanosec = timestamp * 1E6;
  message_imu.header.stamp.sec = timestamp / 1000;
  message_imu.header.stamp.nanosec = timestamp * 1E6;
  pub_navsat_fix_->publish(message_fix);
  pub_heading_->publish(message_heading);
  pub_velocity_->publish(message_velocity);
  pub_imu_->publish(message_imu);

  double grab_time = this->get_clock()->now().seconds();

  if (performance_check_) {
    RCLCPP_INFO(get_logger(), "%f ", ((grab_time * 1000) - (timestamp)));
  }
}

void ADMADriver::updateLoop()
{
  fd_set s;
  struct timeval timeout;
  // struct sockaddr src_addr;
  // socklen_t src_addr_len;

  std::array<char, 856> recv_buf;

  while (rclcpp::ok()) {
    // check if new data is available
    FD_ZERO(&s);
    FD_SET(rcv_sock_fd_, &s);
    timeout.tv_sec = 1;
    timeout.tv_usec = 0;
    int ret = select(rcv_sock_fd_ + 1, &s, NULL, NULL, &timeout);
    if (ret == 0) {
      // reached timeout
      RCLCPP_INFO(get_logger(), "Waiting for ADMA data...");
      continue;
    } else if (ret == -1) {
      // error
      RCLCPP_WARN(get_logger(), "Select-error: %s", strerror(errno));
      continue;
    }

    ret = ::recv(rcv_sock_fd_, (void *)(&recv_buf), len_, 0);
    if (ret < 0) {
      RCLCPP_WARN(get_logger(), "Receive-error: %s", strerror(errno));
      continue;
    } else if (ret != len_) {
      RCLCPP_WARN(get_logger(), "Invalid ADMA message size: %d instead of %ld", ret, len_);
      continue;
    }

    parseData(recv_buf);
  }
}
}  // namespace genesys

RCLCPP_COMPONENTS_REGISTER_NODE(genesys::ADMADriver)
