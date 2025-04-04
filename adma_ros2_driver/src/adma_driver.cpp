#include "adma_ros2_driver/adma_driver.hpp"

#include <rclcpp_components/register_node_macro.hpp>
#include "adma_core_lib/parser/parser_utils.hpp"
#include "adma_core_lib/parser/mapping.hpp"
#include <adma_ros_driver_msgs/msg/admanet_header.hpp>

namespace genesys
{
ADMADriver::ADMADriver(const rclcpp::NodeOptions & options)
: Node("adma_driver", options)
{
  // define ROS parameters, adjustable by config yaml file
  std::string param_address = this->declare_parameter("destination_ip", "0.0.0.0");
  int adma_port = this->declare_parameter("destination_port", 1040);
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
  mode_ = this->declare_parameter("mode", 0); // 0 / 1
  time_mode_ = this->declare_parameter("time_mode", 0); // 0 / 1
  publish_clock_ = this->declare_parameter("publish_clock", false);
  
  // setup publisher that are protocol version indepent
  pub_navsat_fix_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("adma/fix", 1);
  pub_imu_ = this->create_publisher<sensor_msgs::msg::Imu>("adma/imu", 1);
  pub_heading_ = this->create_publisher<std_msgs::msg::Float64>("adma/heading", 1);
  pub_velocity_ = this->create_publisher<std_msgs::msg::Float64>("adma/velocity", 1);

  if (mode_ == 1)
      {
        RCLCPP_INFO(get_logger(), "Starting in rosbag replay mode..");
        subRawData_ = create_subscription<adma_ros_driver_msgs::msg::AdmaDataRaw>(
        "adma/data_raw", 10, std::bind(&ADMADriver::rawDataCallback,
        this, std::placeholders::_1));
      }
  if(publish_clock_)
    {
      pub_clock_ = this->create_publisher<rosgraph_msgs::msg::Clock>("/clock", 1);
    }

  if(mode_ == 0)
  {
    socket_ = new genesys::core::UDPSocket(len_);
    socket_->setupReceiveSocket(param_address, adma_port);
    // only setup UDP connection and loop in live mode
    updateLoop();
  }
  
}

ADMADriver::~ADMADriver()
{
  // unlock socket when stopping application
  if(mode_ == 0)
  {
    socket_->~UDPSocket();
  }
}

void ADMADriver::rawDataCallback(adma_ros_driver_msgs::msg::AdmaDataRaw::SharedPtr newMsg)
{
  // first convert the received raw ROS msg into byte array for easier parsing afterwards
  std::array<char, 856> recv_buf;
  for(size_t i = 0; i < newMsg->size; i++) 
  {
    recv_buf[i] = newMsg->raw_data[i];
  }
  parseData(recv_buf);
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

  builtin_interfaces::msg::Time timestampForMsgs;
  float weektime;
  //offset between UNIX and GNSS (in ms)
  unsigned long long offset_gps_unix = 315964800000;
  unsigned long long week_to_msec = 604800000;
  unsigned long long timestamp;

  // first extract admanet header to get the protocol version
  adma_ros_driver_msgs::msg::AdmanetHeader admaHeaderMsg;
  extractAdmanetHeader(admaHeaderMsg, recv_buf);
  

  if(!setupDone)
  {
    RCLCPP_INFO(get_logger(), "Receiving Admanet version: %d", admaHeaderMsg.format_version);
    if(admaHeaderMsg.format_version == 3200)
    {
      // for version 3.2 we use the old message format
      pub_adma_data_ = this->create_publisher<adma_ros_driver_msgs::msg::AdmaData>("adma/data", 1);
    }else{
      // depending on live/replay mode create publisher/subscriber for raw data
      if(mode_ == 0)
      {
        RCLCPP_INFO(get_logger(), "Starting in live mode..");
        pub_adma_data_raw_ =
        this->create_publisher<adma_ros_driver_msgs::msg::AdmaDataRaw>("adma/data_raw", 1);
      }else if (mode_ == 1)
      {
        RCLCPP_INFO(get_logger(), "Starting in rosbag replay mode..");
        subRawData_ = create_subscription<adma_ros_driver_msgs::msg::AdmaDataRaw>(
        "adma/data_raw", 10, std::bind(&ADMADriver::rawDataCallback,
        this, std::placeholders::_1));
      }
      // setup publisher for all newer versions (>= 3.3.3)
      pub_adma_data_scaled_ =
      this->create_publisher<adma_ros_driver_msgs::msg::AdmaDataScaled>("adma/data_scaled", 1);
      pub_adma_status_ =
        this->create_publisher<adma_ros_driver_msgs::msg::AdmaStatus>("adma/status", 1);
      pub_odometry_ =
        this->create_publisher<nav_msgs::msg::Odometry>("adma/odometry", 1);
    }
    // setup parser and finish setup
    parser_ = new ADMA2ROSParser(admaHeaderMsg.format_version);
    setupDone = true;
  }

  // if(format_version == "3200"){

  // }

  // read Adma msg from UDP data packet
  if (admaHeaderMsg.format_version == 3200) {
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
    admaData_ros_msg.header.stamp.nanosec = (timestamp % 1000) * 1E6;
    pub_adma_data_->publish(admaData_ros_msg);
    weektime = admaData_ros_msg.instimeweek;

    // double oldHeading = message_heading.data;
    //   newParser_->extractHeading(message_heading, recv_buf);
    //   adma_ros_driver_msgs::msg::AdmaStatus status_msg_new;
    //   newParser_->extractAdmaStatus(status_msg_new, recv_buf);
      // RCLCPP_INFO(get_logger(), "statusbyte_0 old/new: %d / %d", status_msg.status_bytes.status_byte_0, status_msg_new.status_bytes.status_byte_0);
      // RCLCPP_INFO(get_logger(), "error_gyro_hw old/new: %d / %d", status_msg.error_warnings.error_gyro_hw, status_msg_new.error_warnings.error_gyro_hw);
  } else {
    // AdmaDataV334 data_struct;
    // memcpy(&data_struct, &recv_buf, sizeof(data_struct));
    adma_ros_driver_msgs::msg::AdmaDataScaled adma_data_scaled_msg;
    adma_data_scaled_msg.adma_header = admaHeaderMsg;
    adma_ros_driver_msgs::msg::AdmaStatus status_msg;
    parser_->extractAdmaDataScaled(adma_data_scaled_msg, recv_buf);
    parser_->extractPOIs(adma_data_scaled_msg, recv_buf);
    parser_->extractHeading(message_heading, recv_buf);
    parser_->extractAdmaStatus(status_msg, recv_buf);
    adma_data_scaled_msg.status = status_msg.status;

    adma_data_scaled_msg.header.frame_id = adma_frame_;
    // parser_->parseV334(adma_data_scaled_msg, data_struct);
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
    adma_data_scaled_msg.time_nsec = (timestamp % 1000) * 1E6;

    if(time_mode_ == 0)
    {
      // mode == 0 -> use ADMA time 
      timestampForMsgs.sec = timestamp / 1000;
      timestampForMsgs.nanosec = (timestamp % 1000) * 1E6;
    }else if(time_mode_ == 1)
    {
      // mode == 1 -> use current ROS system time
      timestampForMsgs = get_clock()->now();
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

    
    status_msg.header.stamp = timestampForMsgs;
    status_msg.header.frame_id = adma_status_frame_;
    // parser_->parseV334Status(status_msg, data_struct);
    pub_adma_status_->publish(status_msg);

    // double oldHeading = message_heading.data;
      // newParser_->extractHeading(message_heading, recv_buf);
      // adma_ros_driver_msgs::msg::AdmaStatus status_msg_new;
      // newParser_->extractAdmaStatus(status_msg_new, recv_buf);
      // RCLCPP_INFO(get_logger(), "statusbyte_0 old/new: %d / %d", status_msg.status_bytes.status_byte_0, status_msg_new.status_bytes.status_byte_0);
      // RCLCPP_INFO(get_logger(), "error_gyro_hw old/new: %d / %d", status_msg.error_warnings.error_gyro_hw, status_msg_new.error_warnings.error_gyro_hw);

      // kind of a "hack" to ensure clock is only published if INS time is valid
      if(adma_data_scaled_msg.ins_time_week > 0 && publish_clock_){
        rosgraph_msgs::msg::Clock clockMsg;
        clockMsg.clock = timestampForMsgs;
        pub_clock_->publish(clockMsg);
      }

      if(mode_ == 0)
      {
        // publish raw data as byte array
      adma_ros_driver_msgs::msg::AdmaDataRaw raw_data_msg;
      raw_data_msg.size = len_;
      raw_data_msg.header.stamp  = timestampForMsgs;
      raw_data_msg.header.frame_id = raw_data_frame_;

      for (int i = 0; i < len_; ++i) {
        raw_data_msg.raw_data.push_back(recv_buf[i]);
      }
      pub_adma_data_raw_->publish(raw_data_msg);
      }
  }

  
  // publish the messages
  message_fix.header.stamp = timestampForMsgs;
  message_imu.header.stamp = timestampForMsgs;
  pub_navsat_fix_->publish(message_fix);
  pub_heading_->publish(message_heading);
  pub_velocity_->publish(message_velocity);
  pub_imu_->publish(message_imu);

  // just for debugging
  if (performance_check_) {
    double grab_time = this->get_clock()->now().seconds();
    RCLCPP_INFO(get_logger(), " parsing time: %f ", ((grab_time * 1000) - (timestamp)));
  }
}

void ADMADriver::updateLoop()
{
  std::array<char, 856> recv_buf;

  while (rclcpp::ok()) {
    socket_->receiveUDPPacket(recv_buf);
    parseData(recv_buf);
  }
}
}  // namespace genesys

RCLCPP_COMPONENTS_REGISTER_NODE(genesys::ADMADriver)
