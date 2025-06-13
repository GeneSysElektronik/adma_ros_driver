// BSD 3-Clause License
// Copyright (c) 2023, GeneSys Elektronik
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// 1. Redistributions of source code must retain the above copyright notice, this
//    list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from
//    this software without specific prior written permission.
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include <arpa/inet.h>
#include <sys/socket.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <iostream>
#include <string>

#include <rclcpp_components/register_node_macro.hpp>
#include <adma_core_lib/parser/parser_utils.hpp>

#include "adma_tools_cpp/gsda_server.hpp"

/**
 * @brief This helper class can replay post-processed ADMA data (GSDA file) and publish
 * the data to ROS. Only supported for protocol version >= 3.3.4
 */
namespace genesys
{
namespace tools
{
GSDAServer::GSDAServer(const rclcpp::NodeOptions & options)
: Node("gsda_server", options),
  msgCounter_(0),
  tfBroadcaster_(*this)
{
  // read ros parameters
  frequency_ = this->declare_parameter("frequency", 100);
  gsdaFilePath_ = declare_parameter("gsda_file", "/home/$USER/$ROS2_WS/data/$FILENAME.gsda");
  gnss_frame_ = this->declare_parameter("frame_ids.navsatfix", "gnss_link");
  imu_frame_ = this->declare_parameter("frame_ids.imu", "imu_link");
  adma_frame_ = this->declare_parameter("frame_ids.adma", "adma");
  adma_status_frame_ = this->declare_parameter("frame_ids.adma_status", "adma_status");
  raw_data_frame_ = this->declare_parameter("frame_ids.raw_data", "data_raw");
  odometry_pose_frame_ = this->declare_parameter("frame_ids.odometry_pose_id", "adma");
  odometry_child_frame_ = this->declare_parameter("frame_ids.odometry_twist_id", "odometry");
  odometry_yaw_offset_ = this->declare_parameter("odometry_yaw_offset", 0.0);
  publish_TF_ = this->declare_parameter("publish_tf", false);
  navsatfix_id_ = this->declare_parameter("topic_pois.navsatfix", 1);
  imu_id_ = this->declare_parameter("topic_pois.imu", 1);
  velocity_id_ = this->declare_parameter("topic_pois.velocity", 1);
  odometry_id_ = this->declare_parameter("topic_pois.odometry", 1);
  gsdaFile_ = std::fstream(gsdaFilePath_);
  if (gsdaFile_) {
    RCLCPP_INFO(get_logger(), "Loaded GSDA-File: %s", gsdaFilePath_.c_str());
  } else {
    RCLCPP_WARN(get_logger(), "Desired GSDA-File not found: %s", gsdaFilePath_.c_str());
  }

  pub_adma_data_scaled_ =
    this->create_publisher<adma_ros_driver_msgs::msg::AdmaDataScaled>("adma/data_scaled", 1);
  pub_adma_status_ =
    this->create_publisher<adma_ros_driver_msgs::msg::AdmaStatus>("adma/status", 1);
  pub_navsat_fix_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("adma/fix", 1);
  pub_imu_ = this->create_publisher<sensor_msgs::msg::Imu>("adma/imu", 1);
  pub_heading_ = this->create_publisher<std_msgs::msg::Float64>("adma/heading", 1);
  pub_velocity_ = this->create_publisher<std_msgs::msg::Float64>("adma/velocity", 1);
  pub_odometry_ = this->create_publisher<nav_msgs::msg::Odometry>("adma/odometry", 1);
  if (publish_TF_) {
    pub_trajectory_ = this->create_publisher<nav_msgs::msg::Path>("adma/trajectory", 1);
    trajectory_msg_.header.frame_id = odometry_child_frame_;
  }

  // TODO(rschilli): make version injection more dynamic
  parser_ = new ADMA2ROSParser(3360);

  updateLoop();
}

GSDAServer::~GSDAServer()
{
  RCLCPP_INFO(get_logger(), "GSDA file streaming done.. Read %ld messages from file", msgCounter_);
}

void GSDAServer::updateLoop()
{
  // define messages to publish
  // TODO(rschilli): inject frame_id as ROS params like its done in driver node
  adma_ros_driver_msgs::msg::AdmaDataScaled dataScaledMsg;
  dataScaledMsg.header.frame_id = adma_frame_;
  adma_ros_driver_msgs::msg::AdmaStatus stateMsg;
  stateMsg.header.frame_id = adma_status_frame_;
  std_msgs::msg::Float64 velMsg;
  std_msgs::msg::Float64 headingMsg;
  sensor_msgs::msg::Imu imuMsg;
  imuMsg.header.frame_id = imu_frame_;
  sensor_msgs::msg::NavSatFix navsatfixMsg;
  navsatfixMsg.header.frame_id = imu_frame_;
  nav_msgs::msg::Odometry odomMsg;
  odomMsg.header.frame_id = odometry_pose_frame_;
  odomMsg.child_frame_id = odometry_child_frame_;

  // offset between UNIX and GNSS (in ms)
  uint64_t offset_gps_unix = 315964800000;
  uint64_t week_to_msec = 604800000;
  uint64_t timestamp;
  builtin_interfaces::msg::Time timestampForMsgs;

  while (rclcpp::ok()) {
    // iterate through gsda file
    while (getline(gsdaFile_, line)) {
      if (msgCounter_ == 0) {
        readLine();
        extractHeader();
        msgCounter_++;
        continue;
      } else if (msgCounter_ == 1) {
        // skip first 2 lines cause they are not used here
        msgCounter_++;
        continue;
      }
      readLine();

      fillDataScaledMsg(dataScaledMsg);
      timestamp = dataScaledMsg.ins_time_msec + offset_gps_unix;
      timestamp += dataScaledMsg.ins_time_week * week_to_msec;
      dataScaledMsg.time_msec = timestamp;
      dataScaledMsg.time_nsec = (timestamp % 1000) * 1E6;

      timestampForMsgs.sec = timestamp / 1000;
      timestampForMsgs.nanosec = (timestamp % 1000) * 1E6;
      dataScaledMsg.header.stamp = timestampForMsgs;
      pois = {
        dataScaledMsg.poi_1,
        dataScaledMsg.poi_2,
        dataScaledMsg.poi_3,
        dataScaledMsg.poi_4,
        dataScaledMsg.poi_5,
        dataScaledMsg.poi_6,
        dataScaledMsg.poi_7,
        dataScaledMsg.poi_8
      };
      // extract separate msgs
      parser_->extractNavSatFix(dataScaledMsg, navsatfixMsg, pois, navsatfix_id_);
      navsatfixMsg.header.stamp = timestampForMsgs;
      parser_->extractIMU(dataScaledMsg, imuMsg, pois, imu_id_);
      // ADMA PP doesnt provide "hr" channels so use normal body rate/acc for IMU
      imuMsg.linear_acceleration.x = dataScaledMsg.acc_body.x * 9.81;
      imuMsg.linear_acceleration.y = dataScaledMsg.acc_body.y * 9.81;
      imuMsg.linear_acceleration.z = dataScaledMsg.acc_body.z * 9.81;
      imuMsg.angular_velocity.x = deg2Rad(dataScaledMsg.rate_body.x);
      imuMsg.angular_velocity.y = deg2Rad(dataScaledMsg.rate_body.y);
      imuMsg.angular_velocity.z = deg2Rad(dataScaledMsg.rate_body.z);
      imuMsg.header.stamp = timestampForMsgs;
      // read heading and velocity
      headingMsg.data = dataScaledMsg.ins_yaw;
      geometry_msgs::msg::Vector3 insSource = velocity_id_ == 0 ?
        dataScaledMsg.ins_vel_frame :
        pois[velocity_id_ - 1].ins_vel_hor;
      velMsg.data = std::sqrt(std::pow(insSource.x, 2) + std::pow(insSource.y, 2)) * 3.6;

      extractBytes(stateMsg, dataScaledMsg);
      stateMsg.header.stamp = timestampForMsgs;

      odomMsg.header.stamp = timestampForMsgs;
      parser_->extractOdometry(dataScaledMsg, odomMsg, odometry_yaw_offset_, pois, odometry_id_);

      // TODO(rschilli): extract the TF stuff to separate node for reusage
      if (publish_TF_) {
        // TODO(rschilli): evaluate those transformations!!
        geometry_msgs::msg::TransformStamped transform_msg;
        transform_msg.header.stamp = timestampForMsgs;
        transform_msg.header.frame_id = "map";
        transform_msg.child_frame_id = adma_frame_;
        transform_msg.transform.rotation = odomMsg.pose.pose.orientation;
        transform_msg.transform.translation.x = odomMsg.pose.pose.position.x;
        transform_msg.transform.translation.y = odomMsg.pose.pose.position.y;
        transform_msg.transform.translation.z = odomMsg.pose.pose.position.z;
        tfBroadcaster_.sendTransform(transform_msg);

        transform_msg.header.frame_id = adma_frame_;
        transform_msg.child_frame_id = odometry_child_frame_;
        tf2::Quaternion rotOdom2Base;
        rotOdom2Base.setRPY(0.0, 0.0, 0.0);
        transform_msg.transform.rotation = tf2::toMsg(rotOdom2Base);
        transform_msg.transform.translation.x = 0.0;
        transform_msg.transform.translation.y = 0.0;
        transform_msg.transform.translation.z = 0.0;
        tfBroadcaster_.sendTransform(transform_msg);

        geometry_msgs::msg::PoseStamped pose_msg;
        pose_msg.pose = odomMsg.pose.pose;
        pose_msg.header = odomMsg.header;
        trajectory_msg_.poses.push_back(pose_msg);
        pub_trajectory_->publish(trajectory_msg_);
      }

      pub_adma_data_scaled_->publish(dataScaledMsg);
      pub_adma_status_->publish(stateMsg);
      pub_navsat_fix_->publish(navsatfixMsg);
      pub_imu_->publish(imuMsg);
      pub_heading_->publish(headingMsg);
      pub_velocity_->publish(velMsg);
      pub_odometry_->publish(odomMsg);

      std::this_thread::sleep_for(std::chrono::milliseconds(1000 / frequency_));
      msgCounter_++;
    }
    rclcpp::shutdown();
  }
}

void GSDAServer::readLine()
{
  row.clear();
  std::stringstream str(line);
  while (getline(str, word, ',')) {
    row.push_back(word);
  }
}

void GSDAServer::extractHeader()
{
  for (size_t i = 0; i < row.size(); i++) {
    std::string channelName = row[i];
    if (channelName.rfind("% ", 0) == 0) {
      channelName.erase(0, 2);
    }
    indexMap_.insert({channelName, i});
  }
}

double GSDAServer::readValue(std::string dataName)
{
  auto it = indexMap_.find(dataName);
  if (it != indexMap_.end()) {
    size_t index = (*it).second;
    if (index <= row.size()) {
      return std::stod(row[index]);
    }
  }
  return 0.0;
}

int GSDAServer::readByteValue(std::string dataName)
{
  auto it = indexMap_.find(dataName);
  if (it != indexMap_.end()) {
    size_t index = (*it).second;
    if (index <= row.size()) {
      return std::stoi(row[index].c_str());
    }
  }
  if (std::find(
      unsupportedFields_.begin(), unsupportedFields_.end(),
      dataName) == unsupportedFields_.end())
  {
    unsupportedFields_.push_back(dataName);
    RCLCPP_WARN(get_logger(), "Channelname %s not found in GSDA File..", dataName.c_str());
  }
  return 0;
}

void GSDAServer::extractBytes(
  adma_ros_driver_msgs::msg::AdmaStatus & stateMsg,
  adma_ros_driver_msgs::msg::AdmaDataScaled & dataScaledMsg)
{
  // read values
  unsigned char gnssStatus = (unsigned char) readByteValue("State0");
  unsigned char signalInStatus = (unsigned char) readByteValue("State1");
  unsigned char miscStatus = (unsigned char) readByteValue("State2");
  unsigned char kfStatus = (unsigned char) readByteValue("State3");
  unsigned char statusRobot = (unsigned char) readByteValue("State4");
  unsigned char error1 = (unsigned char) readByteValue("Error0");
  unsigned char error2 = (unsigned char) readByteValue("Error1");
  unsigned char warn1 = (unsigned char) readByteValue("Error2");
  unsigned char error3 = (unsigned char) readByteValue("Error3");

  // fill status bytes
  stateMsg.status_bytes.status_byte_0 = gnssStatus;
  stateMsg.status_bytes.status_byte_1 = signalInStatus;
  stateMsg.status_bytes.status_byte_2 = miscStatus;
  // stateMsg.status_bytes.status_count = ?;
  stateMsg.status_bytes.status_byte_4 = kfStatus;
  stateMsg.status_bytes.status_byte_5 = statusRobot;
  // fill error/warning bytes
  stateMsg.error_warnings_bytes.error_1 = error1;
  stateMsg.error_warnings_bytes.error_2 = error2;
  stateMsg.error_warnings_bytes.warn_1 = warn1;
  stateMsg.error_warnings_bytes.error_3 = error3;
  dataScaledMsg.error_warning.error_1 = error1;
  dataScaledMsg.error_warning.error_2 = error2;
  dataScaledMsg.error_warning.warn_1 = warn1;
  dataScaledMsg.error_warning.error_3 = error3;

  // parse SEW bits of bytes
  // status_byte_0
  /* status gnss mode */
  std::bitset<8> gnss_status_byte = gnssStatus;
  std::bitset<4> status_gnss_mode;
  status_gnss_mode[0] = gnss_status_byte[0];
  status_gnss_mode[1] = gnss_status_byte[1];
  status_gnss_mode[2] = gnss_status_byte[2];
  status_gnss_mode[3] = gnss_status_byte[3];
  stateMsg.status.status_gnss_mode = status_gnss_mode.to_ulong();
  bool standstill_c = getbit(gnssStatus, 4);
  bool status_skidding = getbit(gnssStatus, 5);
  bool status_external_vel = getbit(gnssStatus, 7);
  /* status stand still */
  stateMsg.status.status_standstill = standstill_c;
  /* status skidding */
  stateMsg.status.status_skidding = status_skidding;
  /* status external velocity slip */
  stateMsg.status.status_external_vel_out = status_external_vel;

  // status_byte_1
  bool status_trig_gnss = getbit(signalInStatus, 0);
  bool status_signal_in3 = getbit(signalInStatus, 1);
  bool status_signal_in2 = getbit(signalInStatus, 2);
  bool status_signal_in1 = getbit(signalInStatus, 3);
  bool status_alignment = getbit(signalInStatus, 4);
  bool status_ahrs_ins = getbit(signalInStatus, 5);
  bool status_dead_reckoning = getbit(signalInStatus, 6);
  bool status_synclock = getbit(signalInStatus, 7);
  /* status statustriggnss */
  stateMsg.status.status_trig_gnss = status_trig_gnss;
  /* status statussignalin3 */
  stateMsg.status.status_signal_in3 = status_signal_in3;
  /* status statussignalin2 */
  stateMsg.status.status_signal_in2 = status_signal_in2;
  /* status statussignalin1 */
  stateMsg.status.status_signal_in1 = status_signal_in1;
  /* status statusalignment */
  stateMsg.status.status_alignment = status_alignment;
  /* status statusahrsins */
  stateMsg.status.status_ahrs_ins = status_ahrs_ins;
  /* status statusdeadreckoning */
  stateMsg.status.status_dead_reckoning = status_dead_reckoning;
  /* status statussynclock */
  stateMsg.status.status_synclock = status_synclock;

  // status_byte_2
  bool status_evk_activ = getbit(miscStatus, 0);
  bool status_evk_estimates = getbit(miscStatus, 1);
  bool status_heading_executed = getbit(miscStatus, 2);
  bool status_configuration_changed = getbit(miscStatus, 3);
  /* status statustriggnss */
  stateMsg.status.status_evk_activ = status_evk_activ;
  /* status status_evk_estimates */
  stateMsg.status.status_evk_estimates = status_evk_estimates;
  /* status status_heading_executed */
  stateMsg.status.status_heading_executed = status_heading_executed;
  /* status status_configuration_changed */
  stateMsg.status.status_config_changed = status_configuration_changed;
  /* status tilt */
  std::bitset<8> evk_status_byte = miscStatus;
  std::bitset<2> status_tilt;
  status_tilt[0] = evk_status_byte[4];
  status_tilt[1] = evk_status_byte[5];
  stateMsg.status.status_tilt = status_tilt.to_ulong();
  /* status pos */
  std::bitset<2> status_pos;
  status_pos[0] = evk_status_byte[6];
  status_pos[1] = evk_status_byte[7];
  stateMsg.status.status_pos = status_pos.to_ulong();

  // status_byte_4
  bool status_kalmanfilter_settled = getbit(kfStatus, 0);
  bool status_kf_lat_stimulated = getbit(kfStatus, 1);
  bool status_kf_long_stimulated = getbit(kfStatus, 2);
  bool status_kf_steady_state = getbit(kfStatus, 3);
  stateMsg.status.status_kalmanfilter_settled = status_kalmanfilter_settled;
  stateMsg.status.status_kf_lat_stimulated = status_kf_lat_stimulated;
  stateMsg.status.status_kf_long_stimulated = status_kf_long_stimulated;
  stateMsg.status.status_kf_steady_state = status_kf_steady_state;
  std::bitset<8> kf_status_byte = kfStatus;
  std::bitset<2> status_speed;
  status_speed[0] = kf_status_byte[4];
  status_speed[1] = kf_status_byte[5];
  stateMsg.status.status_speed = status_speed.to_ulong();

  // status_byte_5
  std::bitset<8> bit_status_robot = statusRobot;
  std::bitset<4> status_robot;
  for (size_t i = 0; i < 4; i++) {
    status_robot[i] = bit_status_robot[i];
  }
  stateMsg.status.status_robot = status_robot.to_ulong();

  // error_byte_0
  stateMsg.error_warnings.error_gyro_hw = getbit(error1, 0);
  stateMsg.error_warnings.error_accel_hw = getbit(error1, 1);
  stateMsg.error_warnings.error_ext_speed_hw = getbit(error1, 2);
  stateMsg.error_warnings.error_gnss_hw = getbit(error1, 3);
  stateMsg.error_warnings.error_data_bus_checksum = getbit(error1, 4);
  stateMsg.error_warnings.error_eeprom = getbit(error1, 5);
  stateMsg.error_warnings.error_cmd = getbit(error1, 7);

  // error_byte_1
  stateMsg.error_warnings.error_data_bus = getbit(error2, 0);
  stateMsg.error_warnings.error_can_bus = getbit(error2, 1);
  stateMsg.error_warnings.error_num = getbit(error2, 3);
  stateMsg.error_warnings.error_temp_warning = getbit(error2, 4);
  stateMsg.error_warnings.error_reduced_accuracy = getbit(error2, 5);
  stateMsg.error_warnings.error_range_max = getbit(error2, 6);

  // warn_byte_0
  stateMsg.error_warnings.warn_gnss_no_solution = getbit(warn1, 0);
  stateMsg.error_warnings.warn_gnss_vel_ignored = getbit(warn1, 1);
  stateMsg.error_warnings.warn_gnss_pos_ignored = getbit(warn1, 2);
  stateMsg.error_warnings.warn_gnss_unable_to_cfg = getbit(warn1, 3);
  stateMsg.error_warnings.warn_speed_off = getbit(warn1, 4);
  stateMsg.error_warnings.warn_gnss_dualant_ignored = getbit(warn1, 5);

  // error_byte_2
  stateMsg.error_warnings.error_hw_sticky = getbit(error3, 0);
}

void GSDAServer::fillDataScaledMsg(adma_ros_driver_msgs::msg::AdmaDataScaled & dataScaledMsg)
{
  // accelerations body in g
  dataScaledMsg.acc_body.x = readValue("Acc_Body_X");
  dataScaledMsg.acc_body.y = readValue("Acc_Body_Y");
  dataScaledMsg.acc_body.z = readValue("Acc_Body_Z");

  // acceleration horizontal in g
  dataScaledMsg.acc_hor.x = readValue("Acc_Hor_X");
  dataScaledMsg.acc_hor.y = readValue("Acc_Hor_Y");
  dataScaledMsg.acc_hor.z = readValue("Acc_Hor_Z");

  // acceleration frame in g
  dataScaledMsg.acc_body_hr.x = readValue("Acc_Frame_X");
  dataScaledMsg.acc_body_hr.y = readValue("Acc_Frame_Y");
  dataScaledMsg.acc_body_hr.z = readValue("Acc_Frame_Z");

  // rates body in deg/s
  dataScaledMsg.rate_body.x = readValue("Rate_Body_X");
  dataScaledMsg.rate_body.y = readValue("Rate_Body_Y");
  dataScaledMsg.rate_body.z = readValue("Rate_Body_Z");

  // rates hor in deg/s
  dataScaledMsg.rate_hor.x = readValue("Rate_Hor_X");
  dataScaledMsg.rate_hor.y = readValue("Rate_Hor_Y");
  dataScaledMsg.rate_hor.z = readValue("Rate_Hor_Z");

  // rates frame in deg/s
  dataScaledMsg.rate_body_hr.x = readValue("Rate_Frame_X");
  dataScaledMsg.rate_body_hr.y = readValue("Rate_Frame_Y");
  dataScaledMsg.rate_body_hr.z = readValue("Rate_Frame_Z");

  // POI's
  // POI1
  // acceleration body in g
  dataScaledMsg.poi_1.acc_body.x = readValue("Acc_Body_X_POI1");
  dataScaledMsg.poi_1.acc_body.y = readValue("Acc_Body_Y_POI1");
  dataScaledMsg.poi_1.acc_body.z = readValue("Acc_Body_Z_POI1");

  // acceleration horizontal in g
  dataScaledMsg.poi_1.acc_hor.x = readValue("Acc_Hor_X_POI1");
  dataScaledMsg.poi_1.acc_hor.y = readValue("Acc_Hor_Y_POI1");
  dataScaledMsg.poi_1.acc_hor.z = readValue("Acc_Hor_Z_POI1");

  // Auxiliary
  dataScaledMsg.poi_1.inv_path_radius = readValue("Inv_Path_Radius_POI1");
  dataScaledMsg.poi_1.side_slip_angle = readValue("Side_Slip_Angle_POI1");
  dataScaledMsg.poi_1.dist_trav = readValue("Dist_Trav_POI1");

  // ins Position
  dataScaledMsg.poi_1.ins_lat_abs = readValue("INS_Lat_Abs_POI1");
  dataScaledMsg.poi_1.ins_long_abs = readValue("INS_Long_Abs_POI1");
  dataScaledMsg.poi_1.ins_height = readValue("INS_Height_POI1");

  // relative position
  dataScaledMsg.poi_1.ins_pos_rel_x = readValue("INS_Pos_Rel_X_POI1");
  dataScaledMsg.poi_1.ins_pos_rel_y = readValue("INS_Pos_Rel_Y_POI1");

  // ins velocities
  dataScaledMsg.poi_1.ins_vel_hor.x = readValue("INS_Vel_Hor_X_POI1");
  dataScaledMsg.poi_1.ins_vel_hor.y = readValue("INS_Vel_Hor_Y_POI1");
  dataScaledMsg.poi_1.ins_vel_hor.z = readValue("INS_Vel_Hor_Z_POI1");

  // POI2
  // acceleration body in g
  dataScaledMsg.poi_2.acc_body.x = readValue("Acc_Body_X_POI2");
  dataScaledMsg.poi_2.acc_body.y = readValue("Acc_Body_Y_POI2");
  dataScaledMsg.poi_2.acc_body.z = readValue("Acc_Body_Z_POI2");

  // acceleration horizontal in g
  dataScaledMsg.poi_2.acc_hor.x = readValue("Acc_Hor_X_POI2");
  dataScaledMsg.poi_2.acc_hor.y = readValue("Acc_Hor_Y_POI2");
  dataScaledMsg.poi_2.acc_hor.z = readValue("Acc_Hor_Z_POI2");

  // Auxiliary
  dataScaledMsg.poi_2.inv_path_radius = readValue("Inv_Path_Radius_POI2");
  dataScaledMsg.poi_2.side_slip_angle = readValue("Side_Slip_Angle_POI2");
  dataScaledMsg.poi_2.dist_trav = readValue("Dist_Trav_POI2");

  // ins Position
  dataScaledMsg.poi_2.ins_lat_abs = readValue("INS_Lat_Abs_POI2");
  dataScaledMsg.poi_2.ins_long_abs = readValue("INS_Long_Abs_POI2");
  dataScaledMsg.poi_2.ins_height = readValue("INS_Height_POI2");

  // relative position
  dataScaledMsg.poi_2.ins_pos_rel_x = readValue("INS_Pos_Rel_X_POI2");
  dataScaledMsg.poi_2.ins_pos_rel_y = readValue("INS_Pos_Rel_Y_POI2");

  // ins velocities
  dataScaledMsg.poi_2.ins_vel_hor.x = readValue("INS_Vel_Hor_X_POI2");
  dataScaledMsg.poi_2.ins_vel_hor.y = readValue("INS_Vel_Hor_Y_POI2");
  dataScaledMsg.poi_2.ins_vel_hor.z = readValue("INS_Vel_Hor_Z_POI2");

  // POI3
  // acceleration body in g
  dataScaledMsg.poi_3.acc_body.x = readValue("Acc_Body_X_POI3");
  dataScaledMsg.poi_3.acc_body.y = readValue("Acc_Body_Y_POI3");
  dataScaledMsg.poi_3.acc_body.z = readValue("Acc_Body_Z_POI3");

  // acceleration horizontal in g
  dataScaledMsg.poi_3.acc_hor.x = readValue("Acc_Hor_X_POI3");
  dataScaledMsg.poi_3.acc_hor.y = readValue("Acc_Hor_Y_POI3");
  dataScaledMsg.poi_3.acc_hor.z = readValue("Acc_Hor_Z_POI3");

  // Auxiliary
  dataScaledMsg.poi_3.inv_path_radius = readValue("Inv_Path_Radius_POI3");
  dataScaledMsg.poi_3.side_slip_angle = readValue("Side_Slip_Angle_POI3");
  dataScaledMsg.poi_3.dist_trav = readValue("Dist_Trav_POI3");

  // ins Position
  dataScaledMsg.poi_3.ins_lat_abs = readValue("INS_Lat_Abs_POI3");
  dataScaledMsg.poi_3.ins_long_abs = readValue("INS_Long_Abs_POI3");
  dataScaledMsg.poi_3.ins_height = readValue("INS_Height_POI3");

  // relative position
  dataScaledMsg.poi_3.ins_pos_rel_x = readValue("INS_Pos_Rel_X_POI3");
  dataScaledMsg.poi_3.ins_pos_rel_y = readValue("INS_Pos_Rel_Y_POI3");

  // ins velocities
  dataScaledMsg.poi_3.ins_vel_hor.x = readValue("INS_Vel_Hor_X_POI3");
  dataScaledMsg.poi_3.ins_vel_hor.y = readValue("INS_Vel_Hor_Y_POI3");
  dataScaledMsg.poi_3.ins_vel_hor.z = readValue("INS_Vel_Hor_Z_POI3");

  // POI4
  // acceleration body in g
  dataScaledMsg.poi_4.acc_body.x = readValue("Acc_Body_X_POI4");
  dataScaledMsg.poi_4.acc_body.y = readValue("Acc_Body_Y_POI4");
  dataScaledMsg.poi_4.acc_body.z = readValue("Acc_Body_Z_POI4");

  // acceleration horizontal in g
  dataScaledMsg.poi_4.acc_hor.x = readValue("Acc_Hor_X_POI4");
  dataScaledMsg.poi_4.acc_hor.y = readValue("Acc_Hor_Y_POI4");
  dataScaledMsg.poi_4.acc_hor.z = readValue("Acc_Hor_Z_POI4");

  // Auxiliary
  dataScaledMsg.poi_4.inv_path_radius = readValue("Inv_Path_Radius_POI4");
  dataScaledMsg.poi_4.side_slip_angle = readValue("Side_Slip_Angle_POI4");
  dataScaledMsg.poi_4.dist_trav = readValue("Dist_Trav_POI4");

  // ins Position
  dataScaledMsg.poi_4.ins_lat_abs = readValue("INS_Lat_Abs_POI4");
  dataScaledMsg.poi_4.ins_long_abs = readValue("INS_Long_Abs_POI4");
  dataScaledMsg.poi_4.ins_height = readValue("INS_Height_POI4");

  // relative position
  dataScaledMsg.poi_4.ins_pos_rel_x = readValue("INS_Pos_Rel_X_POI4");
  dataScaledMsg.poi_4.ins_pos_rel_y = readValue("INS_Pos_Rel_Y_POI4");

  // ins velocities
  dataScaledMsg.poi_4.ins_vel_hor.x = readValue("INS_Vel_Hor_X_POI4");
  dataScaledMsg.poi_4.ins_vel_hor.y = readValue("INS_Vel_Hor_Y_POI4");
  dataScaledMsg.poi_4.ins_vel_hor.z = readValue("INS_Vel_Hor_Z_POI4");

  // POI5
  // acceleration body in g
  dataScaledMsg.poi_5.acc_body.x = readValue("Acc_Body_X_POI5");
  dataScaledMsg.poi_5.acc_body.y = readValue("Acc_Body_Y_POI5");
  dataScaledMsg.poi_5.acc_body.z = readValue("Acc_Body_Z_POI5");

  // acceleration horizontal in g
  dataScaledMsg.poi_5.acc_hor.x = readValue("Acc_Hor_X_POI5");
  dataScaledMsg.poi_5.acc_hor.y = readValue("Acc_Hor_Y_POI5");
  dataScaledMsg.poi_5.acc_hor.z = readValue("Acc_Hor_Z_POI5");

  // Auxiliary
  dataScaledMsg.poi_5.inv_path_radius = readValue("Inv_Path_Radius_POI5");
  dataScaledMsg.poi_5.side_slip_angle = readValue("Side_Slip_Angle_POI5");
  dataScaledMsg.poi_5.dist_trav = readValue("Dist_Trav_POI5");

  // ins Position
  dataScaledMsg.poi_5.ins_lat_abs = readValue("INS_Lat_Abs_POI5");
  dataScaledMsg.poi_5.ins_long_abs = readValue("INS_Long_Abs_POI5");
  dataScaledMsg.poi_5.ins_height = readValue("INS_Height_POI5");

  // relative position
  dataScaledMsg.poi_5.ins_pos_rel_x = readValue("INS_Pos_Rel_X_POI5");
  dataScaledMsg.poi_5.ins_pos_rel_y = readValue("INS_Pos_Rel_Y_POI5");

  // ins velocities
  dataScaledMsg.poi_5.ins_vel_hor.x = readValue("INS_Vel_Hor_X_POI5");
  dataScaledMsg.poi_5.ins_vel_hor.y = readValue("INS_Vel_Hor_Y_POI5");
  dataScaledMsg.poi_5.ins_vel_hor.z = readValue("INS_Vel_Hor_Z_POI5");

  // POI6
  // acceleration body in g
  dataScaledMsg.poi_6.acc_body.x = readValue("Acc_Body_X_POI6");
  dataScaledMsg.poi_6.acc_body.y = readValue("Acc_Body_Y_POI6");
  dataScaledMsg.poi_6.acc_body.z = readValue("Acc_Body_Z_POI6");

  // acceleration horizontal in g
  dataScaledMsg.poi_6.acc_hor.x = readValue("Acc_Hor_X_POI6");
  dataScaledMsg.poi_6.acc_hor.y = readValue("Acc_Hor_Y_POI6");
  dataScaledMsg.poi_6.acc_hor.z = readValue("Acc_Hor_Z_POI6");

  // Auxiliary
  dataScaledMsg.poi_6.inv_path_radius = readValue("Inv_Path_Radius_POI6");
  dataScaledMsg.poi_6.side_slip_angle = readValue("Side_Slip_Angle_POI6");
  dataScaledMsg.poi_6.dist_trav = readValue("Dist_Trav_POI6");

  // ins Position
  dataScaledMsg.poi_6.ins_lat_abs = readValue("INS_Lat_Abs_POI6");
  dataScaledMsg.poi_6.ins_long_abs = readValue("INS_Long_Abs_POI6");
  dataScaledMsg.poi_6.ins_height = readValue("INS_Height_POI6");

  // relative position
  dataScaledMsg.poi_6.ins_pos_rel_x = readValue("INS_Pos_Rel_X_POI6");
  dataScaledMsg.poi_6.ins_pos_rel_y = readValue("INS_Pos_Rel_Y_POI6");

  // ins velocities
  dataScaledMsg.poi_6.ins_vel_hor.x = readValue("INS_Vel_Hor_X_POI6");
  dataScaledMsg.poi_6.ins_vel_hor.y = readValue("INS_Vel_Hor_Y_POI6");
  dataScaledMsg.poi_6.ins_vel_hor.z = readValue("INS_Vel_Hor_Z_POI6");

  // POI7
  // acceleration body in g
  dataScaledMsg.poi_7.acc_body.x = readValue("Acc_Body_X_POI7");
  dataScaledMsg.poi_7.acc_body.y = readValue("Acc_Body_Y_POI7");
  dataScaledMsg.poi_7.acc_body.z = readValue("Acc_Body_Z_POI7");

  // acceleration horizontal in g
  dataScaledMsg.poi_7.acc_hor.x = readValue("Acc_Hor_X_POI7");
  dataScaledMsg.poi_7.acc_hor.y = readValue("Acc_Hor_Y_POI7");
  dataScaledMsg.poi_7.acc_hor.z = readValue("Acc_Hor_Z_POI7");

  // Auxiliary
  dataScaledMsg.poi_7.inv_path_radius = readValue("Inv_Path_Radius_POI7");
  dataScaledMsg.poi_7.side_slip_angle = readValue("Side_Slip_Angle_POI7");
  dataScaledMsg.poi_7.dist_trav = readValue("Dist_Trav_POI7");

  // ins Position
  dataScaledMsg.poi_7.ins_lat_abs = readValue("INS_Lat_Abs_POI7");
  dataScaledMsg.poi_7.ins_long_abs = readValue("INS_Long_Abs_POI7");
  dataScaledMsg.poi_7.ins_height = readValue("INS_Height_POI7");

  // relative position
  dataScaledMsg.poi_7.ins_pos_rel_x = readValue("INS_Pos_Rel_X_POI7");
  dataScaledMsg.poi_7.ins_pos_rel_y = readValue("INS_Pos_Rel_Y_POI7");

  // ins velocities
  dataScaledMsg.poi_7.ins_vel_hor.x = readValue("INS_Vel_Hor_X_POI7");
  dataScaledMsg.poi_7.ins_vel_hor.y = readValue("INS_Vel_Hor_Y_POI7");
  dataScaledMsg.poi_7.ins_vel_hor.z = readValue("INS_Vel_Hor_Z_POI7");

  // POI8
  // acceleration body in g
  dataScaledMsg.poi_8.acc_body.x = readValue("Acc_Body_X_POI8");
  dataScaledMsg.poi_8.acc_body.y = readValue("Acc_Body_Y_POI8");
  dataScaledMsg.poi_8.acc_body.z = readValue("Acc_Body_Z_POI8");

  // acceleration horizontal in g
  dataScaledMsg.poi_8.acc_hor.x = readValue("Acc_Hor_X_POI8");
  dataScaledMsg.poi_8.acc_hor.y = readValue("Acc_Hor_Y_POI8");
  dataScaledMsg.poi_8.acc_hor.z = readValue("Acc_Hor_Z_POI8");

  // Auxiliary
  dataScaledMsg.poi_8.inv_path_radius = readValue("Inv_Path_Radius_POI8");
  dataScaledMsg.poi_8.side_slip_angle = readValue("Side_Slip_Angle_POI8");
  dataScaledMsg.poi_8.dist_trav = readValue("Dist_Trav_POI8");

  // ins Position
  dataScaledMsg.poi_8.ins_lat_abs = readValue("INS_Lat_Abs_POI8");
  dataScaledMsg.poi_8.ins_long_abs = readValue("INS_Long_Abs_POI8");
  dataScaledMsg.poi_8.ins_height = readValue("INS_Height_POI8");

  // relative position
  dataScaledMsg.poi_8.ins_pos_rel_x = readValue("INS_Pos_Rel_X_POI8");
  dataScaledMsg.poi_8.ins_pos_rel_y = readValue("INS_Pos_Rel_Y_POI8");

  // ins velocities
  dataScaledMsg.poi_8.ins_vel_hor.x = readValue("INS_Vel_Hor_X_POI8");
  dataScaledMsg.poi_8.ins_vel_hor.y = readValue("INS_Vel_Hor_Y_POI8");
  dataScaledMsg.poi_8.ins_vel_hor.z = readValue("INS_Vel_Hor_Z_POI8");


  // external velocity
  dataScaledMsg.ext_vel_x_corrected = readValue("Ext_Vel_X_corrected");

  // system data
  dataScaledMsg.system_ta = readValue("System_TA");
  dataScaledMsg.system_temp = readValue("System_Temp");
  dataScaledMsg.system_dsp_load = readValue("System_DSP_Load");
  dataScaledMsg.system_time_since_init = readValue("System_TimeSinceInit");

  // auxiliary
  dataScaledMsg.inv_path_radius = readValue("Inv_Path_Radius");
  dataScaledMsg.side_slip_angle = readValue("Side_Slip_Angle");
  dataScaledMsg.dist_trav = readValue("Dist_Trav");

  // gnss positions
  dataScaledMsg.gnss_lat_abs = readValue("GNSS_Lat_Abs");
  dataScaledMsg.gnss_long_abs = readValue("GNSS_Long_Abs");
  dataScaledMsg.gnss_height = readValue("GNSS_Height");
  dataScaledMsg.gnss_pos_rel_x = readValue("GNSS_Pos_Rel_X");
  dataScaledMsg.gnss_pos_rel_y = readValue("GNSS_Pos_Rel_Y");
  dataScaledMsg.gnss_stddev_lat = readValue("GNSS_Stddev_Lat");
  dataScaledMsg.gnss_stddev_long = readValue("GNSS_Stddev_Long");
  dataScaledMsg.gnss_stddev_height = readValue("GNSS_Stddev_Height");

  // gnss velocities
  dataScaledMsg.gnss_vel_frame.x = readValue("GNSS_Vel_Frame_X");
  dataScaledMsg.gnss_vel_frame.y = readValue("GNSS_Vel_Frame_Y");
  dataScaledMsg.gnss_vel_frame.z = readValue("GNSS_Vel_Frame_Z");
  dataScaledMsg.gnss_vel_latency = readValue("GNSS_Vel_Latency");
  dataScaledMsg.gnss_stddev_vel.x = readValue("GNSS_Stddev_Vel_X");
  dataScaledMsg.gnss_stddev_vel.y = readValue("GNSS_Stddev_Vel_Y");
  dataScaledMsg.gnss_stddev_vel.z = readValue("GNSS_Stddev_Vel_Z");

  // gnss aux data
  dataScaledMsg.gnss_log_delay = readValue("GNSS_Log_Delay");
  dataScaledMsg.gnss_diffage = readValue("GNSS_DiffAge");
  dataScaledMsg.gnss_sats_visible = readValue("GNSS_Sats_Visible");
  dataScaledMsg.gnss_time_msec = readValue("GNSS_Time_msec");
  dataScaledMsg.gnss_time_week = readValue("GNSS_Time_Week");

  // dual ant data
  dataScaledMsg.gnss_dualant_heading = readValue("GNSS_DualAnt_Heading");
  dataScaledMsg.gnss_dualant_stddev_heading = readValue("GNSS_DualAnt_Stddev_Heading");
  dataScaledMsg.gnss_dualant_pitch = readValue("GNSS_DualAnt_Pitch");
  dataScaledMsg.gnss_dualant_stddev_pitch = readValue("GNSS_DualAnt_Stddev_Pitch");
  dataScaledMsg.gnss_dualant_time_msec = readValue("GNSS_DualAnt_Time_msec");

  // angles
  dataScaledMsg.ins_roll = readValue("INS_Roll");
  dataScaledMsg.ins_pitch = readValue("INS_Pitch");
  dataScaledMsg.ins_yaw = readValue("INS_Yaw");
  dataScaledMsg.gnss_cog = readValue("GNSS_COG");
  dataScaledMsg.ins_stddev_roll = readValue("INS_Stddev_Roll");
  dataScaledMsg.ins_stddev_pitch = readValue("INS_Stddev_Pitch");
  dataScaledMsg.ins_stddev_yaw = readValue("INS_Stddev_Yaw");

  // ins position data
  dataScaledMsg.ins_lat_abs = readValue("INS_Lat_Abs");
  dataScaledMsg.ins_long_abs = readValue("INS_Long_Abs");
  dataScaledMsg.ins_height = readValue("INS_Height");
  dataScaledMsg.ins_stddev_lat = readValue("INS_Stddev_Lat");
  dataScaledMsg.ins_stddev_long = readValue("INS_Stddev_Long");
  dataScaledMsg.ins_stddev_height = readValue("INS_Stddev_Height");
  dataScaledMsg.ins_pos_rel_x = readValue("INS_Pos_Rel_X");
  dataScaledMsg.ins_pos_rel_y = readValue("INS_Pos_Rel_Y");
  dataScaledMsg.ins_time_msec = readValue("INS_Time_msec");
  dataScaledMsg.ins_time_week = readValue("INS_Time_Week");

  // ins frame velocities
  dataScaledMsg.ins_vel_frame.x = readValue("INS_Vel_Frame_X");
  dataScaledMsg.ins_vel_frame.y = readValue("INS_Vel_Frame_Y");
  dataScaledMsg.ins_vel_frame.z = readValue("INS_Vel_Frame_Z");

  // ins horizontal velocities
  dataScaledMsg.ins_vel_hor.x = readValue("INS_Vel_Hor_X");
  dataScaledMsg.ins_vel_hor.y = readValue("INS_Vel_Hor_Y");
  dataScaledMsg.ins_vel_hor.z = readValue("INS_Vel_Hor_Z");

  // ins velocity standarddeviations
  dataScaledMsg.ins_stddev_vel.x = readValue("INS_Stddev_Vel_X");
  dataScaledMsg.ins_stddev_vel.y = readValue("INS_Stddev_Vel_Y");
  dataScaledMsg.ins_stddev_vel.z = readValue("INS_Stddev_Vel_Z");

  // kalman filter stati
  dataScaledMsg.kf_lat_stimulated = readValue("KF_Lat_stimulated");
  dataScaledMsg.kf_long_stimulated = readValue("KF_Long_stimulated");
  dataScaledMsg.kf_steady_state = readValue("KF_Steady-State");
}

}  // end namespace tools
}  // end namespace genesys
RCLCPP_COMPONENTS_REGISTER_NODE(genesys::tools::GSDAServer)
