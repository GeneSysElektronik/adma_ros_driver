#include "adma_tools_cpp/gsda_server.hpp"

#include <arpa/inet.h>
#include <sys/socket.h>
#include <iostream>
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
GSDAServer::GSDAServer(const rclcpp::NodeOptions & options)
: Node("gsda_server", options), 
msgCounter_(0)
{
  // read ros parameters
  frequency_ = this->declare_parameter("frequency", 100);
  gsdaFilePath_ = declare_parameter("gsda_file", "/home/$USER/$ROS2_WS/data/$FILENAME.gsda");
  gsdaFile_ = std::fstream(gsdaFilePath_);
  if(gsdaFile_)
  {
    RCLCPP_INFO(get_logger(), "Loaded GSDA-File: %s", gsdaFilePath_.c_str());
  }else
  {
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

  parser_ = new ADMA2ROSParser("v3.3.4");

  updateLoop();
}

GSDAServer::~GSDAServer() 
{ 
  RCLCPP_INFO(get_logger(), "GSDA file streaming done.. Read %ld messages from file", msgCounter_);
}

void GSDAServer::updateLoop()
{

  // define messages to publish
  //TODO: inject frame_id as ROS params like its done in driver node
  adma_ros_driver_msgs::msg::AdmaDataScaled dataScaledMsg;
  dataScaledMsg.header.frame_id = "adma";
  adma_ros_driver_msgs::msg::AdmaStatus stateMsg;
  stateMsg.header.frame_id = "adma_status";
  std_msgs::msg::Float64 velMsg;
  std_msgs::msg::Float64 headingMsg;
  sensor_msgs::msg::Imu imuMsg;
  imuMsg.header.frame_id = "imu_link";
  sensor_msgs::msg::NavSatFix navsatfixMsg;
  navsatfixMsg.header.frame_id = "gnss_link";

  //offset between UNIX and GNSS (in ms)
  unsigned long long offset_gps_unix = 315964800000;
  uint32_t week_to_msec = 604800000;
  unsigned long long timestamp;

  while(rclcpp::ok())
  {
    //iterate through gsda file
    while(getline(gsdaFile_, line))
    {
      if(msgCounter_ == 0){
        readLine();
        extractHeader();
        msgCounter_++;
        continue;
      }
      else if(msgCounter_ == 1)
      {
        //skip first 2 lines cause they are not used here
        msgCounter_++;
        continue;
      }
      readLine();
      
      fillDataScaledMsg(dataScaledMsg);
      timestamp = dataScaledMsg.ins_time_msec + offset_gps_unix;
      timestamp += dataScaledMsg.ins_time_week * week_to_msec;
      dataScaledMsg.time_msec = timestamp;
      dataScaledMsg.time_nsec = timestamp * 1000000;
      dataScaledMsg.header.stamp.sec = timestamp / 1000;
      dataScaledMsg.header.stamp.nanosec = timestamp * 1000000;
      // extract separate msgs
      parser_->extractNavSatFix(dataScaledMsg, navsatfixMsg);
      navsatfixMsg.header.stamp.sec = timestamp / 1000;
      navsatfixMsg.header.stamp.nanosec = timestamp * 1000000;
      parser_->extractIMU(dataScaledMsg, imuMsg);
      // ADMA PP doesnt provide "hr" channels so use normal body rate/acc for IMU
      imuMsg.linear_acceleration.x = dataScaledMsg.acc_body.x * 9.81;
      imuMsg.linear_acceleration.y = dataScaledMsg.acc_body.y * 9.81;
      imuMsg.linear_acceleration.z = dataScaledMsg.acc_body.z * 9.81;
      imuMsg.angular_velocity.x = dataScaledMsg.rate_body.x * PI / 180.0;
      imuMsg.angular_velocity.y = dataScaledMsg.rate_body.y * PI / 180.0;
      imuMsg.angular_velocity.z = dataScaledMsg.rate_body.z * PI / 180.0;
      imuMsg.header.stamp.sec = timestamp / 1000;
      imuMsg.header.stamp.nanosec = timestamp * 1000000;
      // read heading and velocity
      headingMsg.data = dataScaledMsg.ins_yaw;
      velMsg.data = std::sqrt(
                      std::pow(dataScaledMsg.ins_vel_frame.x, 2) +
                      std::pow(dataScaledMsg.ins_vel_frame.y, 2)) *
                    3.6;
      
      extractBytes(stateMsg, dataScaledMsg);
      stateMsg.header.stamp.sec = timestamp / 1000;
      stateMsg.header.stamp.nanosec = timestamp * 1000000;

      pub_adma_data_scaled_->publish(dataScaledMsg);
      pub_adma_status_->publish(stateMsg);
      pub_navsat_fix_->publish(navsatfixMsg);
      pub_imu_->publish(imuMsg);
      pub_heading_->publish(headingMsg);
      pub_velocity_->publish(velMsg);    

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
  while(getline(str, word, ','))
      row.push_back(word);
}

void GSDAServer::extractHeader()
{
  for(int i = 0; i < row.size(); i++)
  {
    std::string channelName = row[i];
    if (channelName.rfind("% ", 0) == 0) {
      channelName.erase(0,2);
    }
    indexMap_.insert({channelName, i});
  }
}

double GSDAServer::readValue(std::string dataName)
{
  auto it = indexMap_.find(dataName);
  if(it != indexMap_.end())
  {
    int index = (*it).second;
    if(index <= row.size())
    {
      return std::stod(row[index]);
    }
  }
  return 0.0;  
}

int GSDAServer::readByteValue(std::string dataName)
{
  auto it = indexMap_.find(dataName);
  if(it != indexMap_.end())
  {
    int index = (*it).second;
    if(index <= row.size())
    {
      return std::stoi(row[index].c_str());
    }
  }
  RCLCPP_WARN(get_logger(), "Channelname %s not found in GSDA File..", dataName.c_str());
  return 0;  
}

void GSDAServer::extractBytes(adma_ros_driver_msgs::msg::AdmaStatus &stateMsg, adma_ros_driver_msgs::msg::AdmaDataScaled& dataScaledMsg)
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
  //fill error/warning bytes
  stateMsg.error_warnings_bytes.error_1 = error1;
  stateMsg.error_warnings_bytes.error_2 = error2;
  stateMsg.error_warnings_bytes.warn_1 = warn1;
  stateMsg.error_warnings_bytes.error_3 = error3;
  dataScaledMsg.error_warning.error_1 = error1;
  dataScaledMsg.error_warning.error_2 = error2;
  dataScaledMsg.error_warning.warn_1 = warn1;
  dataScaledMsg.error_warning.error_3 = error3;

  // parse SEW bits of bytes
  parser_->parserV334_.mapStatusBit0(dataScaledMsg.status, gnssStatus);
  parser_->parserV334_.mapStatusBit1(dataScaledMsg.status, signalInStatus);
  parser_->parserV334_.mapStatusBit2(dataScaledMsg.status, miscStatus);
  parser_->parserV334_.mapStatusBit4(dataScaledMsg.status, kfStatus);
  parser_->parserV334_.mapStatusBit5(dataScaledMsg.status, statusRobot);

  parser_->parserV334_.mapErrorBit0(stateMsg.error_warnings, error1);
  parser_->parserV334_.mapErrorBit1(stateMsg.error_warnings, error2);
  parser_->parserV334_.mapWarningBit0(stateMsg.error_warnings, warn1);
  parser_->parserV334_.mapErrorBit2(stateMsg.error_warnings, error3);
}

void GSDAServer::fillDataScaledMsg(adma_ros_driver_msgs::msg::AdmaDataScaled& dataScaledMsg)
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
    dataScaledMsg.poi_1.ins_lon_abs = readValue("INS_Long_Abs_POI1");
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
    dataScaledMsg.poi_2.ins_lon_abs = readValue("INS_Long_Abs_POI2");
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
    dataScaledMsg.poi_3.ins_lon_abs = readValue("INS_Long_Abs_POI3");
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
    dataScaledMsg.poi_4.ins_lon_abs = readValue("INS_Long_Abs_POI4");
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
    dataScaledMsg.poi_5.ins_lon_abs = readValue("INS_Long_Abs_POI5");
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
    dataScaledMsg.poi_6.ins_lon_abs = readValue("INS_Long_Abs_POI6");
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
    dataScaledMsg.poi_7.ins_lon_abs = readValue("INS_Long_Abs_POI7");
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
    dataScaledMsg.poi_8.ins_lon_abs = readValue("INS_Long_Abs_POI8");
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
