#include "adma_tools_cpp/gsda_server.hpp"

#include <arpa/inet.h>
#include <sys/socket.h>
#include <iostream>
#include <string>

#include <rclcpp_components/register_node_macro.hpp>

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
  gsdaFilePath_ = declare_parameter("gsda_file", "/home/rschilli/Documents/GeneSys/raw_data_60seconds_out_for.gsda");
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
  unsigned long offset_gps_unix = 315964800000;
  uint32_t week_to_msec = 604800000;
  unsigned long timestamp;

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
      imuMsg.header.stamp.sec = timestamp / 1000;
      imuMsg.header.stamp.nanosec = timestamp * 1000000;
      // read heading and velocity
      headingMsg.data = dataScaledMsg.ins_yaw;
      velMsg.data = std::sqrt(
                      std::pow(dataScaledMsg.ins_vel_frame.x, 2) +
                      std::pow(dataScaledMsg.ins_vel_frame.y, 2)) *
                    3.6;
      //TODO: handle status/error
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
    indexMap_.insert({row[i], i});
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
    dataScaledMsg.acc_frame.x = readValue("Acc_Frame_X");
    dataScaledMsg.acc_frame.y = readValue("Acc_Frame_Y");
    dataScaledMsg.acc_frame.z = readValue("Acc_Frame_Z");

    // rates body in deg/s
    dataScaledMsg.rate_body.x = readValue("Rate_Body_X");
    dataScaledMsg.rate_body.y = readValue("Rate_Body_Y");
    dataScaledMsg.rate_body.z = readValue("Rate_Body_Z");

    // rates hor in deg/s
    dataScaledMsg.rate_hor.x = readValue("Rate_Hor_X");
    dataScaledMsg.rate_hor.y = readValue("Rate_Hor_Y");
    dataScaledMsg.rate_hor.z = readValue("Rate_Hor_Z");

    // rates frame in deg/s
    dataScaledMsg.rate_frame.x = readValue("Rate_Frame_X");
    dataScaledMsg.rate_frame.y = readValue("Rate_Frame_Y");
    dataScaledMsg.rate_frame.z = readValue("Rate_Frame_Z");

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
    dataScaledMsg.poi_1.inv_path_radius = readValue("Acc_Hor_X_POI1");
    dataScaledMsg.poi_1.side_slip_angle = readValue("Acc_Hor_Y_POI1");
    dataScaledMsg.poi_1.dist_trav = readValue("Acc_Hor_Z_POI1");

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
    dataScaledMsg.poi_2.inv_path_radius = readValue("Acc_Hor_X_POI2");
    dataScaledMsg.poi_2.side_slip_angle = readValue("Acc_Hor_Y_POI2");
    dataScaledMsg.poi_2.dist_trav = readValue("Acc_Hor_Z_POI2");

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
    dataScaledMsg.ins_time_week = readValue("INS_Time_week");

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

  // poi 1
  // dataScaledMsg.poi_1.acc_body.x = std::stod(row[17]);
  // dataScaledMsg.poi_1.acc_body.y = std::stod(row[18]);
  // dataScaledMsg.poi_1.acc_body.z = std::stod(row[19]);

  // dataScaledMsg.poi_1.acc_hor.x = std::stod(row[20]);
  // dataScaledMsg.poi_1.acc_hor.y = std::stod(row[21]);
  // dataScaledMsg.poi_1.acc_hor.z = std::stod(row[22]);

  // dataScaledMsg.poi_1.inv_path_radius = std::stod(row[23]);
  // dataScaledMsg.poi_1.side_slip_angle = std::stod(row[24]);
  // dataScaledMsg.poi_1.dist_trav = std::stod(row[25]);

  // dataScaledMsg.poi_1.ins_lat_abs = std::stod(row[26]);
  // dataScaledMsg.poi_1.ins_lon_abs = std::stod(row[27]);
  // dataScaledMsg.poi_1.ins_height = std::stod(row[28]);

  // dataScaledMsg.poi_1.ins_pos_rel_x = std::stod(row[29]);
  // dataScaledMsg.poi_1.ins_pos_rel_y = std::stod(row[30]);

  // dataScaledMsg.poi_1.ins_vel_hor.x = std::stod(row[31]);
  // dataScaledMsg.poi_1.ins_vel_hor.y = std::stod(row[32]);
  // dataScaledMsg.poi_1.ins_vel_hor.z = std::stod(row[33]);

  // // poi 2
  // dataScaledMsg.poi_2.acc_body.x = std::stod(row[34]);
  // dataScaledMsg.poi_2.acc_body.y = std::stod(row[35]);
  // dataScaledMsg.poi_2.acc_body.z = std::stod(row[36]);

  // dataScaledMsg.poi_2.acc_hor.x = std::stod(row[37]);
  // dataScaledMsg.poi_2.acc_hor.y = std::stod(row[38]);
  // dataScaledMsg.poi_2.acc_hor.z = std::stod(row[39]);

  // dataScaledMsg.poi_2.inv_path_radius = std::stod(row[40]);
  // dataScaledMsg.poi_2.side_slip_angle = std::stod(row[41]);
  // dataScaledMsg.poi_2.dist_trav = std::stod(row[42]);

  // dataScaledMsg.poi_2.ins_lat_abs = std::stod(row[43]);
  // dataScaledMsg.poi_2.ins_lon_abs = std::stod(row[44]);
  // dataScaledMsg.poi_2.ins_height = std::stod(row[45]);

  // dataScaledMsg.poi_2.ins_pos_rel_x = std::stod(row[46]);
  // dataScaledMsg.poi_2.ins_pos_rel_y = std::stod(row[47]);

  // dataScaledMsg.poi_2.ins_vel_hor.x = std::stod(row[48]);
  // dataScaledMsg.poi_2.ins_vel_hor.y = std::stod(row[49]);
  // dataScaledMsg.poi_2.ins_vel_hor.z = std::stod(row[50]);

  // // poi 3
  // dataScaledMsg.poi_3.acc_body.x = std::stod(row[51]);
  // dataScaledMsg.poi_3.acc_body.y = std::stod(row[52]);
  // dataScaledMsg.poi_3.acc_body.z = std::stod(row[53]);

  // dataScaledMsg.poi_3.acc_hor.x = std::stod(row[54]);
  // dataScaledMsg.poi_3.acc_hor.y = std::stod(row[55]);
  // dataScaledMsg.poi_3.acc_hor.z = std::stod(row[56]);

  // dataScaledMsg.poi_3.inv_path_radius = std::stod(row[57]);
  // dataScaledMsg.poi_3.side_slip_angle = std::stod(row[58]);
  // dataScaledMsg.poi_3.dist_trav = std::stod(row[59]);

  // dataScaledMsg.poi_3.ins_lat_abs = std::stod(row[60]);
  // dataScaledMsg.poi_3.ins_lon_abs = std::stod(row[61]);
  // dataScaledMsg.poi_3.ins_height = std::stod(row[62]);

  // dataScaledMsg.poi_3.ins_pos_rel_x = std::stod(row[63]);
  // dataScaledMsg.poi_3.ins_pos_rel_y = std::stod(row[64]);

  // dataScaledMsg.poi_3.ins_vel_hor.x = std::stod(row[65]);
  // dataScaledMsg.poi_3.ins_vel_hor.y = std::stod(row[66]);
  // dataScaledMsg.poi_3.ins_vel_hor.z = std::stod(row[67]);

  // // poi 4
  // dataScaledMsg.poi_4.acc_body.x = std::stod(row[68]);
  // dataScaledMsg.poi_4.acc_body.y = std::stod(row[69]);
  // dataScaledMsg.poi_4.acc_body.z = std::stod(row[70]);

  // dataScaledMsg.poi_4.acc_hor.x = std::stod(row[71]);
  // dataScaledMsg.poi_4.acc_hor.y = std::stod(row[72]);
  // dataScaledMsg.poi_4.acc_hor.z = std::stod(row[73]);

  // dataScaledMsg.poi_4.inv_path_radius = std::stod(row[74]);
  // dataScaledMsg.poi_4.side_slip_angle = std::stod(row[75]);
  // dataScaledMsg.poi_4.dist_trav = std::stod(row[76]);

  // dataScaledMsg.poi_4.ins_lat_abs = std::stod(row[77]);
  // dataScaledMsg.poi_4.ins_lon_abs = std::stod(row[78]);
  // dataScaledMsg.poi_4.ins_height = std::stod(row[79]);

  // dataScaledMsg.poi_4.ins_pos_rel_x = std::stod(row[80]);
  // dataScaledMsg.poi_4.ins_pos_rel_y = std::stod(row[81]);

  // dataScaledMsg.poi_4.ins_vel_hor.x = std::stod(row[82]);
  // dataScaledMsg.poi_4.ins_vel_hor.y = std::stod(row[83]);
  // dataScaledMsg.poi_4.ins_vel_hor.z = std::stod(row[84]);

  // // poi 5
  // dataScaledMsg.poi_5.acc_body.x = std::stod(row[85]);
  // dataScaledMsg.poi_5.acc_body.y = std::stod(row[86]);
  // dataScaledMsg.poi_5.acc_body.z = std::stod(row[87]);

  // dataScaledMsg.poi_5.acc_hor.x = std::stod(row[88]);
  // dataScaledMsg.poi_5.acc_hor.y = std::stod(row[89]);
  // dataScaledMsg.poi_5.acc_hor.z = std::stod(row[90]);

  // dataScaledMsg.poi_5.inv_path_radius = std::stod(row[91]);
  // dataScaledMsg.poi_5.side_slip_angle = std::stod(row[92]);
  // dataScaledMsg.poi_5.dist_trav = std::stod(row[93]);

  // dataScaledMsg.poi_5.ins_lat_abs = std::stod(row[94]);
  // dataScaledMsg.poi_5.ins_lon_abs = std::stod(row[95]);
  // dataScaledMsg.poi_5.ins_height = std::stod(row[96]);

  // dataScaledMsg.poi_5.ins_pos_rel_x = std::stod(row[97]);
  // dataScaledMsg.poi_5.ins_pos_rel_y = std::stod(row[98]);

  // dataScaledMsg.poi_5.ins_vel_hor.x = std::stod(row[99]);
  // dataScaledMsg.poi_5.ins_vel_hor.y = std::stod(row[100]);
  // dataScaledMsg.poi_5.ins_vel_hor.z = std::stod(row[101]);

  // // poi 6
  // dataScaledMsg.poi_6.acc_body.x = std::stod(row[102]);
  // dataScaledMsg.poi_6.acc_body.y = std::stod(row[103]);
  // dataScaledMsg.poi_6.acc_body.z = std::stod(row[104]);

  // dataScaledMsg.poi_6.acc_hor.x = std::stod(row[105]);
  // dataScaledMsg.poi_6.acc_hor.y = std::stod(row[106]);
  // dataScaledMsg.poi_6.acc_hor.z = std::stod(row[107]);

  // dataScaledMsg.poi_6.inv_path_radius = std::stod(row[108]);
  // dataScaledMsg.poi_6.side_slip_angle = std::stod(row[109]);
  // dataScaledMsg.poi_6.dist_trav = std::stod(row[110]);

  // dataScaledMsg.poi_6.ins_lat_abs = std::stod(row[111]);
  // dataScaledMsg.poi_6.ins_lon_abs = std::stod(row[112]);
  // dataScaledMsg.poi_6.ins_height = std::stod(row[113]);

  // dataScaledMsg.poi_6.ins_pos_rel_x = std::stod(row[114]);
  // dataScaledMsg.poi_6.ins_pos_rel_y = std::stod(row[115]);

  // dataScaledMsg.poi_6.ins_vel_hor.x = std::stod(row[116]);
  // dataScaledMsg.poi_6.ins_vel_hor.y = std::stod(row[117]);
  // dataScaledMsg.poi_6.ins_vel_hor.z = std::stod(row[118]);

  // // poi 7
  // dataScaledMsg.poi_7.acc_body.x = std::stod(row[119]);
  // dataScaledMsg.poi_7.acc_body.y = std::stod(row[120]);
  // dataScaledMsg.poi_7.acc_body.z = std::stod(row[121]);

  // dataScaledMsg.poi_7.acc_hor.x = std::stod(row[122]);
  // dataScaledMsg.poi_7.acc_hor.y = std::stod(row[123]);
  // dataScaledMsg.poi_7.acc_hor.z = std::stod(row[124]);

  // dataScaledMsg.poi_7.inv_path_radius = std::stod(row[125]);
  // dataScaledMsg.poi_7.side_slip_angle = std::stod(row[126]);
  // dataScaledMsg.poi_7.dist_trav = std::stod(row[127]);

  // dataScaledMsg.poi_7.ins_lat_abs = std::stod(row[128]);
  // dataScaledMsg.poi_7.ins_lon_abs = std::stod(row[129]);
  // dataScaledMsg.poi_7.ins_height = std::stod(row[130]);

  // dataScaledMsg.poi_7.ins_pos_rel_x = std::stod(row[131]);
  // dataScaledMsg.poi_7.ins_pos_rel_y = std::stod(row[132]);

  // dataScaledMsg.poi_7.ins_vel_hor.x = std::stod(row[133]);
  // dataScaledMsg.poi_7.ins_vel_hor.y = std::stod(row[134]);
  // dataScaledMsg.poi_7.ins_vel_hor.z = std::stod(row[135]);

  // // poi 8
  // dataScaledMsg.poi_8.acc_body.x = std::stod(row[136]);
  // dataScaledMsg.poi_8.acc_body.y = std::stod(row[137]);
  // dataScaledMsg.poi_8.acc_body.z = std::stod(row[138]);

  // dataScaledMsg.poi_8.acc_hor.x = std::stod(row[139]);
  // dataScaledMsg.poi_8.acc_hor.y = std::stod(row[140]);
  // dataScaledMsg.poi_8.acc_hor.z = std::stod(row[141]);

  // dataScaledMsg.poi_8.inv_path_radius = std::stod(row[142]);
  // dataScaledMsg.poi_8.side_slip_angle = std::stod(row[143]);
  // dataScaledMsg.poi_8.dist_trav = std::stod(row[144]);

  // dataScaledMsg.poi_8.ins_lat_abs = std::stod(row[145]);
  // dataScaledMsg.poi_8.ins_lon_abs = std::stod(row[146]);
  // dataScaledMsg.poi_8.ins_height = std::stod(row[147]);

  // dataScaledMsg.poi_8.ins_pos_rel_x = std::stod(row[148]);
  // dataScaledMsg.poi_8.ins_pos_rel_y = std::stod(row[149]);

  // dataScaledMsg.poi_8.ins_vel_hor.x = std::stod(row[150]);
  // dataScaledMsg.poi_8.ins_vel_hor.y = std::stod(row[151]);
  // dataScaledMsg.poi_8.ins_vel_hor.z = std::stod(row[152]);

  // // accels
  // dataScaledMsg.acc_body.x = std::stod(row[153]);
  // dataScaledMsg.acc_body.y = std::stod(row[154]);
  // dataScaledMsg.acc_body.z = std::stod(row[155]);

  // dataScaledMsg.acc_hor.x = std::stod(row[156]);
  // dataScaledMsg.acc_hor.y = std::stod(row[157]);
  // dataScaledMsg.acc_hor.z = std::stod(row[158]);

  // dataScaledMsg.acc_body_hr.x = std::stod(row[159]);
  // dataScaledMsg.acc_body_hr.y = std::stod(row[160]);
  // dataScaledMsg.acc_body_hr.z = std::stod(row[161]);

  // // rates
  // dataScaledMsg.rate_body.x = std::stod(row[162]);
  // dataScaledMsg.rate_body.y = std::stod(row[163]);
  // dataScaledMsg.rate_body.z = std::stod(row[164]);

  // dataScaledMsg.rate_hor.x = std::stod(row[165]);
  // dataScaledMsg.rate_hor.y = std::stod(row[166]);
  // dataScaledMsg.rate_hor.z = std::stod(row[167]);

  // dataScaledMsg.rate_body_hr.x = std::stod(row[168]);
  // dataScaledMsg.rate_body_hr.y = std::stod(row[169]);
  // dataScaledMsg.rate_body_hr.z = std::stod(row[170]);

  // // ext vel
  // dataScaledMsg.ext_vel_x_corrected = std::stod(row[171]);
 
  // // system data
  // dataScaledMsg.system_ta = std::stoul(row[172]);
  // dataScaledMsg.system_temp = std::stod(row[173]);
  // dataScaledMsg.system_dsp_load = std::stod(row[174]);
  // dataScaledMsg.system_time_since_init = std::stoul(row[175]);
 
  // // aux
  // dataScaledMsg.inv_path_radius = std::stod(row[176]);
  // dataScaledMsg.side_slip_angle = std::stod(row[177]);
  // dataScaledMsg.dist_trav = std::stod(row[178]);
 
  // // gnss data
  // dataScaledMsg.gnss_lat_abs = std::stod(row[179]);
  // dataScaledMsg.gnss_long_abs = std::stod(row[180]);
  // dataScaledMsg.gnss_height = std::stod(row[181]);
  // dataScaledMsg.gnss_pos_rel_x = std::stod(row[182]);
  // dataScaledMsg.gnss_pos_rel_y = std::stod(row[183]);
  // dataScaledMsg.gnss_stddev_lat = std::stod(row[184]);
  // dataScaledMsg.gnss_stddev_long = std::stod(row[185]);
  // dataScaledMsg.gnss_stddev_height = std::stod(row[186]);
  // dataScaledMsg.gnss_vel_frame.x = std::stod(row[187]);
  // dataScaledMsg.gnss_vel_frame.y = std::stod(row[188]);
  // dataScaledMsg.gnss_vel_frame.z = std::stod(row[189]);
  // dataScaledMsg.gnss_vel_latency = std::stod(row[190]);
  // dataScaledMsg.gnss_stddev_vel.x = std::stod(row[191]);
  // dataScaledMsg.gnss_stddev_vel.y = std::stod(row[192]);
  // dataScaledMsg.gnss_stddev_vel.z = std::stod(row[193]);
  // dataScaledMsg.gnss_log_delay = std::stod(row[194]);
  // dataScaledMsg.gnss_diffage = std::stod(row[195]);
  // dataScaledMsg.gnss_sats_visible = std::stod(row[196]);
  // dataScaledMsg.gnss_time_msec = std::stoul(row[197]);
  // dataScaledMsg.gnss_time_week = std::stoul(row[198]);
  // dataScaledMsg.gnss_dualant_heading = std::stod(row[199]);
  // dataScaledMsg.gnss_dualant_stddev_heading = std::stod(row[200]);
  // dataScaledMsg.gnss_dualant_pitch = std::stod(row[201]);
  // dataScaledMsg.gnss_dualant_stddev_pitch = std::stod(row[202]);
  // dataScaledMsg.gnss_dualant_time_msec = std::stoul(row[203]);

  // // ins channels
  // // angles
  // dataScaledMsg.ins_roll = std::stod(row[204]);
  // dataScaledMsg.ins_pitch = std::stod(row[205]);
  // dataScaledMsg.ins_yaw = std::stod(row[206]);
  // dataScaledMsg.gnss_cog = std::stod(row[207]);

  // dataScaledMsg.ins_stddev_roll = std::stod(row[208]);
  // dataScaledMsg.ins_stddev_pitch = std::stod(row[209]);
  // dataScaledMsg.ins_stddev_yaw = std::stod(row[210]);

  // // rel pos
  // dataScaledMsg.ins_pos_rel_x = std::stod(row[211]);
  // dataScaledMsg.ins_pos_rel_y = std::stod(row[212]);

  // // velocities
  // // frame
  // dataScaledMsg.ins_vel_frame.x = std::stod(row[213]);
  // dataScaledMsg.ins_vel_frame.y = std::stod(row[214]);
  // dataScaledMsg.ins_vel_frame.z = std::stod(row[215]);

  // // horizontal
  // dataScaledMsg.ins_vel_hor.x = std::stod(row[216]);
  // dataScaledMsg.ins_vel_hor.y = std::stod(row[217]);
  // dataScaledMsg.ins_vel_hor.z = std::stod(row[218]);

  // // stddev
  // dataScaledMsg.ins_stddev_vel.x = std::stod(row[219]);
  // dataScaledMsg.ins_stddev_vel.y = std::stod(row[220]);
  // dataScaledMsg.ins_stddev_vel.z = std::stod(row[221]);

  // // kf 
  // dataScaledMsg.kf_lat_stimulated = std::stoul(row[222]);
  // dataScaledMsg.kf_long_stimulated = std::stoul(row[223]);
  // dataScaledMsg.kf_steady_state = std::stoul(row[224]);
}

}  // end namespace tools
}  // end namespace genesys
RCLCPP_COMPONENTS_REGISTER_NODE(genesys::tools::GSDAServer)
