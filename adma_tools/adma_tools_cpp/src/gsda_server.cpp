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
      if(msgCounter_ <= 1)
      {
        //skip first 2 lines cause they are not used here
        msgCounter_++;
        continue;
      }
      row.clear();
      std::stringstream str(line);
      //extract lines
      while(getline(str, word, ','))
          row.push_back(word);
      
      fillDataScaledMsg(dataScaledMsg, row);
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

void GSDAServer::fillDataScaledMsg(adma_ros_driver_msgs::msg::AdmaDataScaled& dataScaledMsg, std::vector<std::string>row)
{
  // ins position data
  dataScaledMsg.ins_lat_abs = std::stod(row[9]);
  dataScaledMsg.ins_long_abs = std::stod(row[10]);
  dataScaledMsg.ins_height = std::stod(row[11]);
  dataScaledMsg.ins_stddev_lat = std::stod(row[12]);
  dataScaledMsg.ins_stddev_long = std::stod(row[13]);
  dataScaledMsg.ins_stddev_height = std::stod(row[14]);
  dataScaledMsg.ins_time_msec = std::stod(row[15]);
  dataScaledMsg.ins_time_week = std::stod(row[16]);

  // poi 1
  dataScaledMsg.poi_1.acc_body.x = std::stod(row[17]);
  dataScaledMsg.poi_1.acc_body.y = std::stod(row[18]);
  dataScaledMsg.poi_1.acc_body.z = std::stod(row[19]);

  dataScaledMsg.poi_1.acc_hor.x = std::stod(row[20]);
  dataScaledMsg.poi_1.acc_hor.y = std::stod(row[21]);
  dataScaledMsg.poi_1.acc_hor.z = std::stod(row[22]);

  dataScaledMsg.poi_1.inv_path_radius = std::stod(row[23]);
  dataScaledMsg.poi_1.side_slip_angle = std::stod(row[24]);
  dataScaledMsg.poi_1.dist_trav = std::stod(row[25]);

  dataScaledMsg.poi_1.ins_lat_abs = std::stod(row[26]);
  dataScaledMsg.poi_1.ins_long_abs = std::stod(row[27]);
  dataScaledMsg.poi_1.ins_height = std::stod(row[28]);

  dataScaledMsg.poi_1.pos_rel.x = std::stod(row[29]);
  dataScaledMsg.poi_1.pos_rel.y = std::stod(row[30]);

  dataScaledMsg.poi_1.ins_vel_hor.x = std::stod(row[31]);
  dataScaledMsg.poi_1.ins_vel_hor.y = std::stod(row[32]);
  dataScaledMsg.poi_1.ins_vel_hor.z = std::stod(row[33]);

  // poi 2
  dataScaledMsg.poi_2.acc_body.x = std::stod(row[34]);
  dataScaledMsg.poi_2.acc_body.y = std::stod(row[35]);
  dataScaledMsg.poi_2.acc_body.z = std::stod(row[36]);

  dataScaledMsg.poi_2.acc_hor.x = std::stod(row[37]);
  dataScaledMsg.poi_2.acc_hor.y = std::stod(row[38]);
  dataScaledMsg.poi_2.acc_hor.z = std::stod(row[39]);

  dataScaledMsg.poi_2.inv_path_radius = std::stod(row[40]);
  dataScaledMsg.poi_2.side_slip_angle = std::stod(row[41]);
  dataScaledMsg.poi_2.dist_trav = std::stod(row[42]);

  dataScaledMsg.poi_2.ins_lat_abs = std::stod(row[43]);
  dataScaledMsg.poi_2.ins_long_abs = std::stod(row[44]);
  dataScaledMsg.poi_2.ins_height = std::stod(row[45]);

  dataScaledMsg.poi_2.pos_rel.x = std::stod(row[46]);
  dataScaledMsg.poi_2.pos_rel.y = std::stod(row[47]);

  dataScaledMsg.poi_2.ins_vel_hor.x = std::stod(row[48]);
  dataScaledMsg.poi_2.ins_vel_hor.y = std::stod(row[49]);
  dataScaledMsg.poi_2.ins_vel_hor.z = std::stod(row[50]);

  // poi 3
  dataScaledMsg.poi_3.acc_body.x = std::stod(row[51]);
  dataScaledMsg.poi_3.acc_body.y = std::stod(row[52]);
  dataScaledMsg.poi_3.acc_body.z = std::stod(row[53]);

  dataScaledMsg.poi_3.acc_hor.x = std::stod(row[54]);
  dataScaledMsg.poi_3.acc_hor.y = std::stod(row[55]);
  dataScaledMsg.poi_3.acc_hor.z = std::stod(row[56]);

  dataScaledMsg.poi_3.inv_path_radius = std::stod(row[57]);
  dataScaledMsg.poi_3.side_slip_angle = std::stod(row[58]);
  dataScaledMsg.poi_3.dist_trav = std::stod(row[59]);

  dataScaledMsg.poi_3.ins_lat_abs = std::stod(row[60]);
  dataScaledMsg.poi_3.ins_long_abs = std::stod(row[61]);
  dataScaledMsg.poi_3.ins_height = std::stod(row[62]);

  dataScaledMsg.poi_3.pos_rel.x = std::stod(row[63]);
  dataScaledMsg.poi_3.pos_rel.y = std::stod(row[64]);

  dataScaledMsg.poi_3.ins_vel_hor.x = std::stod(row[65]);
  dataScaledMsg.poi_3.ins_vel_hor.y = std::stod(row[66]);
  dataScaledMsg.poi_3.ins_vel_hor.z = std::stod(row[67]);

  // poi 4
  dataScaledMsg.poi_4.acc_body.x = std::stod(row[68]);
  dataScaledMsg.poi_4.acc_body.y = std::stod(row[69]);
  dataScaledMsg.poi_4.acc_body.z = std::stod(row[70]);

  dataScaledMsg.poi_4.acc_hor.x = std::stod(row[71]);
  dataScaledMsg.poi_4.acc_hor.y = std::stod(row[72]);
  dataScaledMsg.poi_4.acc_hor.z = std::stod(row[73]);

  dataScaledMsg.poi_4.inv_path_radius = std::stod(row[74]);
  dataScaledMsg.poi_4.side_slip_angle = std::stod(row[75]);
  dataScaledMsg.poi_4.dist_trav = std::stod(row[76]);

  dataScaledMsg.poi_4.ins_lat_abs = std::stod(row[77]);
  dataScaledMsg.poi_4.ins_long_abs = std::stod(row[78]);
  dataScaledMsg.poi_4.ins_height = std::stod(row[79]);

  dataScaledMsg.poi_4.pos_rel.x = std::stod(row[80]);
  dataScaledMsg.poi_4.pos_rel.y = std::stod(row[81]);

  dataScaledMsg.poi_4.ins_vel_hor.x = std::stod(row[82]);
  dataScaledMsg.poi_4.ins_vel_hor.y = std::stod(row[83]);
  dataScaledMsg.poi_4.ins_vel_hor.z = std::stod(row[84]);

  // poi 5
  dataScaledMsg.poi_5.acc_body.x = std::stod(row[85]);
  dataScaledMsg.poi_5.acc_body.y = std::stod(row[86]);
  dataScaledMsg.poi_5.acc_body.z = std::stod(row[87]);

  dataScaledMsg.poi_5.acc_hor.x = std::stod(row[88]);
  dataScaledMsg.poi_5.acc_hor.y = std::stod(row[89]);
  dataScaledMsg.poi_5.acc_hor.z = std::stod(row[90]);

  dataScaledMsg.poi_5.inv_path_radius = std::stod(row[91]);
  dataScaledMsg.poi_5.side_slip_angle = std::stod(row[92]);
  dataScaledMsg.poi_5.dist_trav = std::stod(row[93]);

  dataScaledMsg.poi_5.ins_lat_abs = std::stod(row[94]);
  dataScaledMsg.poi_5.ins_long_abs = std::stod(row[95]);
  dataScaledMsg.poi_5.ins_height = std::stod(row[96]);

  dataScaledMsg.poi_5.pos_rel.x = std::stod(row[97]);
  dataScaledMsg.poi_5.pos_rel.y = std::stod(row[98]);

  dataScaledMsg.poi_5.ins_vel_hor.x = std::stod(row[99]);
  dataScaledMsg.poi_5.ins_vel_hor.y = std::stod(row[100]);
  dataScaledMsg.poi_5.ins_vel_hor.z = std::stod(row[101]);

  // poi 6
  dataScaledMsg.poi_6.acc_body.x = std::stod(row[102]);
  dataScaledMsg.poi_6.acc_body.y = std::stod(row[103]);
  dataScaledMsg.poi_6.acc_body.z = std::stod(row[104]);

  dataScaledMsg.poi_6.acc_hor.x = std::stod(row[105]);
  dataScaledMsg.poi_6.acc_hor.y = std::stod(row[106]);
  dataScaledMsg.poi_6.acc_hor.z = std::stod(row[107]);

  dataScaledMsg.poi_6.inv_path_radius = std::stod(row[108]);
  dataScaledMsg.poi_6.side_slip_angle = std::stod(row[109]);
  dataScaledMsg.poi_6.dist_trav = std::stod(row[110]);

  dataScaledMsg.poi_6.ins_lat_abs = std::stod(row[111]);
  dataScaledMsg.poi_6.ins_long_abs = std::stod(row[112]);
  dataScaledMsg.poi_6.ins_height = std::stod(row[113]);

  dataScaledMsg.poi_6.pos_rel.x = std::stod(row[114]);
  dataScaledMsg.poi_6.pos_rel.y = std::stod(row[115]);

  dataScaledMsg.poi_6.ins_vel_hor.x = std::stod(row[116]);
  dataScaledMsg.poi_6.ins_vel_hor.y = std::stod(row[117]);
  dataScaledMsg.poi_6.ins_vel_hor.z = std::stod(row[118]);

  // poi 7
  dataScaledMsg.poi_7.acc_body.x = std::stod(row[119]);
  dataScaledMsg.poi_7.acc_body.y = std::stod(row[120]);
  dataScaledMsg.poi_7.acc_body.z = std::stod(row[121]);

  dataScaledMsg.poi_7.acc_hor.x = std::stod(row[122]);
  dataScaledMsg.poi_7.acc_hor.y = std::stod(row[123]);
  dataScaledMsg.poi_7.acc_hor.z = std::stod(row[124]);

  dataScaledMsg.poi_7.inv_path_radius = std::stod(row[125]);
  dataScaledMsg.poi_7.side_slip_angle = std::stod(row[126]);
  dataScaledMsg.poi_7.dist_trav = std::stod(row[127]);

  dataScaledMsg.poi_7.ins_lat_abs = std::stod(row[128]);
  dataScaledMsg.poi_7.ins_long_abs = std::stod(row[129]);
  dataScaledMsg.poi_7.ins_height = std::stod(row[130]);

  dataScaledMsg.poi_7.pos_rel.x = std::stod(row[131]);
  dataScaledMsg.poi_7.pos_rel.y = std::stod(row[132]);

  dataScaledMsg.poi_7.ins_vel_hor.x = std::stod(row[133]);
  dataScaledMsg.poi_7.ins_vel_hor.y = std::stod(row[134]);
  dataScaledMsg.poi_7.ins_vel_hor.z = std::stod(row[135]);

  // poi 8
  dataScaledMsg.poi_8.acc_body.x = std::stod(row[136]);
  dataScaledMsg.poi_8.acc_body.y = std::stod(row[137]);
  dataScaledMsg.poi_8.acc_body.z = std::stod(row[138]);

  dataScaledMsg.poi_8.acc_hor.x = std::stod(row[139]);
  dataScaledMsg.poi_8.acc_hor.y = std::stod(row[140]);
  dataScaledMsg.poi_8.acc_hor.z = std::stod(row[141]);

  dataScaledMsg.poi_8.inv_path_radius = std::stod(row[142]);
  dataScaledMsg.poi_8.side_slip_angle = std::stod(row[143]);
  dataScaledMsg.poi_8.dist_trav = std::stod(row[144]);

  dataScaledMsg.poi_8.ins_lat_abs = std::stod(row[145]);
  dataScaledMsg.poi_8.ins_long_abs = std::stod(row[146]);
  dataScaledMsg.poi_8.ins_height = std::stod(row[147]);

  dataScaledMsg.poi_8.pos_rel.x = std::stod(row[148]);
  dataScaledMsg.poi_8.pos_rel.y = std::stod(row[149]);

  dataScaledMsg.poi_8.ins_vel_hor.x = std::stod(row[150]);
  dataScaledMsg.poi_8.ins_vel_hor.y = std::stod(row[151]);
  dataScaledMsg.poi_8.ins_vel_hor.z = std::stod(row[152]);

  // accels
  dataScaledMsg.acc_body.x = std::stod(row[153]);
  dataScaledMsg.acc_body.Y = std::stod(row[154]);
  dataScaledMsg.acc_body.z = std::stod(row[155]);

  dataScaledMsg.acc_hor.x = std::stod(row[156]);
  dataScaledMsg.acc_hor.Y = std::stod(row[157]);
  dataScaledMsg.acc_hor.z = std::stod(row[158]);

  dataScaledMsg.accel_frame.x = std::stod(row[159]);
  dataScaledMsg.accel_frame.Y = std::stod(row[160]);
  dataScaledMsg.accel_frame.z = std::stod(row[161]);

  // rates
  dataScaledMsg.rate_body.x = std::stod(row[162]);
  dataScaledMsg.rate_body.Y = std::stod(row[163]);
  dataScaledMsg.rate_body.z = std::stod(row[164]);

  dataScaledMsg.rate_hor.x = std::stod(row[165]);
  dataScaledMsg.rate_hor.Y = std::stod(row[166]);
  dataScaledMsg.rate_hor.z = std::stod(row[167]);

  dataScaledMsg.rate_frame.x = std::stod(row[168]);
  dataScaledMsg.rate_frame.Y = std::stod(row[169]);
  dataScaledMsg.rate_frame.z = std::stod(row[170]);

  // ext vel
  dataScaledMsg.ext_vel_x_corrected = std::stod(row[171]);
 
  // system data
  dataScaledMsg.system_ta = std::stoul(row[172]);
  dataScaledMsg.system_temp = std::stod(row[173]);
  dataScaledMsg.system_dsp_load = std::stod(row[174);
  dataScaledMsg.system_timesinceinit = std::stoul(row[175]);
 
  // aux
  dataScaledMsg.inv_path_radius = std::stod(row[176]);
  dataScaledMsg.sie_slip_angle = std::stod(row[177]);
  dataScaledMsg.dist_trav = std::stod(row[178]);
 
  // gnss data
  dataScaledMsg.gnss_lat_abs = std::stod(row[179]);
  dataScaledMsg.gnss_long_abs = std::stod(row[180]);
  dataScaledMsg.gnss_height = std::stod(row[181]);
  dataScaledMsg.gnss_pos_rel_x = std::stod(row[182]);
  dataScaledMsg.gnss_pos_rel_y = std::stod(row[183]);
  dataScaledMsg.gnss_stddev_lat = std::stod(row[184]);
  dataScaledMsg.gnss_stddev_long = std::stod(row[185]);
  dataScaledMsg.gnss_stddev_height = std::stod(row[186]);
  dataScaledMsg.gnss_vel_frame.x = std::stod(row[187]);
  dataScaledMsg.gnss_vel_frame.y = std::stod(row[188]);
  dataScaledMsg.gnss_vel_frame.z = std::stod(row[189]);
  dataScaledMsg.gnss_vel_latency = std::stod(row[190]);
  dataScaledMsg.gnss_stddev_vel.x = std::stod(row[191]);
  dataScaledMsg.gnss_stddev_vel.y = std::stod(row[192]);
  dataScaledMsg.gnss_stddev_vel.z = std::stod(row[193]);
  dataScaledMsg.gnss_log_delay = std::stod(row[194]);
  dataScaledMsg.gnss_diffage = std::stod(row[195]);
  dataScaledMsg.gnss_sats_visible = std::stod(row[196]);
  dataScaledMsg.gnss_time_msec = std::stoul(row[197]);
  dataScaledMsg.gnss_time_week = std::stoul(row[198]);
  dataScaledMsg.gnss_dualant_heading = std::stod(row[199]);
  dataScaledMsg.gnss_dualant_stddev_heading = std::stod(row[200]);
  dataScaledMsg.gnss_dualant_pitch = std::stod(row[201]);
  dataScaledMsg.gnss_dualant_stddev_pitch = std::stod(row[202]);
  dataScaledMsg.gnss_dualant_time_msec = std::stoul(row[203]);

  // ins channels
  // angles
  dataScaledMsg.ins_roll = std::stod(row[204]);
  dataScaledMsg.ins_pitch = std::stod(row[205]);
  dataScaledMsg.ins_yaw = std::stod(row[206]);
  dataScaledMsg.gnss_cog = std::stod(row[207]);

  dataScaledMsg.ins_stddev_roll = std::stod(row[208]);
  dataScaledMsg.ins_stddev_pitch = std::stod(row[209]);
  dataScaledMsg.ins_stddev_yaw = std::stod(row[210]);

  // rel pos
  dataScaledMsg.ins_pos_rel_x = std::stod(row[211]);
  dataScaledMsg.ins_pos_rel_y = std::stod(row[212]);

  // velocities
  // frame
  dataScaledMsg.ins_vel_frame.x = std::stod(row[213]);
  dataScaledMsg.ins_vel_frame.y = std::stod(row[214]);
  dataScaledMsg.ins_vel_frame.z = std::stod(row[215]);

  // horizontal
  dataScaledMsg.ins_vel_hor.x = std::stod(row[216]);
  dataScaledMsg.ins_vel_hor.y = std::stod(row[217]);
  dataScaledMsg.ins_vel_hor.z = std::stod(row[218]);

  // stddev
  dataScaledMsg.ins_stddev_vel.x = std::stod(row[219]);
  dataScaledMsg.ins_stddev_vel.y = std::stod(row[220]);
  dataScaledMsg.ins_stddev_vel.z = std::stod(row[221]);

  // kf 
  dataScaledMsg.kf_lat_stimulated = std::stoul(row[222]);
  dataScaledMsg.kf_long_stimulated = std::stoul(row[223]);
  dataScaledMsg.kf_steady_state = std::stoul(row[224]);
}

}  // end namespace tools
}  // end namespace genesys
RCLCPP_COMPONENTS_REGISTER_NODE(genesys::tools::GSDAServer)
