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
  adma_ros_driver_msgs::msg::AdmaDataScaled dataScaledMsg;
  adma_ros_driver_msgs::msg::AdmaStatus stateMsg;
  std_msgs::msg::Float64 velMsg;
  std_msgs::msg::Float64 headingMsg;
  sensor_msgs::msg::Imu imuMsg;
  sensor_msgs::msg::NavSatFix navsatfixMsg;
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
      // extract separate msgs
      parser_->extractNavSatFix(dataScaledMsg, navsatfixMsg);
      parser_->extractIMU(dataScaledMsg, imuMsg);
      // read heading and velocity
      headingMsg.data = dataScaledMsg.ins_yaw;
      velMsg.data = std::sqrt(
                      std::pow(dataScaledMsg.ins_vel_frame.x, 2) +
                      std::pow(dataScaledMsg.ins_vel_frame.y, 2)) *
                    3.6;
      //TODO: handle status/error

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
  //TODO: add all data fields
  dataScaledMsg.ins_lat_abs = std::stod(row[9]);
  dataScaledMsg.ins_long_abs = std::stod(row[10]);
}

}  // end namespace tools
}  // end namespace genesys
RCLCPP_COMPONENTS_REGISTER_NODE(genesys::tools::GSDAServer)
