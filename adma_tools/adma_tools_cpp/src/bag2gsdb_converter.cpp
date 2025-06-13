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

#include <filesystem>
#include <rclcpp_components/register_node_macro.hpp>

#include "adma_tools_cpp/bag2gsdb_converter.hpp"

namespace genesys
{
namespace tools
{

Bag2GSDBConverter::Bag2GSDBConverter(const rclcpp::NodeOptions & options)
: Node("bag2gsdb", options),
  admanetMsgCounter_(0),
  addonDeltaMsgCounter_(0)
{
  filePath_ = declare_parameter("rosbag_path", "");
  logAddonDelta_ = declare_parameter("log_addon_delta", false);
  std::string fileName;
  if (filePath_.empty()) {
    // if no filename is given, create new file with timestamp as name to prevent overwriting files
    auto now = std::chrono::system_clock::now();
    auto in_time_t = std::chrono::system_clock::to_time_t(now);
    std::stringstream datetime;
    datetime << std::put_time(std::localtime(&in_time_t), "%Y-%m-%d-%H-%M-%S");
    fileName = datetime.str();
  } else {
    // otherwise create a file next to the *db3/*mcap file
    std::filesystem::path bag_file_path(filePath_);
    fileName = bag_file_path.parent_path().string() + "/";
  }
  admanetGsdbFile_ = std::ofstream(fileName + "_admanet_data_raw.gsdb");
  RCLCPP_INFO(
    get_logger(), "Write ADMAnet GSDB to %s",
    (fileName + "_admanet_data_raw.gsdb").c_str());
  subRawADMAnetData_ = create_subscription<adma_ros_driver_msgs::msg::AdmaDataRaw>(
    "adma/data_raw", 10, std::bind(
      &Bag2GSDBConverter::rawADMAnetDataCallback,
      this, std::placeholders::_1));

  // optional setup GSDB logging for AddonDelta
  if (logAddonDelta_) {
    addonDeltaGsdbFile_ = std::ofstream(fileName + "_addon_delta_data_raw.gsdb");
    RCLCPP_INFO(
      get_logger(), "Write AddonDelta GSDB to %s",
      (fileName + "_addon_delta_data_raw.gsdb").c_str());
    subAddonDeltaRawData_ = create_subscription<adma_ros_driver_msgs::msg::Delta1170Raw>(
      "adma/delta_raw", 10, std::bind(
        &Bag2GSDBConverter::rawAddonDeltaDataCallback,
        this, std::placeholders::_1));
  }
}

Bag2GSDBConverter::~Bag2GSDBConverter()
{
  RCLCPP_INFO(get_logger(), "closing file, written %ld ADMAnet messages.", admanetMsgCounter_);
  admanetGsdbFile_.close();
  if (logAddonDelta_) {
    RCLCPP_INFO(
      get_logger(), "closing file, written %ld AddonDelta messages.", addonDeltaMsgCounter_);
    addonDeltaGsdbFile_.close();
  }
}

void Bag2GSDBConverter::rawADMAnetDataCallback(adma_ros_driver_msgs::msg::AdmaDataRaw newMsg)
{
  admanetMsgCounter_++;
  admanetGsdbFile_.write((const char *) &newMsg.raw_data[0], newMsg.raw_data.size());
}

void Bag2GSDBConverter::rawAddonDeltaDataCallback(adma_ros_driver_msgs::msg::Delta1170Raw newMsg)
{
  addonDeltaMsgCounter_++;
  addonDeltaGsdbFile_.write((const char *) &newMsg.raw_data[0], newMsg.raw_data.size());
}
}  // end namespace tools
}  // end namespace genesys
RCLCPP_COMPONENTS_REGISTER_NODE(genesys::tools::Bag2GSDBConverter)
