#include "adma_tools_cpp/bag2gsdb_converter.hpp"
#include <rclcpp_components/register_node_macro.hpp>
#include <filesystem>
namespace genesys
{
namespace tools
{

Bag2GSDBConverter::Bag2GSDBConverter(const rclcpp::NodeOptions &options)
: Node("bag2gsdb", options),
admanetMsgCounter_(0),
addonDeltaMsgCounter_(0)
{
        filePath_ = declare_parameter("rosbag_path", "");
        logAddonDelta_ = declare_parameter("log_addon_delta", false);
        std::string fileName;
        if(filePath_.empty()){
                // if no filename was defined, create a new file with timestamp as name to prevent overwriting files..
                auto now = std::chrono::system_clock::now();
                auto in_time_t = std::chrono::system_clock::to_time_t(now);
                std::stringstream datetime;
                datetime << std::put_time(std::localtime(&in_time_t), "%Y-%m-%d-%H-%M-%S");
                fileName = datetime.str();
        }else{
                //otherwise create a file next to the *db3/*mcap file
                std::filesystem::path bag_file_path(filePath_);
                fileName = bag_file_path.parent_path().string() + "/";
        }
        admanetGsdbFile_ = std::ofstream(fileName + "_admanet_data_raw.gsdb");
        RCLCPP_INFO(get_logger(), "Write ADMAnet GSDB to %s", (fileName + "_admanet_data_raw.gsdb").c_str());
        subRawADMAnetData_ = create_subscription<adma_ros_driver_msgs::msg::AdmaDataRaw>(
                "adma/data_raw", 10, std::bind(&Bag2GSDBConverter::rawADMAnetDataCallback,
                this, std::placeholders::_1));

        // optional setup GSDB logging for AddonDelta
        if(logAddonDelta_)
        {
                addonDeltaGsdbFile_ = std::ofstream(fileName + "_addon_delta_data_raw.gsdb");
                RCLCPP_INFO(get_logger(), "Write AddonDelta GSDB to %s", (fileName + "_addon_delta_data_raw.gsdb").c_str());
                subAddonDeltaRawData_ = create_subscription<adma_ros_driver_msgs::msg::Delta1170Raw>(
                        "adma/delta_raw", 10, std::bind(&Bag2GSDBConverter::rawAddonDeltaDataCallback,
                        this, std::placeholders::_1));
        }

}

Bag2GSDBConverter::~Bag2GSDBConverter(){
        RCLCPP_INFO(get_logger(), "closing file, written %ld ADMAnet messages.", admanetMsgCounter_);
        admanetGsdbFile_.close();
        if(logAddonDelta_)
        {
                RCLCPP_INFO(get_logger(), "closing file, written %ld AddonDelta messages.", addonDeltaMsgCounter_);
                addonDeltaGsdbFile_.close();
        }
}

void Bag2GSDBConverter::rawADMAnetDataCallback(adma_ros_driver_msgs::msg::AdmaDataRaw newMsg)
{
        admanetMsgCounter_++;
        admanetGsdbFile_.write((const char*) &newMsg.raw_data[0], newMsg.raw_data.size());
}

void Bag2GSDBConverter::rawAddonDeltaDataCallback(adma_ros_driver_msgs::msg::Delta1170Raw newMsg)
{
        addonDeltaMsgCounter_++;
        addonDeltaGsdbFile_.write((const char*) &newMsg.raw_data[0], newMsg.raw_data.size());
}
}// end namespace tools
}// end namespace genesys
RCLCPP_COMPONENTS_REGISTER_NODE(genesys::tools::Bag2GSDBConverter)
