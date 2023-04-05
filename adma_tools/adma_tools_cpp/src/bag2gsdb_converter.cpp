#include "adma_tools_cpp/bag2gsdb_converter.hpp"
#include <rclcpp_components/register_node_macro.hpp>

namespace genesys
{
namespace tools
{

Bag2GSDBConverter::Bag2GSDBConverter(const rclcpp::NodeOptions &options)
: Node("bag2gsdb", options),
msgCounter_(0)
{
        filePath_ = declare_parameter("rosbag_path", "");
        if(filePath_.empty()){
                // if no filename was defined, create a new file with timestamp as name to prevent overwriting files..
                auto now = std::chrono::system_clock::now();
                auto in_time_t = std::chrono::system_clock::to_time_t(now);
                std::stringstream datetime;
                datetime << std::put_time(std::localtime(&in_time_t), "%Y-%m-%d-%H-%M-%S");
                filePath_ = datetime.str() + ".gsdb";
                
        }else{
                //otherwise create a file next to the *db3 rosbag file
                filePath_ = filePath_ + "/raw_data.gsdb";
        }
        gdsbFile_ = std::ofstream(filePath_);
        
        subRawData_ = create_subscription<adma_ros_driver_msgs::msg::AdmaDataRaw>(
                "/genesys/adma/data_raw", 10, std::bind(&Bag2GSDBConverter::rawDataCallback,
                this, std::placeholders::_1));
}

Bag2GSDBConverter::~Bag2GSDBConverter(){
        RCLCPP_INFO(get_logger(), "closing file, written %ld messages to %s", msgCounter_, filePath_.c_str());
        gdsbFile_.close();
}

void Bag2GSDBConverter::rawDataCallback(adma_ros_driver_msgs::msg::AdmaDataRaw newMsg)
{
        msgCounter_++;
        gdsbFile_.write((const char*) &newMsg.raw_data[0], newMsg.raw_data.size());
}
}// end namespace tools
}// end namespace genesys
RCLCPP_COMPONENTS_REGISTER_NODE(genesys::tools::Bag2GSDBConverter)