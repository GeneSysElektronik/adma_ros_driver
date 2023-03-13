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
        filePath_ = declare_parameter("rosbag_path", "output.gsdb");
        //create a file next to the *db3 rosbag file
        gdsbFile_ = std::ofstream(filePath_ + "/raw_data.gsdb");
        subRawData_ = create_subscription<adma_ros_driver_msgs::msg::AdmaDataRaw>(
                "/genesys/adma/data_recorded", 10, std::bind(&Bag2GSDBConverter::rawDataCallback,
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