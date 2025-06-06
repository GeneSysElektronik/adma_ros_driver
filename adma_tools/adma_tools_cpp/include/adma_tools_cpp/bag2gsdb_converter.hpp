#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <fstream>
#include <adma_ros_driver_msgs/msg/adma_data_raw.hpp>
#include <adma_ros_driver_msgs/msg/delta1170_raw.hpp>

namespace genesys
{
namespace tools
{

class Bag2GSDBConverter : public rclcpp::Node
{
        public:
                explicit Bag2GSDBConverter(const rclcpp::NodeOptions &options);
                virtual ~Bag2GSDBConverter();
        private:
                void rawADMAnetDataCallback(adma_ros_driver_msgs::msg::AdmaDataRaw newMsg);
                void rawAddonDeltaDataCallback(adma_ros_driver_msgs::msg::Delta1170Raw newMsg);

                rclcpp::Subscription<adma_ros_driver_msgs::msg::AdmaDataRaw>::SharedPtr subRawADMAnetData_;
                rclcpp::Subscription<adma_ros_driver_msgs::msg::Delta1170Raw>::SharedPtr subAddonDeltaRawData_;

                std::string filePath_;
                std::ofstream admanetGsdbFile_;
                unsigned long admanetMsgCounter_;
                bool logAddonDelta_;
                std::ofstream addonDeltaGsdbFile_;
                unsigned long addonDeltaMsgCounter_;
};

}// end namespace tools
}// end namespace genesys
