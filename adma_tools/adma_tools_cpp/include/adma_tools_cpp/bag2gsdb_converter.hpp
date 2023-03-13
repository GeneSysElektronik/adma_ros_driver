#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <fstream>
#include <adma_ros_driver_msgs/msg/adma_data_raw.hpp>

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
                void rawDataCallback(adma_ros_driver_msgs::msg::AdmaDataRaw newMsg);

                rclcpp::Subscription<adma_ros_driver_msgs::msg::AdmaDataRaw>::SharedPtr subRawData_;
                std::string filePath_;
                std::ofstream gdsbFile_;
                unsigned long msgCounter_;
};

}// end namespace tools
}// end namespace genesys