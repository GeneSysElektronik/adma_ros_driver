#include "adma_ros2_driver/adma_data_server.hpp"
#include <rclcpp_components/register_node_macro.hpp>
#include <boost/asio.hpp>
/**
 * @brief This helper class can "simulate" the ADMA to send its message stream. 
 * can be used to validate the correct parsing of the driver node
 */
namespace genesys
{
        DataServer::DataServer(const rclcpp::NodeOptions &options) : Node("data_server", options)
        {
                 std::string param_address = this->declare_parameter("ip_address", "127.0.0.1");
                _address = boost::asio::ip::address::from_string(param_address);
                _port = this->declare_parameter("port", 3333);
                _frequence = this->declare_parameter("frequence", 20);

                updateLoop();
        }

        DataServer::~DataServer(){

        }

        void DataServer::updateLoop(){
                boost::asio::io_service io_service;
                boost::asio::ip::udp::socket socket(io_service);
                boost::asio::ip::udp::endpoint remote_endpoint = boost::asio::ip::udp::endpoint(_address, _port);
                socket.open(boost::asio::ip::udp::v4());

                boost::system::error_code err;

                // TODO: generate a realistic msg content (or read it from a txt file..)
                std::string msg(856, 0);
                while(rclcpp::ok()){
                        auto sent = socket.send_to(boost::asio::buffer(msg), remote_endpoint, 0, err);
                        RCLCPP_INFO(get_logger(), "sended payload: %ld", sent);
                        std::this_thread::sleep_for(std::chrono::milliseconds(1000 / _frequence));
                }
                socket.close();
        }
} // end namespace genesys
RCLCPP_COMPONENTS_REGISTER_NODE(genesys::DataServer)
