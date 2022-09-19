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
                _port = this->declare_parameter("port", 1040);
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
                // std::string msg(856, 0);
                unsigned char msg[] = {"4742494e00010001003000100004030300000000000000000000000000000000e875000041444d4120534e3a3330313834000000000000000000000000000000000000000a00000002000000000003010b43000044170000040000004936204511200081000200000000090000000000640000002dffffff38000000d1feffff0f27000096fffffffefffdffffff0000fefffdffffff000019000e00c409000000000300c40900001c000f00d40900001b000800b90900001d000800c409000019000e00c40900001b001000d00900001b000c00c309000019000e00c409000019000e00c409000003000500d40900000200fdffb90900000400fdffc409000000000300c409000001000600d009000001000100c309000000000300c409000000000300c4090000fafff9ff000000000000000000000000fafff9ff00000000000000000000000000000000000000000000000000000000000000000300000000000000040000000000000004000000000000000400000000000000030000000000000003000000000000000300000000000000030000000000000003000000000000000000000000000000000000001027590128004200000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000001800c6ffa08c000000000000000000000000000000000000000000000000000000000000000000000100000000000000bfffffffbbffffffbaffffff01000000e0ffffffddffffff01000000010000003a080400000012007dc5f71cb371b504007c362020167903ccc6f71cb371b504407d36201816790310c5f71c3571b504407b3620c015790310c5f71c3072b504407b3620781679037dc5f71cb371b504007c36202016790381c6f71cb371b504c07c36201816790375c5f71cb371b504c07b3620181679037dc5f71cb371b504007c3620201679037dc5f71cb371b504007c36202016790300000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000002d012d012d0100000101010101010000fefffaff0000fbff00000000000000000000000000000000"};
                while(rclcpp::ok()){
                        auto sent = socket.send_to(boost::asio::buffer(msg), remote_endpoint, 0, err);
                        RCLCPP_INFO(get_logger(), "sended payload: %ld", sent);
                        std::this_thread::sleep_for(std::chrono::milliseconds(1000 / _frequence));
                }
                socket.close();
        }
} // end namespace genesys
RCLCPP_COMPONENTS_REGISTER_NODE(genesys::DataServer)
