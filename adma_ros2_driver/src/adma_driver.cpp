#include "adma_ros2_driver/adma_driver.hpp"
#include "adma_ros2_driver/parser/adma_parse_deprecated.hpp"
#include "adma_ros2_driver/parser/parser_utils.hpp"
#include <rclcpp_components/register_node_macro.hpp>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/select.h>
#include <arpa/inet.h>

namespace genesys
{
        ADMADriver::ADMADriver(const rclcpp::NodeOptions &options) : Node("adma_driver", options),
        _rcvSockfd(-1),
        _rcvAddrInfo(NULL),
        _admaAddr(),
        _admaAddrLen(4),
        _admaPort(0)
        {
                std::string param_address = this->declare_parameter("destination_ip", "0.0.0.0");
                _admaPort = this->declare_parameter("destination_port", 1040);
                initializeUDP(param_address);

                _performance_check = this->declare_parameter("use_performance_check", false);
                _gnss_frame = this->declare_parameter("gnss_frame", "gnss_link");
                _imu_frame = this->declare_parameter("imu_frame", "imu_link");
                // define protocol specific stuff
                _protocolversion = this->declare_parameter("protocol_version", "v3.3.3");
                RCLCPP_INFO(get_logger(), "Working with: %s", _protocolversion.c_str());
                if(_protocolversion == "v3.2")
                {
                        _len =  768;
                        _pub_adma_data = this->create_publisher<adma_msgs::msg::AdmaData>("adma/data", 1);
                }else if (_protocolversion == "v3.3.3")
                {
                        _len = 856;
                        _pub_adma_data = this->create_publisher<adma_msgs::msg::AdmaData>("adma/data", 1);
                }else if (_protocolversion == "v3.3.4")
                {
                        _len = 856;
                        _pub_adma_data_raw = this->create_publisher<std_msgs::msg::String>("adma/data_raw", 1);
                        _pub_adma_data_scaled = this->create_publisher<adma_msgs::msg::AdmaDataScaled>("adma/data_scaled", 1);
                }
                _parser = new ADMA2ROSParser(_protocolversion);
                
                _pub_navsat_fix = this->create_publisher<sensor_msgs::msg::NavSatFix>("adma/fix", 1);
                _pub_imu = this->create_publisher<sensor_msgs::msg::Imu>("adma/imu", 1);
                _pub_heading = this->create_publisher<std_msgs::msg::Float64>("adma/heading", 1);
                _pub_velocity = this->create_publisher<std_msgs::msg::Float64>("adma/velocity", 1);

                updateLoop();
        }

        ADMADriver::~ADMADriver(){
                freeaddrinfo(_rcvAddrInfo);
                ::shutdown(_rcvSockfd, SHUT_RDWR);
                _rcvSockfd = -1;
        }

        void ADMADriver::initializeUDP(std::string admaAdress)
        {
                struct addrinfo hints;
                memset(&hints, 0, sizeof(hints));
                hints.ai_family = AF_UNSPEC;
                hints.ai_socktype = SOCK_DGRAM;
                hints.ai_protocol = IPPROTO_UDP;
                std::string rcvPortStr = std::to_string(_admaPort);


                _admaAddrLen= sizeof(_admaAddr);
                memset((char *) &_admaAddr, 0, _admaAddrLen);
                _admaAddr.sin_family = AF_INET;
                _admaAddr.sin_port = htons(_admaPort);
                inet_aton(admaAdress.c_str(), &(_admaAddr.sin_addr));

                int r = getaddrinfo(admaAdress.c_str(), rcvPortStr.c_str(), &hints, &_rcvAddrInfo);
                if (r != 0 || _rcvAddrInfo == NULL) {
                        RCLCPP_FATAL(get_logger(), "Invalid port for UDP socket: \"%s:%s\"", admaAdress.c_str(), rcvPortStr.c_str());
                        throw rclcpp::exceptions::InvalidParameterValueException("Invalid port for UDP socket: \"" + admaAdress + ":" + rcvPortStr + "\"");
                }
                _rcvSockfd = socket(_rcvAddrInfo->ai_family, SOCK_DGRAM | SOCK_CLOEXEC, IPPROTO_UDP);
                if (_rcvSockfd == -1) {
                        freeaddrinfo(_rcvAddrInfo);
                        RCLCPP_FATAL(get_logger(), "Could not create UDP socket for: \"%s:%s", admaAdress.c_str(), rcvPortStr.c_str());
                        throw rclcpp::exceptions::InvalidParameterValueException("Could not create UDP socket for: \"" + admaAdress + ":" + rcvPortStr + "\"");
                }
                r = bind(_rcvSockfd, _rcvAddrInfo->ai_addr, _rcvAddrInfo->ai_addrlen);
                if (r != 0) {
                        freeaddrinfo(_rcvAddrInfo);
                        ::shutdown(_rcvSockfd, SHUT_RDWR);
                        RCLCPP_FATAL(get_logger(), "Could not bind UDP socket with: \"%s:%s", admaAdress.c_str(), rcvPortStr.c_str());
                        throw rclcpp::exceptions::InvalidParameterValueException("Could not bind UDP socket with: \"" + admaAdress + ":" + rcvPortStr + "\"");
                }
        }

        void ADMADriver::updateLoop()
        {
                fd_set s;
                struct timeval timeout;
                struct sockaddr srcAddr;
                socklen_t srcAddrLen;

                std::array<char, 856> recv_buf;

                while (rclcpp::ok())
                {
                        // check if new data is available
                        FD_ZERO(&s);
                        FD_SET(_rcvSockfd, &s);
                        timeout.tv_sec = 1;
                        timeout.tv_usec = 0;
                        int ret = select(_rcvSockfd + 1, &s, NULL, NULL, &timeout);
                        if (ret == 0) {
                                // reached timeout
                                RCLCPP_INFO(get_logger(), "Waiting for ADMA data...");
                                continue;
                        } else if (ret == -1) {
                                // error
                                RCLCPP_WARN(get_logger(), "Select-error: %s", strerror(errno));
                                continue;
                        }

                        ret = ::recv(_rcvSockfd, (void *) (&recv_buf), _len, 0);
                        if (ret < 0) {
                                RCLCPP_WARN(get_logger(), "Receive-error: %s", strerror(errno));
                                continue;
                        } else if (ret != _len) {
                                RCLCPP_WARN(get_logger(), "Invalid ADMA message size: %d instead of %ld", ret, _len);
                                continue;
                        }

                        builtin_interfaces::msg::Time curTimestamp = this->get_clock()->now();
                                                
                        // prepare several ros msgs
                        sensor_msgs::msg::NavSatFix message_fix;
                        message_fix.header.stamp = curTimestamp;
                        message_fix.header.frame_id = "adma";
                        std_msgs::msg::Float64 message_heading;
                        std_msgs::msg::Float64 message_velocity;
                        sensor_msgs::msg::Imu message_imu;
                        message_imu.header.frame_id = _imu_frame;
                        message_fix.header.stamp = curTimestamp;
                        float weektime;
                        uint32_t instimemsec;
                        std::string recvData(recv_buf.begin(), recv_buf.end());

                        // read Adma msg from UDP data packet
                        if (_protocolversion == "v3.2" || _protocolversion == "v3.3.3")
                        {
                                adma_msgs::msg::AdmaData admaData_rosMsg;
                                _parser->mapAdmaMessageToROS(admaData_rosMsg, recv_buf);
                                
                                admaData_rosMsg.timemsec = curTimestamp.sec * 1000;
                                admaData_rosMsg.timensec = curTimestamp.nanosec;
                                
                                // read NavSatFix out of AdmaData
                                _parser->extractNavSatFix(admaData_rosMsg, message_fix);

                                // read heading and velocity
                                message_heading.data = admaData_rosMsg.finsyaw;
                                message_velocity.data = std::sqrt(std::pow(admaData_rosMsg.fgpsvelframex, 2) + std::pow(admaData_rosMsg.fgpsvelframey, 2)) * 3.6;

                                // read IMU
                                _parser->extractIMU(admaData_rosMsg, message_imu);
                                _pub_adma_data->publish(admaData_rosMsg);
                                weektime = admaData_rosMsg.instimeweek;
                                instimemsec = admaData_rosMsg.instimemsec;
                        }else if (_protocolversion == "v3.3.4")
                        {
                                AdmaDataV334 dataStruct;
                                memcpy(&dataStruct , &recv_buf, sizeof(dataStruct));
                                adma_msgs::msg::AdmaDataScaled admaDataScaledMsg;
                                _parser->parseV334(admaDataScaledMsg, dataStruct);
                                admaDataScaledMsg.time_msec = curTimestamp.sec * 1000;
                                admaDataScaledMsg.time_nsec = curTimestamp.nanosec;

                                _parser->extractNavSatFix(admaDataScaledMsg, message_fix);
                                _parser->extractIMU(admaDataScaledMsg, message_imu);

                                // read heading and velocity
                                message_heading.data = admaDataScaledMsg.ins_yaw;
                                message_velocity.data = std::sqrt(std::pow(admaDataScaledMsg.gnss_vel_frame.x, 2) + std::pow(admaDataScaledMsg.gnss_vel_frame.y, 2)) * 3.6;

                                _pub_adma_data_scaled->publish(admaDataScaledMsg);

                                //publish raw data as HEX string
                                std::stringstream ss;
                                for(int i=0; i<_len; ++i)
                                {
                                        ss << std::hex << (int)recv_buf[i];
                                }
                                std_msgs::msg::String rawDataMsg;
                                rawDataMsg.data = ss.str();
                                _pub_adma_data_raw->publish(rawDataMsg);

                                weektime = admaDataScaledMsg.ins_time_week;
                                instimemsec = admaDataScaledMsg.ins_time_msec;
                        }
                        

                        // publish the messages
                        _pub_navsat_fix->publish(message_fix);
                        _pub_heading->publish(message_heading);
                        _pub_velocity->publish(message_velocity);
                        _pub_imu->publish(message_imu);
                        
                        double grab_time = this->get_clock()->now().seconds();

                        if (_performance_check)
                        {
                                RCLCPP_INFO(get_logger(), "%f ", ((grab_time * 1000) - (instimemsec + 1592697600000)));
                        }
                }
        }
} // namespace genesys

RCLCPP_COMPONENTS_REGISTER_NODE(genesys::ADMADriver)