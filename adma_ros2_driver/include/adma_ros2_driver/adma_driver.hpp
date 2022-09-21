#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/float64.hpp>
#include <netdb.h>
#include "adma_msgs/msg/adma_data.hpp"

#pragma once

namespace genesys
{
        class ADMADriver : public rclcpp::Node
        {
                public:
                        explicit ADMADriver(const rclcpp::NodeOptions &options);
                        virtual ~ADMADriver();
                
                private:
                        void initializeUDP(std::string admaAdress);
                        void updateLoop();

                        // Socket file descriptor for receiving from adma
                        int _rcvSockfd; 
                        // Address info for receiving from adma
                        struct addrinfo* _rcvAddrInfo;
                        // adma socket address
                        struct sockaddr_in _admaAddr;
                        //Adma  socket address length
                        socklen_t _admaAddrLen;
                        int _admaPort;
                        size_t _len = 0;
                        /** \brief Check the timings */
                        bool _performance_check = true;
                        std::string _protocolversion;

                        // publisher
                        rclcpp::Publisher<adma_msgs::msg::AdmaData>::SharedPtr _pub_adma_data;
                        rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr _pub_navsat_fix;
                        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr _pub_imu;
                        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr _pub_heading;
                        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr _pub_velocity;

                        std::string _gnss_frame;
                        std::string _imu_frame;
        };
}