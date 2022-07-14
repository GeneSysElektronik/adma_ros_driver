#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <std_msgs/msg/float64.hpp>
#include <boost/asio.hpp>
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
                        void updateLoop();

                        /** \brief IP address to which ADMA broadcasts */
                        boost::asio::ip::address _address;
                        /** \brief port to which ADMA broadcasts */
                        unsigned short _port;
                        /** \brief Length of the stream */
                        size_t _len = 0;
                        /** \brief Check the timings */
                        bool _performance_check = true;

                        // publisher
                        rclcpp::Publisher<adma_msgs::msg::AdmaData>::SharedPtr _pub_adma_data;
                        rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr _pub_navsat_fix;
                        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr _pub_heading;
                        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr _pub_velocity;
        };
}