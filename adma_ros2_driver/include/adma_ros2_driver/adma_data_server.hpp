#include <rclcpp/rclcpp.hpp>
#include <boost/asio.hpp>

namespace genesys
{
        class DataServer : public rclcpp::Node{
                public:
                        explicit DataServer(const rclcpp::NodeOptions &options);
                        virtual ~DataServer();
                private:
                        void updateLoop();

                        /** \brief IP address to boradcast msgs */
                        boost::asio::ip::address _address;
                        /** \brief port to broadcast msgs */
                        unsigned short _port;
                        /** \brief frequence to publish the data */
                        unsigned short _frequence;
        };
} // end namespace genesys