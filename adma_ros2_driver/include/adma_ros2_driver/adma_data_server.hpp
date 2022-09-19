#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <netdb.h>

namespace genesys
{
        class DataServer : public rclcpp::Node{
                public:
                        explicit DataServer(const rclcpp::NodeOptions &options);
                        virtual ~DataServer();
                private:
                        void updateLoop();

                        int _sendSocketfd;
                        struct sockaddr_in _socketAdress;
                        socklen_t _adressLength;
                        int _port;
                        unsigned short _frequence;
        };
} // end namespace genesys