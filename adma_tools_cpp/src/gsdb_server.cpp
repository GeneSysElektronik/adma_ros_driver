#include <netdb.h>
#include <fstream>
#include <memory>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <iostream>
#include <ros/ros.h>

class GSDBServer
{
public:
        GSDBServer(ros::NodeHandle* _nh);
        //~GSDBServer();

private:
        void updateLoop();

        int send_socket_fd_;
        struct sockaddr_in socket_address_;
        socklen_t address_length_;
        int port_;
        int frequency_;
        std::string protocol_version_;

        std::string gsdbFilePath_;
        std::fstream gsdbFile_;
        unsigned long msgCounter_;
        unsigned long protocolLength_;
};

GSDBServer::GSDBServer(ros::NodeHandle* n)
{

        // read ros parameters
        ros::param::get("/gsdbserver/frequency", frequency_);
        std::string ip_address;
        ros::param::get("/gsdbserver/ip_address", ip_address);
        ros::param::get("/gsdbserver/port", port_);
        ros::param::get("/gsdbserver/protocol_version", protocol_version_);
        ros::param::get("/gsdbserver/gsdb_file", gsdbFilePath_);

        gsdbFile_ = std::fstream(gsdbFilePath_);
        if(gsdbFile_)
        {
                ROS_INFO("Loaded GSDB-File: %s", gsdbFilePath_.c_str());
        }else
        {
                ROS_WARN("Desired GSDB-File not found: %s", gsdbFilePath_.c_str());
        }

        ROS_INFO("Working with: %s, publishing data at %d Hz", protocol_version_.c_str(), frequency_);
        if (protocol_version_ == "v3.2") {
                protocolLength_ = 768;
        }else{
                protocolLength_ = 856;
        }

        // setup socket for sending data
        send_socket_fd_ = socket(AF_INET, SOCK_DGRAM | SOCK_CLOEXEC, IPPROTO_UDP);
        address_length_ = sizeof(socket_address_);
        memset((char *)&socket_address_, 0, address_length_);
        socket_address_.sin_family = AF_INET;
        socket_address_.sin_port = htons(port_);
        inet_aton(ip_address.c_str(), &(socket_address_.sin_addr));
        
        msgCounter_ = 0;

        updateLoop();
}

// GSDBServer::~GSDBServer() 
// { 
//         ::shutdown(send_socket_fd_, SHUT_RDWR); 
//         ROS_INFO("GSDB file streaming done.. Read %ld messages from file", msgCounter_);
// }

void GSDBServer::updateLoop()
{
        ros::Rate rate(frequency_);
        char buffer[856];
        while (ros::ok()) {
                if(gsdbFile_.read(buffer, protocolLength_)){
                        ::sendto(send_socket_fd_, (void *)(&buffer), protocolLength_, 0, (struct sockaddr *)&socket_address_, address_length_);
                }else{
                        ROS_INFO("GSDB file streaming done.. Read %ld messages from file", msgCounter_);
                        ::shutdown(send_socket_fd_, SHUT_RDWR); 
                        ros::shutdown();
                }
                rate.sleep();
                msgCounter_++;
                gsdbFile_.seekg(msgCounter_ * protocolLength_);
        }
}

int main(int argc, char **argv)
{
        ros::init(argc, argv, "gsdbserver");
        ros::NodeHandle nh;
        GSDBServer server = GSDBServer(&nh);
        ros::spinOnce();

        return 0;
}
