#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <adma_msgs/Adma.h>
#include <adma_msgs/AdmaDataScaled.h>
#include <adma_msgs/AdmaDataRaw.h>
#include <adma_msgs/AdmaStatus.h>
#include <netdb.h>
#include <arpa/inet.h>
#include "../include/adma_ros_driver/parser/adma2ros_parser.hpp"

// definition of some constants
#define MODE_DEFAULT "default"
#define MODE_RECORD "record"
#define MODE_REPLAY "replay"

class ADMADriver {

        public:
                ADMADriver(ros::NodeHandle* _nh);

        protected:
               void initializeUDP();
                void updateLoop();
                void parseData(std::array<char, 856> recv_buf);
                void recordedDataCB(adma_msgs::AdmaDataRaw dataMsg);

        private:
                ros::Publisher _pubAdmaData;
                ros::Publisher _pubAdmaDataScaled;
                ros::Publisher _pubAdmaDataRaw;
                ros::Publisher _pubAdmaDataRecorded;
                ros::Publisher _pubAdmaStatus;
                ros::Publisher _pubHeading;
                ros::Publisher _pubVelocity;
                ros::Publisher _pubNavSatFix;
                ros::Publisher _pubImu;
                ros::Subscriber _subDataRaw;
                // ROS parameters
                std::string _param_adma_ip;
                int _param_adma_port;
                bool _use_performance_check;
                std::string _protocol_version;
                std::string _mode;
                std::string _frame_id_navsatfix;
                std::string _frame_id_imu;
                std::string _frame_id_adma;
                std::string _frame_id_adma_status;
                std::string _frame_id_data_raw;

                //network attributes
                // Socket file descriptor for receiving from adma
                int _rcvSockfd; 
                // Address info for receiving from adma
                struct addrinfo* _rcvAddrInfo;
                // adma socket address
                struct sockaddr_in _admaAddr;
                //Adma  socket address length
                socklen_t _admaAddrLen;
                size_t _len = 856;

                ADMA2ROSParser* _parser;

                // ROS sequence ID for header
                long _seq = 0;

};

ADMADriver::ADMADriver(ros::NodeHandle* n)
{
        // create ROS subscriber and publisher
        _pubAdmaData = n->advertise<adma_msgs::Adma>("adma/data", 10);
        _pubAdmaDataScaled = n->advertise<adma_msgs::AdmaDataScaled>("adma/data_scaled", 10);
        _pubAdmaDataRaw = n->advertise<adma_msgs::AdmaDataRaw>("adma/data_raw", 10);
        _pubAdmaStatus = n->advertise<adma_msgs::AdmaStatus>("adma/status", 10);
        _pubHeading = n->advertise<std_msgs::Float64>("adma/heading", 10);
        _pubVelocity = n->advertise<std_msgs::Float64>("adma/velocity", 10);
        _pubNavSatFix = n->advertise<sensor_msgs::NavSatFix>("adma/fix", 10);
        _pubImu = n->advertise<sensor_msgs::Imu>("adma/imu", 10);

        //define ROS parameters
        
        ros::param::get("/adma_driver/destination_ip", _param_adma_ip);
        ros::param::get("/adma_driver/destination_port", _param_adma_port);
        ros::param::get("/adma_driver/use_performance_check", _use_performance_check);
        ros::param::get("/adma_driver/protocol_version", _protocol_version);
        ros::param::get("/adma_driver/mode", _mode);
        ros::param::get("/adma_driver/frame_id_navsatfix", _frame_id_navsatfix);
        ros::param::get("/adma_driver/frame_id_imu", _frame_id_imu);
        ros::param::get("/adma_driver/frame_id_adma", _frame_id_adma);
        ros::param::get("/adma_driver/frame_id_adma_status", _frame_id_adma_status);
        ros::param::get("/adma_driver/frame_id_data_raw", _frame_id_data_raw);
        
        ROS_INFO("Try finding ADMA at: %s:%d", _param_adma_ip.c_str(), _param_adma_port);
        ROS_INFO("Using ADMA protocol version: %s", _protocol_version.c_str());

        _parser = new ADMA2ROSParser();

        if(_mode == MODE_RECORD)
        {
                // when recording we receive data from UDP and publish it to the recording topic
                ROS_INFO(" publish recording topic..");
                _pubAdmaDataRecorded = n->advertise<adma_msgs::AdmaDataRaw>("adma/data_recorded", 10);
                initializeUDP();
                updateLoop();
        }
        else if(_mode == MODE_REPLAY)
        {
                // if we use recorded data, create desired subscriber and no UDP connection is required
                _subDataRaw = n->subscribe("adma/data_recorded", 10, &ADMADriver::recordedDataCB, this);
        }else if(_mode == MODE_DEFAULT){
                initializeUDP();
                updateLoop();
        }
}

void ADMADriver::initializeUDP()
{
        struct addrinfo hints;
        memset(&hints, 0, sizeof(hints));
        hints.ai_family = AF_UNSPEC;
        hints.ai_socktype = SOCK_DGRAM;
        hints.ai_protocol = IPPROTO_UDP;
        std::string rcvPortStr = std::to_string(_param_adma_port);


        _admaAddrLen= sizeof(_admaAddr);
        memset((char *) &_admaAddr, 0, _admaAddrLen);
        _admaAddr.sin_family = AF_INET;
        _admaAddr.sin_port = htons(_param_adma_port);
        inet_aton(_param_adma_ip.c_str(), &(_admaAddr.sin_addr));

        int r = getaddrinfo(_param_adma_ip.c_str(), rcvPortStr.c_str(), &hints, &_rcvAddrInfo);
        if (r != 0 || _rcvAddrInfo == NULL) {
                ROS_FATAL("Invalid port for UDP socket: \"%s:%s\"", _param_adma_ip.c_str(), rcvPortStr.c_str());
                throw ros::InvalidParameterException("Invalid port for UDP socket: \"" + _param_adma_ip + ":" + rcvPortStr + "\"");
        }
        _rcvSockfd = socket(_rcvAddrInfo->ai_family, SOCK_DGRAM | SOCK_CLOEXEC, IPPROTO_UDP);
        if (_rcvSockfd == -1) {
                freeaddrinfo(_rcvAddrInfo);
                ROS_FATAL("Could not create UDP socket for: \"%s:%s", _param_adma_ip.c_str(), rcvPortStr.c_str());
                throw ros::InvalidParameterException("Could not create UDP socket for: \"" + _param_adma_ip + ":" + rcvPortStr + "\"");
        }
        r = bind(_rcvSockfd, _rcvAddrInfo->ai_addr, _rcvAddrInfo->ai_addrlen);
        if (r != 0) {
                freeaddrinfo(_rcvAddrInfo);
                ::shutdown(_rcvSockfd, SHUT_RDWR);
                ROS_FATAL("Could not bind UDP socket with: \"%s:%s", _param_adma_ip.c_str(), rcvPortStr.c_str());
                throw ros::InvalidParameterException("Could not bind UDP socket with: \"" + _param_adma_ip + ":" + rcvPortStr + "\"");
        }
}

void ADMADriver::updateLoop()
{
        while(ros::ok())
        {
                fd_set s;
                struct timeval timeout;
                struct sockaddr srcAddr;
                socklen_t srcAddrLen;

                std::array<char, 856> recv_buf;

                // check if new data is available
                FD_ZERO(&s);
                FD_SET(_rcvSockfd, &s);
                timeout.tv_sec = 1;
                timeout.tv_usec = 0;
                int ret = select(_rcvSockfd + 1, &s, NULL, NULL, &timeout);
                if (ret == 0) {
                        // reached timeout
                        ROS_INFO("Waiting for ADMA data...");
                        continue;
                } else if (ret == -1) {
                        // error
                        ROS_WARN("Select-error: %s", strerror(errno));
                        continue;
                }

                ret = ::recv(_rcvSockfd, (void *) (&recv_buf), _len, 0);
                if (ret < 0) {
                        ROS_WARN("Receive-error: %s", strerror(errno));
                        continue;
                } else if (ret != _len) {
                        ROS_WARN("Invalid ADMA message size: %d instead of %ld", ret, _len);
                        continue;
                }

                parseData(recv_buf);
        }
}

void ADMADriver::parseData(std::array<char, 856> recv_buf)
{
        ros::Time curTimestamp = ros::Time::now();
        // prepare several ros msgs
        std_msgs::Float64 message_heading;
        std_msgs::Float64 message_velocity;
        sensor_msgs::NavSatFix message_fix;
        message_fix.header.frame_id = _frame_id_navsatfix;
        message_fix.header.seq = _seq;
        message_fix.header.stamp.sec = curTimestamp.toSec();
        message_fix.header.stamp.nsec = curTimestamp.toNSec();
        sensor_msgs::Imu message_imu;
        message_imu.header.frame_id = _frame_id_imu;
        message_imu.header.seq = _seq;
        message_imu.header.stamp.sec = curTimestamp.toSec();
        message_imu.header.stamp.nsec = curTimestamp.toNSec();
        float weektime;
        uint32_t instimemsec;

        AdmaDataV334 dataStruct;
        memcpy(&dataStruct , &recv_buf, sizeof(dataStruct));
        adma_msgs::AdmaDataScaled admaDataScaledMsg;
        admaDataScaledMsg.header.frame_id = _frame_id_adma;
        admaDataScaledMsg.header.seq = _seq;
        admaDataScaledMsg.header.stamp.sec = curTimestamp.toSec();
        admaDataScaledMsg.header.stamp.nsec = curTimestamp.toNSec();
        _parser->parseV334(admaDataScaledMsg, dataStruct);
        admaDataScaledMsg.time_msec = curTimestamp.toSec() * 1000;
        admaDataScaledMsg.time_nsec = curTimestamp.toNSec();

        _parser->extractNavSatFix(admaDataScaledMsg, message_fix);
        _parser->extractIMU(admaDataScaledMsg, message_imu);

        // read heading and velocity
        message_heading.data = admaDataScaledMsg.ins_yaw;
        message_velocity.data = std::sqrt(std::pow(admaDataScaledMsg.gnss_vel_frame.x, 2) + std::pow(admaDataScaledMsg.gnss_vel_frame.y, 2)) * 3.6;

        // publish raw data as byte array
        adma_msgs::AdmaDataRaw rawDataMsg;
        rawDataMsg.size = _len;
        rawDataMsg.header.frame_id = _frame_id_data_raw;
        rawDataMsg.header.seq = _seq;
        rawDataMsg.header.stamp.sec = curTimestamp.toSec();
        rawDataMsg.header.stamp.nsec = curTimestamp.toNSec();

        //fill raw data byte array
        for(int i=0; i<_len; ++i)
        {
                rawDataMsg.raw_data.push_back(recv_buf[i]);
        }
        
        weektime = admaDataScaledMsg.ins_time_week;
        instimemsec = admaDataScaledMsg.ins_time_msec;

        // parse status msg
        adma_msgs::AdmaStatus statusMsg;
        statusMsg.header.frame_id = _frame_id_adma_status;
        statusMsg.header.seq = _seq;
        statusMsg.header.stamp.sec = curTimestamp.toSec();
        statusMsg.header.stamp.nsec = curTimestamp.toNSec();
        _parser->parseV334Status(statusMsg, dataStruct);

        // publish adma custom messages (version specific)
        _pubAdmaDataRaw.publish(rawDataMsg);
        _pubAdmaDataScaled.publish(admaDataScaledMsg);
        _pubAdmaStatus.publish(statusMsg);
        if(_mode == MODE_RECORD){
                _pubAdmaDataRecorded.publish(rawDataMsg);
        }

        // publish ros standard messages
        _pubNavSatFix.publish(message_fix);
        _pubHeading.publish(message_heading);
        _pubVelocity.publish(message_velocity);
        _pubImu.publish(message_imu);

        _seq++;
}

void ADMADriver::recordedDataCB(adma_msgs::AdmaDataRaw dataMsg)
{
        std::array<char, 856> recv_buf;
        for (size_t i = 0; i < dataMsg.size; i++)
        {
                recv_buf[i] = dataMsg.raw_data[i];
        }
        parseData(recv_buf);

}

int main(int argc, char **argv)
{
        ros::init(argc, argv, "adma_driver");
        ros::NodeHandle nh;
        ADMADriver driver = ADMADriver(&nh);
        ros::spin();

        return 0;
}