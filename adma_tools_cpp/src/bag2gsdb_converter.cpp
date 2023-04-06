#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <adma_ros_driver_msgs/AdmaDataRaw.h>

class Bag2GSDBConverter
{
        public:
                Bag2GSDBConverter(ros::NodeHandle* _nh);
                // virtual ~Bag2GSDBConverter();
        private:
                void rawDataCallback(adma_ros_driver_msgs::AdmaDataRaw newMsg);

                ros::Subscriber subRawData_;
                std::string bagfilePath_;
                std::string logPath_;
                std::string filePath_;
                std::ofstream gdsbFile_;
                unsigned long msgCounter_;
};

Bag2GSDBConverter::Bag2GSDBConverter(ros::NodeHandle* n)
{
        ros::param::get("/bag2gsdb/rosbag_path", bagfilePath_);
        ros::param::get("/bag2gsdb/log_path", logPath_);
        if(bagfilePath_.empty()){
                // if no filename was defined, create a new file with timestamp as name to prevent overwriting files..
                auto now = std::chrono::system_clock::now();
                auto in_time_t = std::chrono::system_clock::to_time_t(now);
                std::stringstream datetime;
                datetime << std::put_time(std::localtime(&in_time_t), "%Y-%m-%d-%H-%M-%S");
                filePath_ = logPath_ + "/" + datetime.str() + ".gsdb";
                
        }else{
                //otherwise create a file next to the *db3 rosbag file
                bagfilePath_.replace(bagfilePath_.find(".bag"), sizeof(".bag") - 1, ".gsdb");
                filePath_ = bagfilePath_;
        }
        ROS_INFO("writing gsdb log to: %s", filePath_.c_str());
        gdsbFile_ = std::ofstream(filePath_);
        subRawData_ = n->subscribe("adma/data_recorded", 10, &Bag2GSDBConverter::rawDataCallback, this);
        msgCounter_ = 0;
}

// Bag2GSDBConverter::~Bag2GSDBConverter(){
//         ROS_INFO("closing file, written %ld messages to %s", msgCounter_, filePath_.c_str());
//         gdsbFile_.close();
// }

void Bag2GSDBConverter::rawDataCallback(adma_ros_driver_msgs::AdmaDataRaw newMsg)
{
        msgCounter_++;
        gdsbFile_.write((const char*) &newMsg.raw_data[0], newMsg.raw_data.size());
}

int main(int argc, char **argv)
{
        ros::init(argc, argv, "bag2gsdb");
        ros::NodeHandle nh;
        Bag2GSDBConverter converter = Bag2GSDBConverter(&nh);
        ros::spin();

        return 0;
}