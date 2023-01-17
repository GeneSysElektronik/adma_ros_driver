cmake_minimum_required(VERSION 3.0.2)
project(adma_msgs)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
set(CMAKE_EXPORT_COMPILE_COMMANDS YES)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        geometry_msgs
        message_generation
)

## Generate messages in the 'msg' folder
add_message_files(
        FILES
        Adma_delta.msg
        Adma.msg
        AdmaDataRaw.msg
        AdmaDataScaled.msg
        AdmaStatus.msg
        ByteErrorWarning.msg
        ByteStatus.msg
        ErrorWarning.msg
        POI.msg
        Status.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
        DEPENDENCIES
        std_msgs
        geometry_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
        CATKIN_DEPENDS 
        std_msgs
        geometry_msgs
        DEPENDS
        message_runtime
)