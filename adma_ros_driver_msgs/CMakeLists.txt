cmake_minimum_required(VERSION 3.5)
project(adma_ros_driver_msgs)

set(CMAKE_EXPORT_COMPILE_COMMANDS YES)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/AdmaDataDelta.msg"
  "msg/AdmaData.msg"
  "msg/AdmaDataRaw.msg"
  "msg/AdmaDataScaled.msg"
  "msg/POI.msg"
  "msg/ByteErrorWarning.msg"
  "msg/ByteStatus.msg"
  "msg/ErrorWarning.msg"
  "msg/Status.msg"
  "msg/AdmaStatus.msg"
  DEPENDENCIES builtin_interfaces geometry_msgs std_msgs
)

ament_package()
