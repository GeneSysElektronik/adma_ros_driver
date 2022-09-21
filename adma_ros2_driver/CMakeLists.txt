cmake_minimum_required(VERSION 3.5)
project(adma_ros2_driver)

set(CMAKE_EXPORT_COMPILE_COMMANDS YES)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options() #(-Wall -Wextra) #-Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(adma_msgs REQUIRED)

include_directories(include
)

add_library(${PROJECT_NAME}_node SHARED
  src/parser/adma_parse_deprecated.cpp
  src/parser/adma2ros_parser_v32.cpp
  src/parser/parser_utils.cpp
  src/adma_driver.cpp
  src/adma_data_server.cpp
)

ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  rclcpp_components
  sensor_msgs
  std_msgs
  tf2_ros
  tf2_geometry_msgs
  adma_msgs
)

rclcpp_components_register_node(${PROJECT_NAME}_node
  PLUGIN "genesys::ADMADriver"
  EXECUTABLE adma_driver
)

rclcpp_components_register_node(${PROJECT_NAME}_node
  PLUGIN "genesys::DataServer"
  EXECUTABLE data_server
)

install(TARGETS ${PROJECT_NAME}_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(DIRECTORY include/
  DESTINATION include)

# install additional files of folders
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_node)

ament_package()
