cmake_minimum_required(VERSION 3.5)
project(adma_tools_cpp)

set(CMAKE_EXPORT_COMPILE_COMMANDS YES)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options() # (-Wall -Wextra) #-Wpedantic)
endif()

# define ROS specific dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# define own genesys specific dependencies
find_package(adma_ros_driver_msgs REQUIRED)
find_package(adma_ros2_driver REQUIRED)

include_directories(include)

# define the library for this node
add_library(${PROJECT_NAME}_node SHARED
  src/bag2gsdb_converter.cpp
  src/gsdb_server.cpp
  src/gsda_server.cpp
  src/raw_bag_decoder.cpp
)

# link ROS deps to library of this node
ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  rclcpp_components
  sensor_msgs
  std_msgs
  nav_msgs
  tf2_ros
  tf2_geometry_msgs
  adma_ros_driver_msgs
  adma_ros2_driver
)

# macro to define executables
rclcpp_components_register_node(${PROJECT_NAME}_node
  PLUGIN "genesys::tools::Bag2GSDBConverter"
  EXECUTABLE bag2gsdb_converter
)

rclcpp_components_register_node(${PROJECT_NAME}_node
  PLUGIN "genesys::tools::GSDBServer"
  EXECUTABLE gsdb_server
)

rclcpp_components_register_node(${PROJECT_NAME}_node
  PLUGIN "genesys::tools::GSDAServer"
  EXECUTABLE gsda_server
)

rclcpp_components_register_node(${PROJECT_NAME}_node
  PLUGIN "genesys::tools::RawBagDecoder"
  EXECUTABLE raw_bag_decoder
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

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_node)

ament_package()
