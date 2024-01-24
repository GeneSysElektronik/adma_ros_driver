cmake_minimum_required(VERSION 3.5)
project(adma_ros2_driver)

set(CMAKE_EXPORT_COMPILE_COMMANDS YES)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options() # (-Wall -Wextra) #-Wpedantic)
endif()

# define ROS dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# define genesys dependencies
find_package(adma_ros_driver_msgs REQUIRED)

include_directories(include)

# define library of this package
add_library(${PROJECT_NAME}_node SHARED
  src/parser/adma2ros_parser.cpp
  src/parser/adma2ros_parser_v32.cpp
  src/parser/adma2ros_parser_v333.cpp
  src/parser/adma2ros_parser_v334.cpp
  src/parser/adma2ros_parser_v335.cpp
  src/parser/parser_utils.cpp
  src/adma_driver.cpp
)

# link ROS deps to this package
ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  rclcpp_components
  sensor_msgs
  geometry_msgs
  nav_msgs
  std_msgs
  tf2_ros
  tf2_geometry_msgs
  adma_ros_driver_msgs
)

# setup executable
rclcpp_components_register_node(${PROJECT_NAME}_node
  PLUGIN "genesys::ADMADriver"
  EXECUTABLE adma_driver
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
