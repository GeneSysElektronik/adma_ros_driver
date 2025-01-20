cmake_minimum_required(VERSION 3.8)
project(adma_ros2_delta_driver)

set(CMAKE_EXPORT_COMPILE_COMMANDS YES)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find ROS dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)

# find ADMA dependencies
find_package(adma_ros_driver_msgs REQUIRED)
find_package(adma_core_lib REQUIRED)

include_directories(include)

# define library of this package
add_library(${PROJECT_NAME}_node SHARED
  src/adma_delta_driver.cpp
)

# link ROS deps to this package
ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  rclcpp_components
  adma_ros_driver_msgs
  adma_core_lib
)

# setup executable
rclcpp_components_register_node(${PROJECT_NAME}_node
  PLUGIN "genesys::ADMADeltaDriver"
  EXECUTABLE adma_delta_driver
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
