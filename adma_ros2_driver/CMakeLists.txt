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
find_package(adma_msgs REQUIRED)
find_package(Boost REQUIRED COMPONENTS system date_time)

include_directories(include
  SYSTEM {Boost_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}_node SHARED
  src/adma_parse.cpp
  src/adma_driver.cpp
)

target_link_libraries(${PROJECT_NAME}_node
  ${Boost_LIBRARIES}
)

ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  rclcpp_components
  sensor_msgs
  std_msgs
  adma_msgs
)

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

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_node)

ament_package()

