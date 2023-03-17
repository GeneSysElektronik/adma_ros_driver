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

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(adma_ros_driver_msgs REQUIRED)

include_directories(include)

add_library(${PROJECT_NAME}_node SHARED
  src/bag2gsdb_converter.cpp
  src/gsdb_server.cpp
)

ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  rclcpp_components
  adma_ros_driver_msgs
)

rclcpp_components_register_node(${PROJECT_NAME}_node
  PLUGIN "genesys::tools::Bag2GSDBConverter"
  EXECUTABLE bag2gsdb_converter
)

rclcpp_components_register_node(${PROJECT_NAME}_node
  PLUGIN "genesys::tools::GSDBServer"
  EXECUTABLE gsdb_server
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
