cmake_minimum_required(VERSION 3.8)
project(adma_core_lib)

set(CMAKE_EXPORT_COMPILE_COMMANDS YES)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options() # (-Wall -Wextra -Wpedantic)
endif()

# find ROS dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

include_directories(include)

# define library of this package
add_library(${PROJECT_NAME}_node SHARED
  src/network/udp_socket.cpp
)

# link ROS deps to this package
ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
)

install(TARGETS ${PROJECT_NAME}_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(DIRECTORY include/
  DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_node)

ament_package()
