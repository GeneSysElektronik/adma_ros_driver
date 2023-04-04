cmake_minimum_required(VERSION 2.8.3)
project(adma_ros_driver)

# # Compile as C++11, supported in ROS melodic and newer
add_compile_options(-std=c++11)
set(CMAKE_EXPORT_COMPILE_COMMANDS YES)

set(${PROJECT_NAME}_CATKIN_DEPS
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
  tf
  adma_ros_driver_msgs
)

# # Find catkin macros and libraries
# # if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
# # is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  ${${PROJECT_NAME}_CATKIN_DEPS}
)

# # System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system date_time)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS ${${PROJECT_NAME}_CATKIN_DEPS}
  DEPENDS Boost
)

set(use_SYSTEM_BOOST TRUE)

# # Specify additional locations of header files
# # Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  SYSTEM {Boost_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/adma_parse.cpp
  src/parser/adma2ros_parser.cpp
  src/parser/adma2ros_parser_v333.cpp
  src/parser/adma2ros_parser_v334.cpp
  src/parser/parser_utils.cpp
)

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(driver_new
  src/adma_driver.cpp
)

add_dependencies(driver_new ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME})
target_link_libraries(driver_new
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)