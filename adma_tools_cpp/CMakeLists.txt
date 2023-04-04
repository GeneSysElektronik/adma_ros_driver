cmake_minimum_required(VERSION 2.8.3)
project(adma_tools_cpp)

# # Compile as C++11, supported in ROS melodic and newer
add_compile_options(-std=c++11)
set(CMAKE_EXPORT_COMPILE_COMMANDS YES)

set(${PROJECT_NAME}_CATKIN_DEPS
  roscpp
  adma_ros_driver_msgs
)

# # Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  ${${PROJECT_NAME}_CATKIN_DEPS}
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS ${${PROJECT_NAME}_CATKIN_DEPS}
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/bag2gsdb_converter.cpp
)

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(bag2gsdb_converter
  src/bag2gsdb_converter.cpp
)

add_dependencies(bag2gsdb_converter ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME})
target_link_libraries(bag2gsdb_converter
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_executable(gsdb_server
  src/gsdb_server.cpp
)

add_dependencies(gsdb_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME})
target_link_libraries(gsdb_server
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)