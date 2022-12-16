cmake_minimum_required(VERSION 2.8.3)
project(adma_ros_driver)

## Compile as C++11, supported in ROS melodic and newer
add_compile_options(-std=c++11)
set(CMAKE_EXPORT_COMPILE_COMMANDS YES)

set(${PROJECT_NAME}_CATKIN_DEPS
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
  tf
  adma_msgs
)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  ${${PROJECT_NAME}_CATKIN_DEPS}
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system date_time)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS ${${PROJECT_NAME}_CATKIN_DEPS}
  DEPENDS Boost
)

set(use_SYSTEM_BOOST TRUE)
## Specify additional locations of header files
## Your package locations should be listed before other locations
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

add_executable(driver_old
  src/adma_connect.cpp
)

add_dependencies(driver_old ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME})
target_link_libraries(driver_old
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)




## Add dependencies to exported targets, like ROS msgs or srvs
# add_dependencies(${PROJECT_NAME}_core
#   ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS} 
# )

# add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_generate_messages_cpp)


# add_dependencies(${PROJECT_NAME}
#   ${catkin_EXPORTED_TARGETS}
# )

## Specify libraries to link executable targets against
# target_link_libraries(${PROJECT_NAME}_core
#   ${catkin_LIBRARIES}
# )

# target_link_libraries(${PROJECT_NAME}
#   # ${PROJECT_NAME}_core
#   ${catkin_LIBRARIES}
#   ${Boost_LIBRARIES}

# )

#############
## Install ##
#############

# Mark executables and/or libraries for installation
# install(
#   TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# # Mark cpp header files for installation
# install(
#   DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.hpp"
# )

# # Mark other files for installation
# install(
#   DIRECTORY doc
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )


#############
## Testing ##
#############

#if(CATKIN_ENABLE_TESTING)
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
## Add gtest based cpp test target and link libraries
#catkin_add_gtest(${PROJECT_NAME}-test
#  test/test_receive_utm_pub_gps.cpp
#  test/UTMTest.cpp)
#endif()

#if(TARGET ${PROJECT_NAME}-test)
#  target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME}_core)
#endif()
