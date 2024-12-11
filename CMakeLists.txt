cmake_minimum_required(VERSION 3.0.2)
project(livox_transform)

######################
### Cmake flags
######################
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++17")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g -pthread")

find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  dynamic_reconfigure
  livox_ros_driver
)

generate_dynamic_reconfigure_options(
  config/Test.cfg
)

catkin_package(CATKIN_DEPENDS rospy dynamic_reconfigure livox_ros_driver)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME} src/livox_transform.cpp)

target_link_libraries(${PROJECT_NAME}
  ${OpenCV_LIBRARIES}
  ${catkin_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)