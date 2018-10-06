cmake_minimum_required(VERSION 2.8.3)
project(imu_publisher)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
)

catkin_package(
)

include_directories(
  include ${catkin_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

file(GLOB_RECURSE LIB_SRCS "*.cpp")
file(GLOB_RECURSE LIB_HEADERS "*.h")
list(REMOVE_ITEM LIB_SRCS "src/imu_publisher.cpp")
add_library(${PROJECT_NAME} ${LIB_SRCS} ${LIB_HEADERS})

add_executable(imu_pub
  src/imu_publisher.cpp
)

add_dependencies(imu_pub ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(imu_pub
  ${catkin_LIBRARIES}
  /usr/local/lib/libRTIMULib.so
  /usr/local/lib/libRTIMULib.so.7
  /usr/local/lib/libRTIMULib.so.7.2.1
)

