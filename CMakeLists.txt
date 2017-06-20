cmake_minimum_required(VERSION 2.8.3)
project(nautonomous_actuation_synchronizer)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  message_generation
  roscpp
  std_msgs
  rostest
  nautonomous_msgs
  #include "visualization_msgs/MarkerArray.h"

)

set(CMAKE_CXX_FLAGS "-std=c++11 -L/usr/local/lib ${CMAKE_CXX_FLAGS} -g")

catkin_package(
    CATKIN_DEPENDS nautonomous_msgs
)

include_directories(nautonomous_msgs)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(send_actuation
  src/send_actuation.cpp
)
add_library(serial
  src/serial.cpp
)
add_library(serial_unix
  src/impl/unix.cpp
)
add_library(sabertooth
  src/sabertooth.cpp
)

add_library(watchdog
  src/watchdog.cpp
)

add_executable(command_actuation_node src/command_actuation_node.cpp)
target_link_libraries(command_actuation_node ${catkin_LIBRARIES} send_actuation serial serial_unix sabertooth watchdog)

add_executable(independent_inputs_publisher src/independent_inputs_publisher_node.cpp)
target_link_libraries(independent_inputs_publisher ${catkin_LIBRARIES})

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(sabertooth_test test/sabertooth.test src/test/sabertooth_test.cpp )
  target_link_libraries(sabertooth_test ${catkin_LIBRARIES} sabertooth)
endif()