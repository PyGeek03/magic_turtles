cmake_minimum_required(VERSION 2.8.3)
project(magic_turtles)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

catkin_package(
   CATKIN_DEPENDS message_runtime
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
add_executable(receive_turtle src/receive_turtle.cpp)
target_link_libraries(receive_turtle ${catkin_LIBRARIES})
