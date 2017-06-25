cmake_minimum_required(VERSION 2.8.3)
project(drone_control)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mavros
  roscpp
  rospy
  sensor_msgs
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES drone_control
#  CATKIN_DEPENDS geometry_msgs mavros roscpp rospy sensor_msgs std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(offb_node src/offb_node.cpp)
add_dependencies(offb_node ${catkin_INCLUDE_DIRS})
target_link_libraries(offb_node ${catkin_LIBRARIES})


