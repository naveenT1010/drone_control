cmake_minimum_required(VERSION 2.8.3)
project(drone_control)

include_directories(${OpenCV_INCLUDE_DIRS})
LINK_DIRECTORIES(/usr/local/lib)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages


find_package(OpenCV REQUIRED
	#NO_MODULE # should be optional, tells CMake to use config mode
	#PATHS /usr/local # look here
	#NO_DEFAULT_PATH
) # and don't look anywhere else

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mavros
  roscpp
  rospy
  sensor_msgs
  std_msgs
  cv_bridge
  image_transport
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES drone_control
#  CATKIN_DEPENDS geometry_msgs mavros roscpp rospy sensor_msgs std_msgs
#  DEPENDS system_lib
)

#find_package(
#    OpenCV REQUIRED
#)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(offb_node src/offb_node.cpp)
add_dependencies(offb_node ${catkin_INCLUDE_DIRS})
target_link_libraries(offb_node ${catkin_LIBRARIES})

add_executable(drone_tracker src/drone_tracker.cpp)
add_dependencies(drone_tracker ${catkin_INCLUDE_DIRS})
#target_link_libraries(drone_tracker ${OpenCV_LIBS})
target_link_libraries(drone_tracker ${catkin_LIBRARIES} /usr/local/lib/libopencv_tracking.so)
