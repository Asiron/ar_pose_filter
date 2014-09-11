cmake_minimum_required(VERSION 2.8.3)
project(drift_correction)

set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")

set(ROS_CXX_DEPENDENCIES
  geometry_msgs
  ar_pose_msgs
  std_msgs
  std_srvs
  tf
  roscpp
)

find_package(catkin REQUIRED COMPONENTS
  ${ROS_CXX_DEPENDENCIES}
)

#find Boost
find_package(Boost REQUIRED COMPONENTS signals)

#include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
#link_directories(${catkin_LIBRARY_DIRS})

catkin_package(
   CATKIN_DEPENDS ${ROS_CXX_DEPENDENCIES}
)

#create target nao_waypoint_node
#add_executable(drift_correction_node src/drift_correction.cpp)
#target_link_libraries(drift_correction_node ${catkin_LIBRARIES})
#add_dependencies(drift_correction_node ${catkin_EXPORTED_TARGETS})

#install nao_waypoint
#install(TARGETS
#  drift_correction_node
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)
