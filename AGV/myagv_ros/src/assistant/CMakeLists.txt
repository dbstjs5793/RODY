cmake_minimum_required(VERSION 3.0.2)
project(assistant)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  move_base_msgs
  roscpp
  rospy
  std_msgs
  tf
)
find_package(Boost REQUIRED COMPONENTS system thread filesystem)
find_package(PkgConfig REQUIRED)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES assistant
#  CATKIN_DEPENDS actionlib move_base_msgs roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
   include
  ${catkin_INCLUDE_DIRS}
  #${CMAKE_CURRENT_SOURCE_DIR}
  ${Boost_INCLUDE_DIRS}
  /usr/include/jsoncpp
  #${CMAKE_CURRENT_SOURCE_DIR}/include
)

install(PROGRAMS
       src/record_location.py
DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


add_executable( goal_sending src/goal_sending.cpp)
target_link_libraries(goal_sending
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  /usr/lib/aarch64-linux-gnu/libjsoncpp.so
)
