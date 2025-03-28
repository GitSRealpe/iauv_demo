cmake_minimum_required(VERSION 3.10)
project(dual_gironas)

find_package(catkin REQUIRED
  roscpp
  rospy
  cola2_nav
  message_generation
  genmsg
  actionlib_msgs
)

add_action_files(DIRECTORY action FILES Docking.action)
generate_messages(DEPENDENCIES actionlib_msgs)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS
  CATKIN_DEPENDS roscpp rospy cola2_nav stonefish_ros
  DEPENDS
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)
