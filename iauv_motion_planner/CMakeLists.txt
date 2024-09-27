cmake_minimum_required(VERSION 3.0.2)
project(iauv_motion_planner)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
set(CMAKE_CXX_STANDARD 17)
# set(CMAKE_CXX_STANDARD_REQUIRED ON)

# set(CMAKE_CXX_STANDARD 11)
#   set(CMAKE_CXX_STANDARD_REQUIRED ON)
#   set(CMAKE_CXX_EXTENSIONS OFF)
#   # this next line shouldn't be necessary, but doesn't always get added by cmake (e.g., for clang++-5)
#   add_definitions(-std=c++11)

set(CMAKE_MODULE_PATH
  "${CMAKE_MODULE_PATH}"
  "${CMAKE_CURRENT_SOURCE_DIR}")
  
include(../macros)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED)
find_package(Eigen3 REQUIRED)

# set(ompl_DIR /home/srealper/theLibs/ompl/build/Release/)
# set(OMPL_LIBRARIES /home/srealper/theLibs/ompl/build/Release/lib/libompl.so)
find_package(ompl REQUIRED)
message("ompl print")
message("OMPL found? ${OMPL_FOUND}")
message("2 ${OMPL_INCLUDE_DIRS}")
message("3 ${OMPL_LIBRARY_DIRS}")
message("4 ${OMPL_LIBRARY_DIR}")
message("5 ${OMPL_LIBRARIES}")
message("OMPL version ${OMPL_VERSION}")

find_package(fcl REQUIRED)
message("fcl prints")
message("fcl found? ${fcl_FOUND}")
# message("1 ${fcl_FOUND}")
# message("2 ${FCL_INCLUDE_DIRS}")
# message("3 ${FCL_LIBRARY_DIRS}")
# message("4 ${FCL_LIBRARIES}")
message("FCL version ${FCL_VERSION}")

list(APPEND CATKIN_SOURCES
    roscpp
    rospy
    tf
    tf2_ros
    eigen_conversions
    visualization_msgs
    octomap_msgs
    octomap_ros
    geometry_msgs
    nav_msgs
    rviz_visual_tools
  )

find_package(catkin REQUIRED COMPONENTS ${CATKIN_SOURCES} message_generation actionlib genmsg)

add_message_files(
  FILES
  PlannerParam.msg
)

add_service_files(
  FILES
  GetPath.srv
)


generate_messages(
 DEPENDENCIES
 geometry_msgs
 nav_msgs
)

catkin_package(
  LIBRARIES ${LIBRARY_NAME}
  CATKIN_DEPENDS ${CATKIN_SOURCES}
  DEPENDS
  INCLUDE_DIRS ${CMAKE_CURRENT_LIST_DIR}/include
  )

message("${PROJECT_NAME}")

###########s
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations

list(APPEND INCLUDES
  ${CMAKE_CURRENT_LIST_DIR}/include
  ${catkin_INCLUDE_DIRS}
  ${OMPL_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${FCL_INCLUDE_DIRS}
  )

include_directories(SYSTEM ${INCLUDES})

list(APPEND LIBRARIES
    ${OMPL_LIBRARIES}
    ${FCL_LIBRARIES}
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
)

add_library(${PROJECT_NAME}
  include/${PROJECT_NAME}/Scene.h
  src/sbmp/Validator3.cpp
  src/Scene.cpp
  src/Planner.cpp
  src/sbmp/SampleBasedPlanner.cpp
  src/sbmp/RRTPlanner.cpp
  src/ScanPlanner.cpp
  src/CircularPlanner.cpp
)
add_dependencies(iauv_motion_planner ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(iauv_motion_planner ${catkin_LIBRARIES} ${LIBRARIES})

add_executable(iauv_motion_planner_node scripts/iauv_motion_planner_node.cpp)
target_link_libraries(iauv_motion_planner_node iauv_motion_planner)

# add_script(tree_main scripts/bt)

#############
## Install ##
#############

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination


# catkin_install_python(PROGRAMS
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )
