cmake_minimum_required(VERSION 2.8.11)
project(rviz_cloud_annotation)

find_package(catkin REQUIRED COMPONENTS
  interactive_markers
  pcl_conversions
  roscpp
  tf
  visualization_msgs
  geometry_msgs
  eigen_conversions
  cmake_modules
  std_msgs
  rviz
  sensor_msgs
)

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)
find_package(PCL 1.8)

set(MAYBE_PCL_COMMON_COLORS "")
if (NOT ${PCL_FOUND})
  message(WARNING "PCL 1.8 not found, attempting 1.7...")
  find_package(PCL 1.7 REQUIRED)
  include_directories(${CMAKE_CURRENT_SOURCE_DIR}/pcl_include)
  set(MAYBE_PCL_COMMON_COLORS "${CMAKE_CURRENT_SOURCE_DIR}/pcl_include/colors.cpp")
endif()

find_package(Qt5Widgets REQUIRED)
set(CMAKE_AUTOMOC ON)
add_definitions(-DQT_NO_KEYWORDS)

add_definitions(${PCL_DEFINITIONS})
link_directories(${PCL_LIBRARY_DIRS})

################################################
## Declare ROS messages, services and actions ##
################################################

add_message_files(
  FILES
  UndoRedoState.msg
  RectangleSelectionViewport.msg
  LabeledPointCloud2.msg
  LabeledPointCloud2Array.msg
)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES rviz_cloud_annotation
#  CATKIN_DEPENDS interactive_markers pcl_conversions roscpp tf visualization_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
)

add_executable(rviz_cloud_annotation_node
  src/rviz_cloud_annotation.cpp
  src/rviz_cloud_annotation_points.cpp
  src/rviz_cloud_annotation_points_io.cpp
  src/rviz_cloud_annotation_point_plane.cpp
  src/rviz_cloud_annotation_class.cpp
  src/rviz_cloud_annotation_undo.cpp
  src/point_neighborhood.cpp
  src/point_neighborhood_search.cpp

  ${MAYBE_PCL_COMMON_COLORS}
  )

add_dependencies(rviz_cloud_annotation_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(rviz_cloud_annotation_node
  ${PCL_LIBRARIES}
  ${Boost_LIBRARIES}
  ${Eigen3_LIBRARIES}
  ${catkin_LIBRARIES}
  )

set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

add_library(rviz_cloud_annotation_plugin
  src/rviz_cloud_annotation_plugin.cpp
  src/rviz_select_tool.cpp

  ${MAYBE_PCL_COMMON_COLORS}
)

add_dependencies(rviz_cloud_annotation_plugin ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(rviz_cloud_annotation_plugin
  ${PCL_LIBRARIES}
  ${Boost_LIBRARIES}
  ${Eigen3_LIBRARIES}
  ${catkin_LIBRARIES}
  Qt5::Widgets
)

#############
## Install ##
#############

