cmake_minimum_required(VERSION 2.8.3)
project(test_assignment)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  pcl_ros
  cv_bridge
  image_transport
  image_geometry
  backward_ros
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES test_assignment
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


add_executable(${PROJECT_NAME}_node src/merge.cc src/MyPointCloud.cc)

add_executable(${PROJECT_NAME}_ph_node src/ph_merge.cc)

target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES}
)


target_link_libraries(${PROJECT_NAME}_ph_node
   ${catkin_LIBRARIES}
)
