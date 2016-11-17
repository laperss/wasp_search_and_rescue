cmake_minimum_required(VERSION 2.8.3)
project(drone)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  apriltags
  image_transport
  roscpp
  sensor_msgs
  geometry_msgs
  tf
  cv_bridge
  rospy
  std_msgs
  message_generation
  cmake_modules
  nodelet
  wasp_custom_msgs
  ardrone_autonomy
  genmsg
  actionlib_msgs
  actionlib
)

## Generate services in the 'srv' folder
add_service_files(FILES
#   SendCommand.srv
)

#find_package(catkin REQUIRED genmsg actionlib_msgs actionlib)
add_action_files(DIRECTORY action FILES DoCommand.action)
generate_messages(DEPENDENCIES actionlib_msgs)

catkin_package(
  INCLUDE_DIRS
  LIBRARIES apriltag_detector
  CATKIN_DEPENDS image_transport roscpp sensor_msgs geometry_msgs tf cv_bridge message_runtime std_msgs nodelet apriltags wasp_custom_msgs actionlib_msgs
  DEPENDS Eigen
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

include_directories(include ${catkin_INCLUDE_DIRS})
include_directories(
  ${Eigen_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

# set header ans source files
set(DRONE_POS_SOURCE_FILES       
  src/DroneGlobalPosition.cpp
)
set(DRONE_POS_HEADER_FILES    
  src/DroneGlobalPosition.h 
)

# set header ans source files
set(DRONE_CONTROL_SOURCE_FILES       
  src/DroneControl.cpp
)
set(DRONE_CONTROL_HEADER_FILES    
  src/DroneControl.h 
)

add_executable(control src/control_drone_main.cpp ${DRONE_CONTROL_SOURCE_FILES} ${DRONE_CONTROL_HEADER_FILES})
target_link_libraries(control ${catkin_LIBRARIES})
add_dependencies(control control tum_ardrone_gencpp)

add_executable(global_position src/global_position_main.cpp ${DRONE_POS_SOURCE_FILES} ${DRONE_POS_HEADER_FILES})
target_link_libraries(global_position ${catkin_LIBRARIES})
add_dependencies(global_position global_position tum_ardrone_gencpp)


#add_executable(apriltag src/apriltags_detection.cpp)
#target_link_libraries(apriltag ${catkin_LIBRARIES})
#add_dependencies(apriltag apriltag)

