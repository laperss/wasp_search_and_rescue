cmake_minimum_required(VERSION 2.8.3)
project(drone)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
#  apriltags
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
  ardrone_autonomy
  genmsg
  actionlib_msgs
  actionlib
#  rosplan_dispatch_msgs
#  rosplan_knowledge_msgs
#  rosplan_planning_system
  diagnostic_msgs
)

## Generate messages in the 'msg' folder
add_message_files(DIRECTORY msg
  FILES
  object_pose.msg
)

add_service_files(FILES
  SendCommand.srv
  StringService.srv
)

#find_package(catkin REQUIRED genmsg actionlib_msgs actionlib)
add_action_files(DIRECTORY action 
  FILES 
  DoCommand.action
  DoPositionCommand.action
)
generate_messages(DEPENDENCIES actionlib_msgs  geometry_msgs)

catkin_package(
  INCLUDE_DIRS
#  LIBRARIES apriltag_detector
  CATKIN_DEPENDS image_transport roscpp sensor_msgs geometry_msgs tf cv_bridge message_runtime 
                 std_msgs nodelet actionlib_msgs # apriltag
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

# Global position header and source files
set(DRONE_POS_SOURCE_FILES       
  src/DroneGlobalPosition.cpp
)
set(DRONE_POS_HEADER_FILES    
  src/DroneGlobalPosition.h 
)

# Control header and source files
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

# Example file
add_executable(search_and_rescue_test src/search_rescue_test.cpp)
target_link_libraries(search_and_rescue_test ${catkin_LIBRARIES})
add_dependencies(search_and_rescue_test search_and_rescue_test)


