cmake_minimum_required(VERSION 2.8.3)
project(tag_position)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  tf
  turtlesim
  std_msgs
  geometry_msgs
  message_generation
)

add_service_files(FILES
   EditPosition.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs  # and/or other packages containing depended messages
  geometry_msgs  # and/or other packages containing depended messages
)

catkin_package(
  CATKIN_DEPENDS message_runtime roscpp geometry_msgs std_msgs
)





include_directories(
  ${catkin_INCLUDE_DIRS}
)



add_executable(tag_pos src/tag_main.cpp src/Tag.h src/Tag.cpp)
target_link_libraries(tag_pos ${catkin_LIBRARIES})
add_dependencies(tag_pos edit_tag_pos ${PROJECT_NAME}_generate_messages_cpp)
