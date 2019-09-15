cmake_minimum_required(VERSION 2.8.3)
project(communication)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  
  geometry_msgs
  roscpp
  roslaunch
  std_msgs
  message_generation

)


################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

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

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES communication
  CATKIN_DEPENDS roscpp std_msgs
  DEPENDS
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include 
  ${catkin_INCLUDE_DIRS}
  include/config
)

add_executable(comm src/comm/comm.cpp)
add_executable(robot src/robot/robot.cpp)

target_link_libraries(comm ${catkin_LIBRARIES})
target_link_libraries(robot ${catkin_LIBRARIES})



#############
## Install ##
#############

## Mark cpp header files for installation
 install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
   PATTERN ".svn" EXCLUDE
 )