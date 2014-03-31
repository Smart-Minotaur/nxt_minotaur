cmake_minimum_required(VERSION 2.8.3)
project(nxt_minotaur)

find_package(catkin REQUIRED COMPONENTS
    std_msgs
    roscpp
    rospy
    message_generation
)


################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
add_message_files(
    FILES
    nxtPower.msg
)

## Generate services in the 'srv' folder
add_service_files(
    FILES
    nxtUltrasonic.srv
    nxtTicks.srv
    nxtAddUltrasonic.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(   
    DEPENDENCIES
    std_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need

catkin_package(
#    INCLUDE_DIRS include
#  LIBRARIES nxt_minotaur
#  CATKIN_DEPENDS message_runtime
#  DEPENDS system_lib
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(nxtTeleop nxtControl/test/nxtTeleop.cpp)
target_link_libraries(nxtTeleop ${catkin_LIBRARIES})
add_dependencies(nxtTeleop nxt_minotaur_generate_messages_cpp)

add_executable(nxtLatency nxtControl/test/nxtLatency.cpp)
target_link_libraries(nxtLatency ${catkin_LIBRARIES})
add_dependencies(nxtLatency nxt_minotaur_generate_messages_cpp)

add_executable(nxtMeassure nxtControl/test/nxtMeassure.cpp)
target_link_libraries(nxtMeassure ${catkin_LIBRARIES})
add_dependencies(nxtMeassure nxt_minotaur_generate_messages_cpp)