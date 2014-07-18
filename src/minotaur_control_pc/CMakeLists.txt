cmake_minimum_required(VERSION 2.8.3)
project(minotaur_control_pc)

find_package(catkin REQUIRED
    std_msgs
    roscpp
    tf
    message_generation
    robot_control_beagle
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES MinotaurControlPC
)

include_directories(include ${catkin_INCLUDE_DIRS})

#compile MinotaurControlPC lib
file( GLOB MINOTAUR_CONTROL_PC_LIB_SOURCES src/minotaur_control_pc/*.cpp )
add_library(MinotaurControlPC ${MINOTAUR_CONTROL_PC_LIB_SOURCES})
add_dependencies(MinotaurControlPC robot_control_beagle_generate_messages_cpp)
target_link_libraries(MinotaurControlPC ${catkin_LIBRARIES})