cmake_minimum_required(VERSION 2.8.3)
project(minotaur_teleop)

find_package(catkin REQUIRED
    std_msgs
    roscpp
    message_generation
    minotaur_common
)

catkin_package(
	CATKIN_DEPENDS message_runtime
)

include_directories(${catkin_INCLUDE_DIRS} include)
file( GLOB JOY_TELEOP_SRC src/joy_teleop.cpp)

add_executable(joy_teleop ${JOY_TELEOP_SRC})
target_link_libraries(joy_teleop ${catkin_LIBRARIES})
add_dependencies(joy_teleop minotaur_common_generate_messages_cpp)