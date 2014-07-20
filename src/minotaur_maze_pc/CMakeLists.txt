cmake_minimum_required(VERSION 2.8.3)
project(minotaur_maze_pc)

find_package(catkin REQUIRED
    std_msgs
    roscpp
    tf
    message_generation
    robot_control_beagle
    minotaur_control_pc
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES minotaur_maze_pc
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

include_directories(${catkin_INCLUDE_DIRS} include)
file(GLOB MINOTAUR_MAZE_SRC "src/minotaur_maze_pc/*.cpp")

add_library(MinotaurMazePC ${MINOTAUR_MAZE_SRC})
target_link_libraries(MinotaurMazePC ${catkin_LIBRARIES})
add_dependencies(MinotaurMazePC robot_control_beagle_generate_messages_cpp)
