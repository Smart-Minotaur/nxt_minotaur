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

file(GLOB MINOTAUR_MAZE_MAIN_SRC "src/minotaur_maze_pc/main/*.cpp")

foreach(MAIN_SRC_FILE ${MINOTAUR_MAZE_MAIN_SRC})
    get_filename_component(MAIN_NAME ${MAIN_SRC_FILE} NAME_WE)
    add_executable(${MAIN_NAME} ${MAIN_SRC_FILE})
    target_link_libraries(${MAIN_NAME} ${catkin_LIBRARIES} MinotaurMazePC)
    add_dependencies(${MAIN_NAME} robot_control_beagle_generate_messages_cpp)
endforeach(MAIN_SRC_FILE ${MINOTAUR_MAZE_MAIN_SRC})
