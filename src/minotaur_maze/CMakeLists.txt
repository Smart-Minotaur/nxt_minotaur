cmake_minimum_required(VERSION 2.8.3)
project(minotaur_maze)

find_package(catkin REQUIRED
    std_msgs
    roscpp
    tf
    message_generation
    minotaur_common
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES minotaur_maze_pc
    CATKIN_DEPENDS message_runtime
#  DEPENDS system_lib
)

include_directories(${catkin_INCLUDE_DIRS} include)
file(GLOB MINOTAUR_MAZE_SRC "src/minotaur_maze/*.cpp")

add_library(${PROJECT_NAME} ${MINOTAUR_MAZE_SRC})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME} minotaur_common_generate_messages_cpp)

file(GLOB MINOTAUR_MAZE_MAIN_SRC "src/minotaur_maze/main/*.cpp")

foreach(MAIN_SRC_FILE ${MINOTAUR_MAZE_MAIN_SRC})
    get_filename_component(MAIN_NAME ${MAIN_SRC_FILE} NAME_WE)
    add_executable(${MAIN_NAME} ${MAIN_SRC_FILE})
    target_link_libraries(${MAIN_NAME} ${catkin_LIBRARIES} ${PROJECT_NAME})
    add_dependencies(${MAIN_NAME} minotaur_common_generate_messages_cpp)
endforeach(MAIN_SRC_FILE ${MINOTAUR_MAZE_MAIN_SRC})
