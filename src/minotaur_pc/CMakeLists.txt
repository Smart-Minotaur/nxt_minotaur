cmake_minimum_required(VERSION 2.8.3)
project(minotaur_pc)

find_package(catkin REQUIRED COMPONENTS
    std_msgs
    roscpp
    tf
    message_generation
    robot_control_beagle
)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES MinotaurPC
)

include_directories(include ${catkin_INCLUDE_DIRS})

#compile MinotaurPC lib
file( GLOB MINOTAUR_PC_LIB_SOURCES src/*.cpp )
add_library(MinotaurPC ${MINOTAUR_PC_LIB_SOURCES})
add_dependencies(MinotaurPC robot_control_beagle_generate_messages_cpp)
target_link_libraries(MinotaurPC ${catkin_LIBRARIES})

#compile all executeables
file( GLOB EXEC_SOURCES src/main/*.cpp)

foreach( mainsourcefile ${EXEC_SOURCES} )
    get_filename_component(mainname ${mainsourcefile} NAME_WE)
    add_executable(${mainname} ${mainsourcefile})
    target_link_libraries(${mainname} ${catkin_LIBRARIES} MinotaurPC)
    add_dependencies(${mainname} robot_control_beagle_generate_messages_cpp)
endforeach( mainsourcefile ${EXEC_SOURCES} )