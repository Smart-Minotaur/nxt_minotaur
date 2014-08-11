cmake_minimum_required(VERSION 2.8.3)
project(minotaur_map)

set(LIB_NAME "${PROJECT_NAME}")

find_package(catkin REQUIRED COMPONENTS
    std_msgs
    roscpp
    tf
    minotaur_common
)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${LIB_NAME}
)

include_directories(include ${catkin_INCLUDE_DIRS})

#compile MinotaurPC lib
file( GLOB LIB_SRC src/minotaur_map/*.cpp )
add_library(${LIB_NAME} ${LIB_SRC})
add_dependencies(${LIB_NAME} minotaur_common_generate_messages_cpp)
target_link_libraries(${LIB_NAME} ${catkin_LIBRARIES})

#compile all executeables
file( GLOB EXEC_SRC src/minotaur_map/main/*.cpp)

foreach( mainsourcefile ${EXEC_SRC} )
    get_filename_component(mainname ${mainsourcefile} NAME_WE)
    add_executable(${mainname} ${mainsourcefile})
    target_link_libraries(${mainname} ${catkin_LIBRARIES} ${LIB_NAME})
    add_dependencies(${mainname} minotaur_common_generate_messages_cpp)
endforeach( mainsourcefile ${EXEC_SRC} )
