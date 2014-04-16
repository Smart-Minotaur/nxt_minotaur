cmake_minimum_required(VERSION 2.8.3)
project(minotaur_pc)

#set Version
set(MINOTAUR_PC_MAJOR_VERSION 0)
set(MINOTAUR_PC_MINOR_VERSION 1)
set(MINOTAUR_PC_PATCH_VERSION 0)
set(MINOTAUR_PC_VERSION "${MINOTAUR_PC_MAJOR_VERSION}.${MINOTAUR_PC_MINOR_VERSION}.${MINOTAUR_PC_PATCH_VERSION}")

find_package(catkin REQUIRED COMPONENTS
    std_msgs
    roscpp
    rospy
    message_generation
    nxt_beagle
)

# Generate messages in the 'msg' folder
add_message_files()

# Generate services in the 'srv' folder
add_service_files()

generate_messages(   
    DEPENDENCIES
    std_msgs
)

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS message_runtime
)

include_directories(include ${catkin_INCLUDE_DIRS} ${NXT_BEAGLE_INCLUDE_DIRS})

# process confiurable headers
file( GLOB CONFIGURE_HEADERS config_headers/*.hpp.in)

foreach(configheader ${CONFIGURE_HEADERS})
    get_filename_component(configheadername ${configheader} NAME_WE)
    configure_file (
        "${configheader}"
        "${ABS_NXT_BEAGLE_HEADER_DIR}/${configheadername}.hpp"
        )
endforeach(configheader ${CONFIGURE_HEADERS})

#compile MotorControl lib
file( GLOB MINOTAUR_PC_LIB_SOURCES src/*.cpp )
add_library(MinotaurPC ${MINOTAUR_PC_LIB_SOURCES})
add_dependencies(MinotaurPC nxt_beagle_generate_messages_cpp)

#compile all executeables
file( GLOB EXEC_SOURCES src/main/*.cpp)

foreach( mainsourcefile ${EXEC_SOURCES} )
    get_filename_component(mainname ${mainsourcefile} NAME_WE)
    add_executable(${mainname} ${mainsourcefile})
    target_link_libraries(${mainname} ${catkin_LIBRARIES} MotorControl)
    add_dependencies(${mainname} nxt_beagle_generate_messages_cpp)
endforeach( mainsourcefile ${EXEC_SOURCES} )