cmake_minimum_required(VERSION 2.8.3)
project(nxt_beagle)

#set Version
set(NXT_BEAGLE_MAJOR_VERSION 0)
set(NXT_BEAGLE_MINOR_VERSION 1)
set(NXT_BEAGLE_PATCH_VERSION 0)
set(NXT_BEAGLE_VERSION "${NXT_BEAGLE_MAJOR_VERSION}.${NXT_BEAGLE_MINOR_VERSION}.${NXT_BEAGLE_PATCH_VERSION}")

find_package(catkin REQUIRED COMPONENTS
    std_msgs
    roscpp
    rospy
    message_generation
)

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

## Generate added messages and services with any dependencies listed here
generate_messages(   
    DEPENDENCIES
    std_msgs
)

catkin_package( )

#configure headers with cmake metainfo
file( GLOB CONFIGURE_HEADERS global/*.hpp.in)

foreach(configheader ${CONFIGURE_HEADERS})
    get_filename_component(configheadername ${configheader} NAME_WE)
     get_filename_component(configheaderpath ${configheader} PATH)
    configure_file (
        "${configheader}"
        "${configheaderpath}/${configheadername}.hpp"
        )
endforeach( configheader ${CONFIGURE_HEADERS} )

include_directories(include ${catkin_INCLUDE_DIRS} motorControl)

file( GLOB MOTOR_CONTROL_SOURCES motorControl/*.cpp )
file( GLOB MOTOR_CONTROL_HEADERS motorControl/*.hpp )
add_library(MotorControl ${MOTOR_CONTROL_SOURCES} ${MOTOR_CONTROL_HEADERS})

file( GLOB TEST_SOURCES */test/*.cpp)

#compile all test executeables
foreach( testsourcefile ${TEST_SOURCES} )
    get_filename_component(testname ${testsourcefile} NAME_WE)
    add_executable(${testname} ${testsourcefile})
    target_link_libraries(${testname} ${catkin_LIBRARIES} MotorControl)
endforeach( testsourcefile ${TEST_SOURCES} )