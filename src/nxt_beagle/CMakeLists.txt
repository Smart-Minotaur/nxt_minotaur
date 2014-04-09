cmake_minimum_required(VERSION 2.8.3)
project(nxt_beagle)

#set Version
set(NXT_BEAGLE_MAJOR_VERSION 0)
set(NXT_BEAGLE_MINOR_VERSION 1)
set(NXT_BEAGLE_PATCH_VERSION 0)
set(NXT_BEAGLE_VERSION "${NXT_BEAGLE_MAJOR_VERSION}.${NXT_BEAGLE_MINOR_VERSION}.${NXT_BEAGLE_PATCH_VERSION}")

set(NXT_BEAGLE_INCLUDE_DIRS "include")
set(NXT_BEAGLE_HEADER_DIR "${NXT_BEAGLE_INCLUDE_DIRS}/nxt_beagle")
get_filename_component(ABS_NXT_BEAGLE_HEADER_DIR ${NXT_BEAGLE_HEADER_DIR} ABSOLUTE)

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
    MVelocity.msg
    RVelocity.msg
    SamplingInterval.msg
    UltraSensor.msg
    PIDParam.msg
    ClearSensor.msg
    SetModel.msg
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

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS message_runtime
)

include_directories(include ${catkin_INCLUDE_DIRS} ${NXT_BEAGLE_INCLUDE_DIRS})

#configure headers with cmake metainfo
file( GLOB CONFIGURE_HEADERS config_headers/*.hpp.in)

foreach(configheader ${CONFIGURE_HEADERS})
    get_filename_component(configheadername ${configheader} NAME_WE)
    configure_file (
        "${configheader}"
        "${ABS_NXT_BEAGLE_HEADER_DIR}/${configheadername}.hpp"
        )
endforeach(configheader ${CONFIGURE_HEADERS})

#compile MotorControl lib
file( GLOB MOTOR_CONTROL_SOURCES MotorControl/*.cpp )
add_library(MotorControl ${MOTOR_CONTROL_SOURCES})
add_dependencies(MotorControl nxt_beagle_generate_messages_cpp)

#compile all test executeables
file( GLOB TEST_SOURCES */test/*.cpp)

foreach( testsourcefile ${TEST_SOURCES} )
    get_filename_component(testname ${testsourcefile} NAME_WE)
    add_executable(${testname} ${testsourcefile})
    target_link_libraries(${testname} ${catkin_LIBRARIES} MotorControl)
    add_dependencies(${testname} nxt_beagle_generate_messages_cpp)
endforeach( testsourcefile ${TEST_SOURCES} )

#compile RobotControl node
add_executable(RobotControl RobotControl/RobotControl.cpp)
target_link_libraries(RobotControl ${catkin_LIBRARIES} MotorControl)
add_dependencies(RobotControl nxt_beagle_generate_messages_cpp)