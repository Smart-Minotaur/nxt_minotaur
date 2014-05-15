cmake_minimum_required(VERSION 2.8.3)
project(nxt_beagle)

#add c++ standard
SET(CMAKE_CXX_FLAGS "-std=c++0x")

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
    tf
    message_generation
)

## Generate messages in the 'msg' folder
add_message_files(
    FILES
    UltraSensor.msg
    PIDParam.msg
    ClearSensor.msg
    DebugMouseData.msg
)

## Generate services in the 'srv' folder
add_service_files(
    FILES
    nxtUltrasonic.srv
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

#compile NxtControl lib
file( GLOB NXT_CONTROL_SOURCES NxtControl/*.cpp )
add_library(NxtControl ${NXT_CONTROL_SOURCES})
add_dependencies(NxtControl nxt_beagle_generate_messages_cpp)

set(MINOTAUR_CUSTOM_LIBS NxtControl MotorControl usb-1.0)

#compile all test executeables
file( GLOB TEST_SOURCES */test/*.cpp)

foreach( testsourcefile ${TEST_SOURCES} )
    get_filename_component(testname ${testsourcefile} NAME_WE)
    add_executable(${testname} ${testsourcefile})
    target_link_libraries(${testname} ${catkin_LIBRARIES} ${MINOTAUR_CUSTOM_LIBS})
    add_dependencies(${testname} nxt_beagle_generate_messages_cpp)
endforeach( testsourcefile ${TEST_SOURCES} )

#compile RobotControl node
file( GLOB ROBOT_CONTROL_SOURCES RobotControl/*.cpp )
add_executable(RobotControl ${ROBOT_CONTROL_SOURCES})
target_link_libraries(RobotControl ${catkin_LIBRARIES}  ${MINOTAUR_CUSTOM_LIBS})
add_dependencies(RobotControl nxt_beagle_generate_messages_cpp)

#compile MouseSensor node
if(${USE_MOUSE_SENSOR})
    file(GLOB MINOTAUR_LIB ../../../trunk/build)
    file(GLOB MINOTAUR ../../../trunk/src)
    include_directories(${MINOTAUR})
    link_directories(${MINOTAUR_LIB})

    file( GLOB MOUSE_SENSOR_SOURCES MouseSensor/*.cpp )
    add_executable(MouseNode ${MOUSE_SENSOR_SOURCES})
    target_link_libraries(MouseNode ${catkin_LIBRARIES} ${MINOTAUR_CUSTOM_LIBS} pln_minotaur)
    add_dependencies(MouseNode nxt_beagle_generate_messages_cpp)
endif(${USE_MOUSE_SENSOR})
