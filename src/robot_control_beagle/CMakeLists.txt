cmake_minimum_required(VERSION 2.8.3)
project(robot_control_beagle)

# add c++ standard
set(CMAKE_CXX_FLAGS "-std=c++0x")

find_package(catkin REQUIRED COMPONENTS
    std_msgs
    roscpp
    tf
    message_generation
)

# Generate messages in the 'msg' folder
add_message_files(
    FILES
    UltrasonicData.msg
    SetPIDParameter.msg
    ClearSensor.msg
)

# Generate services in the 'srv' folder
add_service_files(
    FILES
    GetUltrasonic.srv
    AddUltrasonic.srv
)

# Generate added messages and services with any dependencies listed here
generate_messages(   
    DEPENDENCIES
    std_msgs
)

# export info
catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS message_runtime
)

include_directories(include ${catkin_INCLUDE_DIRS} include)

if(${USE_MOUSE_SENSOR})
    # get source files for comnpilation
    set(NXT_CONTROL_DIR src/nxt_control)
    file(GLOB NXT_CONTROL_LIB_SRC ${NXT_CONTROL_DIR}/*.cpp)
    file(GLOB NXT_CONTROL_TEST_SRC ${NXT_CONTROL_DIR}/test/*.cpp)

    set(ROBOT_CONTROL_DIR src/robot_control)
    file(GLOB ROBOT_CONTROL_LIB_SRC ${ROBOT_CONTROL_DIR}/*.cpp)
    file(GLOB ROBOT_CONTROL_TEST_SRC ${ROBOT_CONTROL_DIR}/test/*.cpp)
    file(GLOB ROBOT_CONTROL_MAIN_SRC ${ROBOT_CONTROL_DIR}/main/*.cpp)

    # compile libraries
    add_library(nxt_control ${NXT_CONTROL_LIB_SRC})
    add_dependencies(nxt_control robot_control_beagle_generate_messages_cpp)

    add_library(robot_control ${ROBOT_CONTROL_LIB_SRC})
    target_link_libraries(robot_control ${catkin_LIBRARIES} nxt_control)
    add_dependencies(robot_control robot_control_beagle_generate_messages_cpp)

    # compile test files
    foreach(TEST_SRC_FILE ${NXT_CONTROL_TEST_SRC})
        get_filename_component(TEST_FILE_NAME ${TEST_SRC_FILE} NAME_WE)
        add_executable(${TEST_FILE_NAME} ${TEST_SRC_FILE})
        target_link_libraries(${TEST_FILE_NAME} ${catkin_LIBRARIES} nxt_control usb-1.0)
        add_dependencies(${TEST_FILE_NAME} robot_control_beagle_generate_messages_cpp)
    endforeach(TEST_SRC_FILE ${NXT_CONTROL_TEST_SRC})

    foreach(TEST_SRC_FILE ${ROBOT_CONTROL_TEST_SRC})
        get_filename_component(TEST_FILE_NAME ${TEST_SRC_FILE} NAME_WE)
        add_executable(${TEST_FILE_NAME} ${TEST_SRC_FILE})
        target_link_libraries(${TEST_FILE_NAME} ${catkin_LIBRARIES} nxt_control robot_control usb-1.0)
        add_dependencies(${TEST_FILE_NAME} robot_control_beagle_generate_messages_cpp)
    endforeach(TEST_SRC_FILE ${NXT_CONTROL_TEST_SRC})

    # compile main executeables
    foreach(MAIN_SRC_FILE ${ROBOT_CONTROL_MAIN_SRC})
        get_filename_component(MAIN_FILE_NAME ${ROBOT_CONTROL_MAIN_SRC} NAME_WE)
        add_executable(${MAIN_FILE_NAME} ${ROBOT_CONTROL_MAIN_SRC})
        target_link_libraries(${MAIN_FILE_NAME} ${catkin_LIBRARIES} nxt_control robot_control usb-1.0)
        add_dependencies(${MAIN_FILE_NAME} robot_control_beagle_generate_messages_cpp)
    endforeach(MAIN_SRC_FILE ${ROBOT_CONTROL_MAIN_SRC})
endif(${USE_MOUSE_SENSOR})