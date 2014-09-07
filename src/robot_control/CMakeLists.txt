cmake_minimum_required(VERSION 2.8.3)
project(robot_control)

set(CMAKE_CXX_FLAGS "-std=c++0x")

add_subdirectory("lib/NXTControl")

find_package(catkin REQUIRED COMPONENTS
	std_msgs
	roscpp
	tf
	message_generation
	minotaur_common
)

# export info
catkin_package(
	INCLUDE_DIRS include
	CATKIN_DEPENDS message_runtime
)

include_directories(include ${catkin_INCLUDE_DIRS} "lib/NXTControl/include")

set(ROBOT_CONTROL_DIR src/robot_control)
file(GLOB ROBOT_CONTROL_LIB_SRC ${ROBOT_CONTROL_DIR}/*.cpp)
file(GLOB ROBOT_CONTROL_MAIN_SRC ${ROBOT_CONTROL_DIR}/main/*.cpp)

add_library(robot_control ${ROBOT_CONTROL_LIB_SRC})
target_link_libraries(robot_control ${catkin_LIBRARIES} NXTControl)
add_dependencies(robot_control minotaur_common_generate_messages_cpp)

# compile main executeables
foreach(MAIN_SRC_FILE ${ROBOT_CONTROL_MAIN_SRC})
	get_filename_component(MAIN_FILE_NAME ${ROBOT_CONTROL_MAIN_SRC} NAME_WE)
	add_executable(${MAIN_FILE_NAME} ${ROBOT_CONTROL_MAIN_SRC})
	target_link_libraries(${MAIN_FILE_NAME} ${catkin_LIBRARIES} NXTControl robot_control)
	add_dependencies(${MAIN_FILE_NAME} minotaur_common_generate_messages_cpp)
endforeach(MAIN_SRC_FILE ${ROBOT_CONTROL_MAIN_SRC})

# build test files only if the flag is set
if(${BUILD_TEST_FILES})
	#get source files
	file(GLOB ROBOT_CONTROL_TEST_SRC ${ROBOT_CONTROL_DIR}/test/*.cpp)

	foreach(TEST_SRC_FILE ${ROBOT_CONTROL_TEST_SRC})
		get_filename_component(TEST_FILE_NAME ${TEST_SRC_FILE} NAME_WE)
		add_executable(${TEST_FILE_NAME} ${TEST_SRC_FILE})
		target_link_libraries(${TEST_FILE_NAME} ${catkin_LIBRARIES} NXTControl robot_control)
		add_dependencies(${TEST_FILE_NAME} minotaur_common_generate_messages_cpp)
	endforeach(TEST_SRC_FILE ${NXT_CONTROL_TEST_SRC})
endif(${BUILD_TEST_FILES})

