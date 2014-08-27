cmake_minimum_required(VERSION 2.8.3)
project(mouse_monitor_beagle)

find_package(catkin REQUIRED COMPONENTS
    std_msgs
    roscpp
    message_generation
)

# Generate messages in the 'msg' folder
add_message_files(
    FILES
    MouseMonitorSensorData.msg
    MouseMonitorSensorSettings.msg
)

# Generate services in the 'srv' folder
add_service_files(
    FILES
    MouseMonitorSensorGetSettings.srv
    MouseMonitorSensorGetData.srv
    MouseMonitorSensorSetResolution.srv
)

# Generate added messages and services with any dependencies listed here
generate_messages(   
    DEPENDENCIES
    std_msgs
)

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS message_runtime
)

if(${USE_MOUSE_SENSOR})
	# get source files
	file(GLOB MOUSE_MONITOR_SRC_DIR src/mouse_monitor)
	file(GLOB MOUSE_MONITOR_SRC ${MOUSE_MONITOR_SRC_DIR}/*.cpp)

	# get dependencies to PLN library
	set(PLN_MINOTAUR_SVN_DIRECTORY ../../../pln_minotaur)
	file(GLOB PLN_MINOTAUR_LIB ${PLN_MINOTAUR_SVN_DIRECTORY}/trunk/build)
	file(GLOB PLN_MINOTAUR_INCLUDE_DIRECTORY ${PLN_MINOTAUR_SVN_DIRECTORY}/trunk/src)

	include_directories(include ${catkin_INCLUDE_DIRS} ${PLN_MINOTAUR_INCLUDE_DIRECTORY} include)
	link_directories(${PLN_MINOTAUR_LIB})

	# compile excuteable
	add_executable(${PROJECT_NAME} ${MOUSE_MONITOR_SRC})
	target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} pln_minotaur)
	add_dependencies(${PROJECT_NAME} mouse_monitor_beagle_generate_messages_cpp)
endif(${USE_MOUSE_SENSOR})
