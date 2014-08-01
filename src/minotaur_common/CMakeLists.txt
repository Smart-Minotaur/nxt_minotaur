cmake_minimum_required(VERSION 2.8.3)
project(minotaur_common)

set(MINOTAUR_COMMON_LIB ${PROJECT_NAME})

find_package(catkin REQUIRED
    std_msgs
    roscpp
    tf
    message_generation
)

# Generate messages in the 'msg' folder
add_message_files(
    FILES
    UltrasonicData.msg
    PIDParameter.msg
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

#export info
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${MINOTAUR_COMMON_LIB}
  CATKIN_DEPENDS message_runtime
)

include_directories(include ${catkin_INCLUDE_DIRS})

#compile lib
file( GLOB MINOTAUR_COMMON_SRC "src/minotaur_common/*.cpp" )
add_library(${MINOTAUR_COMMON_LIB} ${MINOTAUR_COMMON_SRC})
target_link_libraries(${MINOTAUR_COMMON_LIB} ${catkin_LIBRARIES})
