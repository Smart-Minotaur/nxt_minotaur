cmake_minimum_required(VERSION 2.8)
project(NXTControl)

set(CMAKE_CXX_FLAGS "-std=c++0x")

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake-modules")

find_package(USB-1.0 REQUIRED)

include_directories(include ${USB-1.0_INCLUDE_DIRS})

file(GLOB LIB_SRC "src/nxt/*.cpp")
add_library(${PROJECT_NAME} ${LIB_SRC})
target_link_libraries(${PROJECT_NAME} ${USB-1.0_LIBRARIES})
