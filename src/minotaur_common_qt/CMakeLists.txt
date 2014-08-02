cmake_minimum_required(VERSION 2.8.3)
project(minotaur_common_qt)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake-modules")
set(MINOTAUR_COMMON_QT_LIB_NAME ${PROJECT_NAME})

find_package(catkin REQUIRED
	roscpp
    qt_build
    minotaur_common
)

find_package(Qwt REQUIRED)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${MINOTAUR_COMMON_QT_LIB_NAME}
)

#====================
# do all the QT stuff
rosbuild_prepare_qt4(
    QtCore
    QtGui
)
file(GLOB QT_FORMS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ui/*.ui)
file(GLOB QT_RESOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} resources/*.qrc)
file(GLOB_RECURSE QT_MOC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS include/${PROJECT_NAME}/*.hpp)

QT4_ADD_RESOURCES(QT_RESOURCES_CPP ${QT_RESOURCES})
QT4_WRAP_UI(QT_FORMS_HPP ${QT_FORMS})
QT4_WRAP_CPP(QT_MOC_HPP ${QT_MOC})
#====================

include_directories(include ${catkin_INCLUDE_DIRS} ${QWT_INCLUDE_DIRS})

file(GLOB MINOTAUR_COMMON_QT_LIB_SRC "src/minotaur_common_qt/*cpp")

add_library(${MINOTAUR_COMMON_QT_LIB_NAME} ${MINOTAUR_COMMON_QT_LIB_SRC} ${QT_RESOURCES_CPP} ${QT_FORMS_HPP} ${QT_MOC_HPP})
target_link_libraries(${MINOTAUR_COMMON_QT_LIB_NAME} ${QT_LIBRARIES} ${catkin_LIBRARIES} ${QWT_LIBRARIES})
add_dependencies(${MINOTAUR_COMMON_QT_LIB_NAME} minotaur_common_generate_messages_cpp)
