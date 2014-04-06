cmake_minimum_required(VERSION 2.8.0)
project(nxt_qt)

find_package(catkin REQUIRED COMPONENTS
    qt_build
    roscpp
    nxt_beagle
)

file( GLOB NXT_QT_INCLUDE_DIRS "include")
include_directories(${catkin_INCLUDE_DIRS} ${NXT_QT_INCLUDE_DIRS})

catkin_package()

# Add the appropriate components to the component list here
rosbuild_prepare_qt4(
    QtCore
    QtGui
) 

file(GLOB QT_FORMS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ui/*.ui)
file(GLOB QT_RESOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} resources/*.qrc)
file(GLOB_RECURSE QT_MOC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS include/nxt_qt/*.hpp)

QT4_ADD_RESOURCES(QT_RESOURCES_CPP ${QT_RESOURCES})
QT4_WRAP_UI(QT_FORMS_HPP ${QT_FORMS})
QT4_WRAP_CPP(QT_MOC_HPP ${QT_MOC})

#source files
file(GLOB_RECURSE QT_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/*.cpp)

add_executable(QPIDControl ${QT_SOURCES} ${QT_RESOURCES_CPP} ${QT_FORMS_HPP} ${QT_MOC_HPP})
target_link_libraries(QPIDControl ${QT_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(QPIDControl nxt_beagle_generate_messages_cpp)
install(TARGETS QPIDControl RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

