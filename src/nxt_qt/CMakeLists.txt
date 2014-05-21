cmake_minimum_required(VERSION 2.8.0)
project(nxt_qt)

## Generate messages in the 'msg' folder
#add_message_files(
#    FILES
#)

find_package(catkin REQUIRED COMPONENTS
    qt_build
    roscpp
    nxt_beagle
)

## Generate added messages and services with any dependencies listed here
#generate_messages(   
#    DEPENDENCIES
#    std_msgs
#)

# Only for the Settings struct.
set(PLN_MINOTAUR_SVN_DIRECTORY ../../../pln_minotaur)
file(GLOB PLN_MINOTAUR_LIB ${PLN_MINOTAUR_SVN_DIRECTORY}/trunk/build)
file(GLOB PLN_MINOTAUR_INCLUDE_DIRECTORY ${PLN_MINOTAUR_SVN_DIRECTORY}/trunk/src)
link_directories(${PLN_MINOTAUR_LIB})
include_directories(${PLN_MINOTAUR_INCLUDE_DIRECTORY})
set(PLN_MINOTAUR_LIB_NAME pln_minotaur_x86)

file(GLOB NXT_QT_INCLUDE_DIRS "include" "/usr/local/qwt-6.0.1/include")
include_directories(${catkin_INCLUDE_DIRS} ${NXT_QT_INCLUDE_DIRS} ${QT_USE_FILE})

link_directories("/usr/local/qwt-6.0.1/lib")

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

# Source files
file(GLOB QT_LIB_SOURCES ${CMAKE_CURRENT_SOURCE_DIR} src/*.cpp)
add_library(NxtQT ${QT_LIB_SOURCES} ${QT_RESOURCES_CPP} ${QT_FORMS_HPP} ${QT_MOC_HPP})
target_link_libraries(NxtQT ${catkin_LIBRARIES} ${QT_LIBRARIES} qwt ${PLN_MINOTAUR_LIB_NAME})
add_dependencies(NxtQT nxt_beagle_generate_messages_cpp)

file(GLOB EXEC_SOURCES src/main/*.cpp)
foreach(mainsourcefile ${EXEC_SOURCES})
    get_filename_component(mainname ${mainsourcefile} NAME_WE)
    add_executable(${mainname} ${mainsourcefile})
    target_link_libraries(${mainname} ${QT_LIBRARIES} ${catkin_LIBRARIES} NxtQT)
    add_dependencies(${mainname} nxt_beagle_generate_messages_cpp)
endforeach(mainsourcefile ${EXEC_SOURCES})
