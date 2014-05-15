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

file( GLOB NXT_QT_INCLUDE_DIRS "include" "/usr/local/qwt-6.0.1/include")
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
#neu
file(GLOB_RECURSE QT_MOC_DEBUG RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS include/nxt_qt/*.h)
#ende

QT4_ADD_RESOURCES(QT_RESOURCES_CPP ${QT_RESOURCES})
QT4_WRAP_UI(QT_FORMS_HPP ${QT_FORMS})
QT4_WRAP_CPP(QT_MOC_HPP ${QT_MOC})
#neu
QT4_WRAP_CPP(QT_MOC_DEBUG_H ${QT_MOC_DEBUG})
#ende

#source files
file(GLOB_RECURSE QT_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/*.cpp)

add_executable(QPIDControl ${QT_SOURCES} ${QT_RESOURCES_CPP} ${QT_FORMS_HPP} ${QT_MOC_HPP})
target_link_libraries(QPIDControl ${QT_LIBRARIES} ${catkin_LIBRARIES} qwt)
add_dependencies(QPIDControl nxt_beagle_generate_messages_cpp)
install(TARGETS QPIDControl RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

#neu
file(GLOB MOUSE_DEBUG_SRC ${CMAKE_CURRENT_SOURCE_DIR} MouseGUI/*.cpp)
file(GLOB MOUSE_DEBUG_MAIN MouseGUI/mains/*.cpp)
file(GLOB MINOTAUR_LIB ../../../trunk/build/*.a)
file(GLOB MINOTAUR ../../../trunk/src)
include_directories(${MINOTAUR})
link_directories(${MINOTAUR_LIB})

foreach(main_file ${MOUSE_DEBUG_MAIN})
	get_filename_component(main_name ${main_file} NAME_WE)
	add_executable(debug_${main_name} ${main_file} ${MOUSE_DEBUG_SRC} ${QT_RESOURCES_CPP} ${QT_FORMS_HPP} ${QT_MOC_DEBUG_H})
	target_link_libraries(debug_${main_name} ${QT_LIBRARIES} ${catkin_LIBRARIES} qwt ${MINOTAUR_LIB})
	add_dependencies(debug_${main_name} nxt_beagle_generate_messages_cpp)
endforeach(main_file ${MOUSE_DEBUG_MAIN})
#ende

