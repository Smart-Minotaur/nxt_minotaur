#include <QtGui>
#include <QApplication>
#include <ros/ros.h>

#include "nxt_qt/MouseMonitorWindow.hpp"
#include "nxt_beagle/MouseMonitorConfig.hpp"
#include "PLN2033_Settings.h"


#include <exception>
#include <iostream>

int main(int argc, char **argv)
{
    ros::init(argc, argv, NODE_NAME_QT);
    if (!ros::master::check()) {
        ROS_ERROR("Roscore has to be started.");
        return -1;
    }

    qRegisterMetaType<minotaur::MouseData>("MouseData");
    qRegisterMetaType<pln_minotaur::PLN2033_Settings>("PLN2033_Settings");

    QApplication app(argc, argv);

    minotaur::MouseMonitorWindow window;

    window.show();
    window.getMonitorNode().start();

    app.connect(&app, SIGNAL(lastWindowClosed()), &app, SLOT(quit()));
    int result = app.exec();
    ros::shutdown();

    return result;
}
