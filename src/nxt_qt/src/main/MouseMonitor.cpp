#include <QtGui>
#include <QApplication>
#include <ros/ros.h>
#include "nxt_qt/MouseMonitorWindow.hpp"

#define NODE_NAME "MouseMonitor"

int main(int argc, char **argv)
{
    ros::init(argc, argv, NODE_NAME);
    if (!ros::master::check()) {
        ROS_ERROR("Roscore has to be started.");
        return -1;
    }
    
    QApplication app(argc, argv);
    minotaur::MouseMonitorWindow window;
    
    window.show();
    window.getMonitorNode().start();
    
    app.connect(&app, SIGNAL(lastWindowClosed()), &app, SLOT(quit()));
    int result = app.exec();
    ros::shutdown();

    return result;
}
