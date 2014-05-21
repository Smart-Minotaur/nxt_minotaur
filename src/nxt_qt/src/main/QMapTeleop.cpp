#include <QtGui>
#include <QApplication>
#include "nxt_qt/MapWindow.hpp"

#define NODE_NAME "QMapTeleop"

int main(int argc, char **argv)
{
    ros::init(argc, argv, NODE_NAME);
    if(!ros::master::check())
    {
        ROS_ERROR("Roscore has to be started.");
        return -1;
    }
    
    qRegisterMetaType<minotaur::QOdometry>("QOdometry");
    qRegisterMetaType<minotaur::QUltraSensor>("QUltraSensor");
    QApplication app(argc, argv);
    minotaur::MapWindow w;
    
    w.show();
    w.getNavigationNode().start();
    
    app.connect(&app, SIGNAL(lastWindowClosed()), &app, SLOT(quit()));
    int result = app.exec();
    ros::shutdown();
    return result;
}
