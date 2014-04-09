#include <QtGui>
#include <QApplication>
#include "nxt_qt/PIDWindow.hpp"

#define NODE_NAME "QPIDControl"
#define INTERVAL 25

int main(int argc, char **argv)
{
    ros::init(argc, argv, NODE_NAME);
    if(!ros::master::check())
    {
        ROS_ERROR("Roscore hast to be started.");
        return -1;
    }
    
    qRegisterMetaType<minotaur::QMotorVelocity>("QMotorVelocity");
    QApplication app(argc, argv);
    minotaur::PIDWindow w;
    
    QTimer::singleShot(1000, &w, SLOT(setInitModel()));
    
    w.show();
    w.getPIDNode().start();
    
    app.connect(&app, SIGNAL(lastWindowClosed()), &app, SLOT(quit()));
    int result = app.exec();
    ros::shutdown();
    return result;
}
