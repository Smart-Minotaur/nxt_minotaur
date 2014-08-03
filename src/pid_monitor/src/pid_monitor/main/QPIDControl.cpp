#include <QtGui>
#include <QApplication>
#include "pid_monitor/PIDWindow.hpp"
#include "minotaur_common_qt/MetaTypes.hpp"
#include "minotaur_common_qt/QMinotaurControlThread.hpp"

#define NODE_NAME "QPIDControl"
#define INTERVAL 25

int main(int argc, char **argv)
{
    ros::init(argc, argv, NODE_NAME);
    if(!ros::master::check())
    {
        ROS_ERROR("Roscore has to be started.");
        return -1;
    }
    
    minotaur::registerMetatypes();
    QApplication app(argc, argv);
    minotaur::PIDWindow w;
    minotaur::QMinotaurControlThread thread(w.getControlNode());
    w.show();
    thread.start();
    
    app.connect(&app, SIGNAL(lastWindowClosed()), &app, SLOT(quit()));
    int result = app.exec();
    thread.stop();
    thread.wait();
    ros::shutdown();
    return result;
}
