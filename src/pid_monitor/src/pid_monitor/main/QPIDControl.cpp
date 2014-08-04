#include <QtGui>
#include <QApplication>
#include <signal.h>
#include "pid_monitor/PIDWindow.hpp"
#include "minotaur_common_qt/MetaTypes.hpp"
#include "minotaur_common_qt/QMinotaurControlThread.hpp"

#define NODE_NAME "QPIDControl"
#define INTERVAL 25

static QApplication *app;

static void signalHandler(int sig)
{
    app->quit();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, NODE_NAME);
    if(!ros::master::check())
    {
        ROS_ERROR("Roscore has to be started.");
        return -1;
    }
    
    minotaur::registerMetatypes();
    QApplication a(argc, argv);
    app = &a;
    minotaur::PIDWindow w;
    ros::NodeHandle handle;
    w.getControlNode().connectToROS(handle);
    minotaur::QMinotaurControlThread thread(w.getControlNode());
    w.show();
    thread.start();
    
    app->connect(app, SIGNAL(lastWindowClosed()), app, SLOT(quit()));
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);
    int result = app->exec();
    ROS_INFO("Shutting down.");
    thread.stop();
    thread.wait();
    ros::shutdown();
    return result;
}
