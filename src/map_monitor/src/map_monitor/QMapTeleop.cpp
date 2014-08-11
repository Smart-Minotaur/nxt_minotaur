#include <QtGui>
#include <QApplication>
#include "map_monitor/MapWindow.hpp"
#include "minotaur_common_qt/QMinotaurControlThread.hpp"

#define NODE_NAME "QMapTeleop"

int main(int argc, char **argv)
{
    ros::init(argc, argv, NODE_NAME);
    if(!ros::master::check()) {
        ROS_ERROR("Roscore has to be started.");
        return -1;
    }
    ros::NodeHandle handle;
    
    minotaur::registerMetatypes();
    QApplication app(argc, argv);
    minotaur::MapWindow w;
    
    w.show();
    w.getControlNode().connectToROS(handle);
    minotaur::QMinotaurControlThread thread(w.getControlNode());
    thread.start();
    
    app.connect(&app, SIGNAL(lastWindowClosed()), &app, SLOT(quit()));
    int result = app.exec();
    
    thread.stop();
    thread.wait();
    
    ros::shutdown();
    return result;
}
