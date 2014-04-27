#include "nxt_qt/mainwindow.h"
#include <QApplication>
#include <QtGui>
#include <QMetaType>
#include "ros/ros.h"
#include <QtGui/QMainWindow>
#include "ui_mainwindow_debug.h"
#include "QWidget"
#include <QLine>
#include "nxt_qt/QMouseNode.h"

#define NODE_NAME "MOUSEGUI"
#define INTERVAL 25



int main(int argc, char *argv[])
{
    ros::init(argc, argv, NODE_NAME);
	if(!ros::master::check())
    {
        ROS_ERROR("Roscore has to be started.");
        return -1;
    }

    //qRegisterMetaType<MainWindow::MainWindow>("QDebugMouse");
    QApplication a(argc, argv);
    MainWindow w;

    QTimer::singleShot(1000, &w, SLOT(setInitIntervall()));

    w.show();
    w.getMouseNode().start();

    a.connect(&a, SIGNAL(lastWindowClosed()), &a, SLOT(quit()));
	
    int result = a.exec();
    ros::shutdown();
	
    return result;
}




