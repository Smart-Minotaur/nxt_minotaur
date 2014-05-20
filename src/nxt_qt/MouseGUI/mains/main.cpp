#include "nxt_qt/mainwindow.h"
#include "ros/ros.h"

#include <QApplication>
#include <QtGui>
#include <QMetaType>
#include <QtGui/QMainWindow>
#include <QWidget>
#include <QLine>

#define NODE_NAME "MOUSEGUI"

int main(int argc, char *argv[])
{
	ros::init(argc, argv, NODE_NAME);

	if (!ros::master::check()) {
		ROS_ERROR("Roscore has to be started.");
		return -1;
	}

    //qRegisterMetaType<MainWindow::MainWindow>("QDebugMouse");
   	QApplication app(argc, argv);
 	minotaur::MainWindow window;

	window.show();
	window.getMouseNode().start();

	app.connect(&app, SIGNAL(lastWindowClosed()), &app, SLOT(quit()));
	
	int result = app.exec();
	ros::shutdown();
	
   	return result;
}
