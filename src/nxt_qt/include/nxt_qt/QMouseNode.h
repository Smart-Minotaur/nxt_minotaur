#ifndef QMOUSENODE_H
#define QMOUSENODE_H

#include "ros/ros.h"
//#include "std_msgs/String.h"
//#include "nxt_beagle/Config.hpp"
#include "nxt_qt/DebugMouse.h"
#include <QThread>
#include <QMetaType>
//#include "nxt_qt/mainwindow.h"


//namespace minotaur {

class QMouseNode : public QThread
{
	Q_OBJECT
	
private:
	ros::NodeHandle nodeHandle;
	ros::Subscriber sub;
	
	void chatterCallback(const nxt_qt::DebugMouse& mouse);

public:
	QMouseNode();
	virtual ~QMouseNode() { }

	void run();

Q_SIGNALS:
    void sendMouseMovement(int x, int y);
};


//}
#endif
