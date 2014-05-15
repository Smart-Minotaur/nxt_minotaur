#ifndef QMOUSENODE_H
#define QMOUSENODE_H

#include "ros/ros.h"
#include "nxt_beagle/DebugMouseData.h"
#include <QThread>
#include <QMetaType>


//namespace minotaur {

class QMouseNode : public QThread
{
	Q_OBJECT
	
private:
	ros::NodeHandle nodeHandle;
	ros::Subscriber sub;
	
	void mouseDebugCallback(const nxt_beagle::DebugMouseData& mouse);

public:
	QMouseNode();
	virtual ~QMouseNode() { }

	void run();

Q_SIGNALS:
    void sendMouseMovement(double x, double y);
};


//}
#endif
