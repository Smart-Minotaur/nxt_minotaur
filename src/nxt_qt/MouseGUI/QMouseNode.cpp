#include "nxt_qt/QMouseNode.h"
#include "nxt_qt/DebugMouse.h"
//#include "nxt_qt/mainwindow.h"

//namespace minotaur {

QMouseNode::QMouseNode()
{
	ROS_INFO("QMouseNode.cpp: testest");
	ROS_INFO("Subscribing to topic 'chatter'");
	sub = nodeHandle.subscribe("chatter", 1000, &QMouseNode::chatterCallback, this);
}


void QMouseNode::run()
{
	ROS_INFO("Inside run. Going to spin");
	ros::spin();
}


void QMouseNode::chatterCallback(const nxt_qt::DebugMouse& mouse)
{
	ROS_INFO("MouseNode heard: [X: %d, Y: %d]", mouse.MouseXValue, mouse.MouseYValue);
	Q_EMIT sendMouseMovement(mouse.MouseXValue, mouse.MouseYValue);
}

//}

