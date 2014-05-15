#include "nxt_qt/QMouseNode.h"
#include "nxt_qt/DebugMouseData.h"
//#include "nxt_qt/mainwindow.h"

//namespace minotaur {

QMouseNode::QMouseNode()
{
	ROS_INFO("Subscribing to topic 'mouseDebug'");
	sub = nodeHandle.subscribe("mouseDebug", 1000, &QMouseNode::mouseDebugCallback, this);
}


void QMouseNode::run()
{
	ROS_INFO("Inside run. Going to spin");
	ros::spin();
}


void QMouseNode::mouseDebugCallback(const nxt_qt::DebugMouseData& mouse)
{
	//ROS_INFO("MouseNode heard: [X: %d, Y: %d]", mouse.MouseXValue, mouse.MouseYValue);
	Q_EMIT sendMouseMovement(mouse.mouse1_x_disp, mouse.mouse1_y_disp);
}

//}

