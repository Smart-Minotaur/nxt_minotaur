#include "nxt_qt/MouseMonitorNode.hpp"
//#include "nxt_beagle/MouseMonitorNodeBeagle.hpp" // Because of topic name

#define ROS_MOUSE_DATA_TOPIC "/mouseData"

namespace minotaur
{

	MouseMonitorNode::MouseMonitorNode()
	{
		ROS_INFO("Subscribing to topic \"%s\"", ROS_MOUSE_DATA_TOPIC);

		sensorData = nodeHandle.subscribe(ROS_MOUSE_DATA_TOPIC,
			100,
			&MouseMonitorNode::processMouseDataMessage,
			this);
	}

        MouseMonitorNode::~MouseMonitorNode() {}

	void MouseMonitorNode::run()
	{
		ros::spin();
		Q_EMIT rosShutdown();
	}

	void MouseMonitorNode::processMouseDataMessage(
		const nxt_beagle::MouseMonitorSensorData& msg)
	{
		MouseData data;
		
		data.id = msg.id;
		data.x_disp = msg.x_disp;
		data.y_disp = msg.y_disp;
		data.x_speed = msg.x_speed;
		data.y_speed = msg.y_speed;

		Q_EMIT measuredMouseData(data);
	}

	void MouseMonitorNode::processMouseSettingsMessage(
		const nxt_beagle::MouseMonitorSensorSettings& msg)
        {

        }

}
