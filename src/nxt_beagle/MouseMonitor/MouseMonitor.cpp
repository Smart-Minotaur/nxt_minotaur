#include "MouseMonitorNodeBeagle.hpp"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "MouseMonitor");
	if (!ros::master::check()) {
		ROS_ERROR("Roscore has to be started.");
		return -1;
	}

	MouseMonitorNodeBeagle node;

	node.run();
	ros::shutdown();

	return 0;
}
