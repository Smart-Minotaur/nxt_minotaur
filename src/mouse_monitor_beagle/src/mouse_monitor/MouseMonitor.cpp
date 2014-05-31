#include "mouse_monitor_beagle/MouseMonitorNodeBeagle.hpp"
#include "mouse_monitor_beagle/MouseMonitorConfig.hpp"

int main(int argc, char **argv)
{
	ros::init(argc, argv, NODE_NAME_BEAGLE);
	if (!ros::master::check()) {
		ROS_ERROR("Roscore has to be started.");
		return -1;
	}

	minotaur::MouseMonitorNodeBeagle node;

	node.run();
	ros::shutdown();

	return 0;
}
