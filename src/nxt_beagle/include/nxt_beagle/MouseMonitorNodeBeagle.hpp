#ifndef MOUSE_MONITOR_NODE_BEAGLE_H
#define MOUSE_MONITOR_NODE_BEAGLE_H

#define ROS_MOUSE_DATA_TOPIC "/mouseData"
#define ROS_MOUSE_SETTINGS_TOPIC "/mouseSettings"

#include <ros/ros.h>
#include "nxt_beagle/MouseMonitorSensorData.h"
#include "nxt_beagle/MouseMonitorSensorSettings.h"
#include "IPLNTrackingDevice.h"
#include "PLN2033.h"

namespace minotaur
{

	class MouseMonitorNodeBeagle
	{
		private:
			ros::NodeHandle nodeHandle;
			ros::Publisher pubMouseData;
			ros::Publisher pubMouseSettings;

			pln_minotaur::IPLNTrackingDevice *sensor1;
			pln_minotaur::IPLNTrackingDevice *sensor2;

		public:
			MouseMonitorNodeBeagle();
			virtual ~MouseMonitorNodeBeagle();

			void run();
	};

}

#endif
