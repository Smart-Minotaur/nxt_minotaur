#include "MouseMonitorNodeBeagle.hpp"

#define SENSOR1 "/dev/spidev1.0"
#define SENSOR2 "/dev/spidev1.1"

namespace minotaur
{

	MouseMonitorNodeBeagle::MouseMonitorNodeBeagle() :
		sensor1(new pln_minotaur::PLN2033(SENSOR1)),
		sensor2(new pln_minotaur::PLN2033(SENSOR2))
	{
		pubMouseData = nodeHandle.advertise<nxt_beagle::MouseMonitorSensorData>(
			ROS_MOUSE_DATA_TOPIC, 100);
		pubMouseSettings = nodeHandle.advertise<nxt_beagle::MouseMonitorSensorSettings>(
			ROS_MOUSE_SETTINGS_TOPIC, 100);
	}

	MouseMonitorNodeBeagle::~MouseMonitorNodeBeagle()
	{
		delete sensor1;
		delete sensor2;
	}

	void MouseMonitorNodeBeagle::run()
	{
		ros::Rate loop_rate(100);

		while (ros::ok()) {
			nxt_beagle::MouseMonitorSensorData data_s1;
			nxt_beagle::MouseMonitorSensorData data_s2;

			if (sensor1->readStatusAndDisplacementAndSpeed(
				data_s1.x_speed,
				data_s1.y_speed,
				data_s1.x_disp,
				data_s1.y_disp)) {

				data_s1.id = SENSOR1;

				pubMouseData.publish(data_s1);
			}

			if (sensor2->readStatusAndDisplacementAndSpeed(
				data_s2.x_speed,
				data_s2.y_speed,
				data_s2.x_disp,
				data_s2.y_disp)) {

				data_s2.id = SENSOR2;

				pubMouseData.publish(data_s2);
			}

			ros::spinOnce();
			loop_rate.sleep();
		}
	}

}
