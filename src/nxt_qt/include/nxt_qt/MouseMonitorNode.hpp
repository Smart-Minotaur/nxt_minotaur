#ifndef MOUSE_MONITOR_NODE_H
#define MOUSE_MONITOR_NODE_H

#include <ros/ros.h>
#include <QThread>
#include <QMetaType>
#include "nxt_beagle/MouseMonitorSensorData.h"
#include "nxt_beagle/MouseMonitorSensorSettings.h"

namespace minotaur
{
	
	struct MouseData
	{
		std::string id;
		double x_disp;
		double y_disp;
		double x_speed;
		double y_speed;
	};

	class MouseMonitorNode : public QThread
	{
		Q_OBJECT

		private:
			ros::NodeHandle nodeHandle;

			ros::Subscriber sensorData;
			ros::Subscriber sensorSettings;

		void processMouseDataMessage(
			const nxt_beagle::MouseMonitorSensorData& msg);
        	
		void processMouseSettingsMessage(
                	const nxt_beagle::MouseMonitorSensorSettings& msg);

		public:
			MouseMonitorNode();
			virtual ~MouseMonitorNode();

			void run();

		Q_SIGNALS:
			void rosShutdown();
			void measuredMouseData(const MouseData data);
			//void measuredMouseSettings();	 // TODO: Settings class
	};

}
	
	Q_DECLARE_METATYPE(minotaur::MouseData);
	//Q_DECLARE_METATYPE(minotaur::QUltraSensor); // TODO: From the SPI lib

#endif
