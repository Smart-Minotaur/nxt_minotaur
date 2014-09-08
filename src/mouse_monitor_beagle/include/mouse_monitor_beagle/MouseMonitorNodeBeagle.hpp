#ifndef MOUSE_MONITOR_NODE_BEAGLE_H
#define MOUSE_MONITOR_NODE_BEAGLE_H

#include <ros/ros.h>
#include "mouse_monitor_beagle/MouseMonitorConfig.hpp"

// PLN2033 software
#include "IPLNTrackingDevice.h"
#include "PLN2033.h"
#include "PLN2033_Settings.h"

// Outputs debug messages if true
#define DEBUG_ENABLE 0

namespace minotaur
{

	/**
	 * Sums up the sensor data.
	 */
	struct ProcessedSensorData {
		std::string spiDevice;

		double xDisplacement;
		double yDisplacement;

		double xSpeed; //< Last x speed value
		double ySpeed; //< Last y speed value

		int liftBitErrors;
		int otherErros;

		ProcessedSensorData(std::string spiDevice) :
			spiDevice(spiDevice) {
			reset();
		}

		void reset() {
			xDisplacement = 0.0;
			yDisplacement = 0.0;
			xSpeed = 0.0;
			ySpeed = 0.0;
			liftBitErrors = 0;
			otherErros = 0;
		}
	};

	class MouseMonitorNodeBeagle
	{
		private:
			ros::NodeHandle nodeHandle;
			ros::ServiceServer serviceData;
			ros::ServiceServer serviceSettings;
			ros::ServiceServer serviceSetResolution;

			pln_minotaur::IPLNTrackingDevice *sensor1;
			pln_minotaur::IPLNTrackingDevice *sensor2;

			ProcessedSensorData sensor1Data;
			ProcessedSensorData sensor2Data;

			/**
			 * Sample frequency for the sensors.
			 */
			double sampleFrequency;

			/**
			 * If debug output is enabled.
			 */
			bool debug;

			/**
			 * @name Triggered from PC.
			 */
			///@{
			bool sendData(mouse_monitor_beagle::MouseMonitorSensorGetData::Request &req,
			              mouse_monitor_beagle::MouseMonitorSensorGetData::Response &res);
			bool sendSettings(mouse_monitor_beagle::MouseMonitorSensorGetSettings::Request &req,
			                  mouse_monitor_beagle::MouseMonitorSensorGetSettings::Response &res);
			bool setResolution(mouse_monitor_beagle::MouseMonitorSensorSetResolution::Request &req,
			                   mouse_monitor_beagle::MouseMonitorSensorSetResolution::Response &res);
			///@}

			/**
			 * @name Converting
			 *
			 * Converts sensor data to ros messages.
			 */
			///@{
			mouse_monitor_beagle::MouseMonitorSensorSettings getSettings(
			    pln_minotaur::IPLNTrackingDevice *sensor);

			mouse_monitor_beagle::MouseMonitorSensorData getData(
			    ProcessedSensorData *data);
			///@}

			/**
			 * Reads sensor data and saves it.
			 *
			 * Displacement values are summed up. Lift bit is checked.
			 */
			void processSensorData(
			    pln_minotaur::IPLNTrackingDevice *sensor,
			    ProcessedSensorData *data);

		public:
			MouseMonitorNodeBeagle();
			virtual ~MouseMonitorNodeBeagle();

			void run();
	};

}

#endif
