#include "mouse_monitor_beagle/MouseMonitorNodeBeagle.hpp"

#include <iostream>

namespace minotaur
{

	MouseMonitorNodeBeagle::MouseMonitorNodeBeagle() :
		sensor1(new pln_minotaur::PLN2033(SENSOR1)),
		sensor2(new pln_minotaur::PLN2033(SENSOR2)),
		sensor1Data(SENSOR1),
		sensor2Data(SENSOR2),
		sampleFrequency(SENSOR_SAMPLE_FREQUENCY),
		debug(DEBUG_ENABLE) // TODO
	{
		serviceData = nodeHandle.advertiseService("getSensorData", &MouseMonitorNodeBeagle::sendData, this);
		serviceSettings = nodeHandle.advertiseService("getSensorSettings", &MouseMonitorNodeBeagle::sendSettings, this);
		serviceSetResolution = nodeHandle.advertiseService("setResolution", &MouseMonitorNodeBeagle::setResolution, this);
	}

	MouseMonitorNodeBeagle::~MouseMonitorNodeBeagle()
	{
		delete sensor1;
		delete sensor2;
	}

	void MouseMonitorNodeBeagle::run()
	{
		ros::Rate r(sampleFrequency);

		while (true) {
			processSensorData(sensor1, &sensor1Data);
			processSensorData(sensor2, &sensor2Data);

			ros::spinOnce();
			r.sleep();
		}
	}

	void MouseMonitorNodeBeagle::processSensorData(
	    pln_minotaur::IPLNTrackingDevice *sensor,
	    ProcessedSensorData *data)
	{
		double xs, ys; // Speed
		double xd, yd; // Displacement

		// TODO: Check lift bit

		if (sensor->readStatusAndDisplacementAndSpeed(xs, ys, xd, yd)) {
			data->xDisplacement += xd;
			data->yDisplacement += yd;
			data->xSpeed = xs;
			data->ySpeed = ys;
		} else {

		}
	}

	bool MouseMonitorNodeBeagle::sendData(
	    mouse_monitor_beagle::MouseMonitorSensorGetData::Request &req,
	    mouse_monitor_beagle::MouseMonitorSensorGetData::Response &res)
	{
		if (req.id == SENSOR1) {
			res.data = getData(&sensor1Data);
		} else if (req.id == SENSOR2) {
			res.data = getData(&sensor2Data);
		} else
			return false;

		return true;
	}

	bool MouseMonitorNodeBeagle::sendSettings(
	    mouse_monitor_beagle::MouseMonitorSensorGetSettings::Request &req,
	    mouse_monitor_beagle::MouseMonitorSensorGetSettings::Response &res)
	{
		if (req.id == SENSOR1)
			res.settings = getSettings(sensor1);
		else if (req.id == SENSOR2)
			res.settings = getSettings(sensor2);
		else
			return false;

		return true;
	}

	bool MouseMonitorNodeBeagle::setResolution(mouse_monitor_beagle::MouseMonitorSensorSetResolution::Request &req,
	        mouse_monitor_beagle::MouseMonitorSensorSetResolution::Response &res)
	{
		if (req.id == SENSOR1) {
			sensor1->setXResolution(req.newResolution);
			sensor1->setYResolution(req.newResolution);
		} else if (req.id == SENSOR2) {
			sensor2->setXResolution(req.newResolution);
			sensor2->setYResolution(req.newResolution);
		} else
			return false;
			
		if (debug)
			std::cout << req.id << " " << "resolution: " << req.newResolution << std::endl;

		return true;
	}

	mouse_monitor_beagle::MouseMonitorSensorData MouseMonitorNodeBeagle::getData(
	    ProcessedSensorData *data)
	{
		mouse_monitor_beagle::MouseMonitorSensorData dataMsg;

		dataMsg.id = data->spiDevice;
		dataMsg.x_disp = data->xDisplacement;
		dataMsg.y_disp = data->yDisplacement;
		dataMsg.x_speed = data->xSpeed;
		dataMsg.y_speed = data->ySpeed;
		
		if (debug)
			std::cout << dataMsg.id << " X disp: " << dataMsg.x_disp << ", " << dataMsg.y_disp << std::endl;

		data->reset();

		return dataMsg;
	}

	mouse_monitor_beagle::MouseMonitorSensorSettings MouseMonitorNodeBeagle::getSettings(
	    pln_minotaur::IPLNTrackingDevice *sensor)
	{
		mouse_monitor_beagle::MouseMonitorSensorSettings settingsMsg;
		pln_minotaur::PLN2033_Settings settings;

		settings = sensor->readPLNSettings();

		settingsMsg.spiDevice = settings.spiDevice;

		settingsMsg.status_register = sensor->readPLNStatusRegister;
		// Unused user registers
		settingsMsg.delta_x_disp_register = 0; // Unused
		settingsMsg.delta_y_disp_register = 0; // Unused

		settingsMsg.command_high_register = settings.command_high_register;
		settingsMsg.command_low_register = settings.command_low_register;
		settingsMsg.memory_pointer_register = settings.memory_pointer_register;
		settingsMsg.memory_data_register = settings.memory_data_register;
		settingsMsg.mode_control_register = settings.mode_control_register;
		settingsMsg.power_control_register = settings.power_control_register;
		settingsMsg.mode_status_register = settings.mode_status_register;
		settingsMsg.system_control_register = settings.system_control_register;
		settingsMsg.miscellaneous_register = settings.miscellaneous_register;
		settingsMsg.interrupt_output_register = settings.interrupt_output_register;
		
		if (debug)
			std::cout << settings.toSring();

		return settingsMsg;
	}

}
