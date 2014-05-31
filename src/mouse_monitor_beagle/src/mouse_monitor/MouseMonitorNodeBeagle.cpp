#include "mouse_monitor_beagle/MouseMonitorNodeBeagle.hpp"

#include <iostream>

namespace minotaur
{

    MouseMonitorNodeBeagle::MouseMonitorNodeBeagle() :
        sensor1(new pln_minotaur::PLN2033(SENSOR1)),
        sensor2(new pln_minotaur::PLN2033(SENSOR2))
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
        ros::spin();
    }

    bool MouseMonitorNodeBeagle::sendData(
        mouse_monitor_beagle::MouseMonitorSensorGetData::Request &req,
        mouse_monitor_beagle::MouseMonitorSensorGetData::Response &res)
    {
        if (req.id == SENSOR1) {
            res.data = getData(SENSOR1, sensor1);
        } else if (req.id == SENSOR2) {
            res.data = getData(SENSOR2, sensor2);
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

        return true;
    }

    mouse_monitor_beagle::MouseMonitorSensorData MouseMonitorNodeBeagle::getData(
        std::string id,
        pln_minotaur::IPLNTrackingDevice *sensor)
    {
        mouse_monitor_beagle::MouseMonitorSensorData data;
        double xs, ys; // Speed
        double xd, yd; // Displacement

        // TODO: Here is a bug.
        //pln_minotaur::PLN2033_Settings s = sensor->readPLNSettings();
        //data.id = sensor1->readPLNSettings().spiDevice;

        if (sensor->readStatusAndDisplacementAndSpeed(xs, ys, xd, yd)) {
            data.id = id;
            data.x_speed = xs;
            data.y_speed = ys;
            data.x_disp = xd;
            data.y_disp = yd;
        } else
            data.id = "";

        return data;
    }

    mouse_monitor_beagle::MouseMonitorSensorSettings MouseMonitorNodeBeagle::getSettings(
        pln_minotaur::IPLNTrackingDevice *sensor)
    {
        mouse_monitor_beagle::MouseMonitorSensorSettings settingsMsg;
        pln_minotaur::PLN2033_Settings settings;

        settings = sensor->readPLNSettings();

        settingsMsg.spiDevice = settings.spiDevice;
        settingsMsg.status_register = settings.status_register;
        settingsMsg.delta_x_disp_register = 0; // TODO
        settingsMsg.delta_y_disp_register = 0; // TODO
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

        return settingsMsg;
    }

}
