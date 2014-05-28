#include "nxt_beagle/MouseMonitorNodeBeagle.hpp"

#include <iostream>

namespace minotaur
{

    MouseMonitorNodeBeagle::MouseMonitorNodeBeagle() :
        sensor1(new pln_minotaur::PLN2033(SENSOR1)),
        sensor2(new pln_minotaur::PLN2033(SENSOR2))
    {
        serviceData = nodeHandle.advertiseService("getSensorData", &MouseMonitorNodeBeagle::sendData, this);
        serviceSettings = nodeHandle.advertiseService("getSensorSettings", &MouseMonitorNodeBeagle::sendSettings, this);

        // TODO
        sensor1->setXResolution(100);
        sensor1->setYResolution(100);
        sensor2->setXResolution(100);
        sensor2->setYResolution(100);
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
        nxt_beagle::MouseMonitorSensorGetData::Request &req,
        nxt_beagle::MouseMonitorSensorGetData::Response &res)
    {
        if (req.id == SENSOR1) {
            res.data = getData(SENSOR1, sensor1);
            std::cout << "SENSOR1: " << res.data.x_disp << " " << res.data.y_disp << std::endl;
        } else if (req.id == SENSOR2) {
            res.data = getData(SENSOR2, sensor2);
            std::cout << "SENSOR2: " << res.data.x_disp << " " << res.data.y_disp << std::endl;
        } else
            return false;

        return true;
    }

    bool MouseMonitorNodeBeagle::sendSettings(
        nxt_beagle::MouseMonitorSensorGetSettings::Request &req,
        nxt_beagle::MouseMonitorSensorGetSettings::Response &res)
    {
        if (req.id == SENSOR1)
            res.settings = getSettings(sensor1);
        else if (req.id == SENSOR2)
            res.settings = getSettings(sensor2);
        else
            return false;

        return true;
    }

    nxt_beagle::MouseMonitorSensorData MouseMonitorNodeBeagle::getData(
        std::string id,
        pln_minotaur::IPLNTrackingDevice *sensor)
    {
        nxt_beagle::MouseMonitorSensorData data;
        double xs, ys; // Speed
        double xd, yd; // Displacement

        // TODO: Here is a bug.
        //pln_minotaur::PLN2033_Settings s = sensor->readPLNSettings();
        //data.id = sensor1->readPLNSettings().spiDevice;
        data.id = id;

        if (sensor->readStatusAndDisplacementAndSpeed(xs, ys, xd, yd)) {
            data.x_speed = xs;
            data.y_speed = ys;
            data.x_disp = xd;
            data.y_disp = yd;
        } else {
            std::cout << "NO MOVEMENT" << std::endl;
            data.id = "";
        }

        return data;
    }

    nxt_beagle::MouseMonitorSensorSettings MouseMonitorNodeBeagle::getSettings(
        pln_minotaur::IPLNTrackingDevice *sensor)
    {
        nxt_beagle::MouseMonitorSensorSettings settingsMsg;
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
