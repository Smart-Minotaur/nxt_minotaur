#include "nxt_beagle/MouseMonitorNodeBeagle.hpp"

#include <iostream>

namespace minotaur
{

    MouseMonitorNodeBeagle::MouseMonitorNodeBeagle() :
        sensor1(new pln_minotaur::PLN2033(SENSOR1)),
        sensor2(new pln_minotaur::PLN2033(SENSOR2))
    {
        /*pubData = nodeHandle.advertise<nxt_beagle::MouseMonitorSensorData>(
                      ROS_MOUSE_DATA_TOPIC, 100);
        pubSettings = nodeHandle.advertise<nxt_beagle::MouseMonitorSensorSettings>(
                          ROS_MOUSE_SETTINGS_TOPIC, 100);*/

        serviceData = nodeHandle.advertiseService("getSensorData", &MouseMonitorNodeBeagle::sendData, this);
        serviceSettings = nodeHandle.advertiseService("getSensorSettings", &MouseMonitorNodeBeagle::sendSettings, this);

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

        /*
        ros::Rate loop_rate(10);
        while (ros::ok()) {
            publishData(sensor1);
            publishData(sensor2);

            publishSettings(sensor1);
            publishSettings(sensor2);

            ros::spinOnce();
            loop_rate.sleep();
        }
        */
    }

    bool MouseMonitorNodeBeagle::sendData(
        nxt_beagle::MouseMonitorSensorGetData::Request &req,
        nxt_beagle::MouseMonitorSensorGetData::Response &res)
    {
        std::cout << "Get Data" << std::endl;
        if (req.id == SENSOR1) {
            res.data = getData(sensor1);
            std::cout << "SENSOR1: " << res.data.x_disp << " " << res.data.y_disp << std::endl;
        } else if (req.id == SENSOR2) {
            res.data = getData(sensor2);
            std::cout << "SENSOR1: " << res.data.x_disp << " " << res.data.y_disp << std::endl;
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
        pln_minotaur::IPLNTrackingDevice *sensor)
    {
        nxt_beagle::MouseMonitorSensorData data;

        data.id = sensor->readPLNSettings().spiDevice;

        if (!sensor->readStatusAndDisplacementAndSpeed(
                    data.x_speed,
                    data.y_speed,
                    data.x_disp,
                    data.y_disp)) {
            data.id = "";
        }

        return data;
    }

    void MouseMonitorNodeBeagle::publishData(pln_minotaur::IPLNTrackingDevice *sensor)
    {
        nxt_beagle::MouseMonitorSensorData data = getData(sensor);

        pubData.publish(data);
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

    void MouseMonitorNodeBeagle::publishSettings(pln_minotaur::IPLNTrackingDevice *sensor)
    {
        nxt_beagle::MouseMonitorSensorSettings settingsMsg = getSettings(sensor);

        pubSettings.publish(settingsMsg);
    }

}
