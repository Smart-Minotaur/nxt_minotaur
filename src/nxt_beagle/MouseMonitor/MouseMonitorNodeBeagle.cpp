#include "nxt_beagle/MouseMonitorNodeBeagle.hpp"

namespace minotaur
{

    MouseMonitorNodeBeagle::MouseMonitorNodeBeagle() :
        sensor1(new pln_minotaur::PLN2033(SENSOR1)),
        sensor2(new pln_minotaur::PLN2033(SENSOR2))
    {
        pubData = nodeHandle.advertise<nxt_beagle::MouseMonitorSensorData>(
                           ROS_MOUSE_DATA_TOPIC, 100);
        pubSettings = nodeHandle.advertise<nxt_beagle::MouseMonitorSensorSettings>(
                               ROS_MOUSE_SETTINGS_TOPIC, 100);

        serviceData = nodeHandle.advertiseService("getSensorData", &MouseMonitorNodeBeagle::sendData, this);
        serviceSettings = nodeHandle.advertiseService("getSensorSettings", &MouseMonitorNodeBeagle::sendSettings, this);
    }

    MouseMonitorNodeBeagle::~MouseMonitorNodeBeagle()
    {
        delete sensor1;
        delete sensor2;
    }

    void MouseMonitorNodeBeagle::run()
    {
        ros::Rate loop_rate(100);

        //ros::spin();

        while (ros::ok()) {
            publishData(sensor1);
            publishData(sensor2);
            publishSettings(sensor1);
            publishSettings(sensor2);

            ros::spinOnce();
            loop_rate.sleep();
        }
    }

    bool sendData(nxt_beagle::MouseMonitorSensorGetData::Request  &req,
              nxt_beagle::MouseMonitorSensorGetData::Response &res)
    {

    }

    bool sendSettings(nxt_beagle::MouseMonitorSensorGetSettings::Request  &req,
              nxt_beagle::MouseMonitorSensorGetSettings::Response &res)
    {

    }

    void MouseMonitorNodeBeagle::publishData(pln_minotaur::IPLNTrackingDevice *sensor)
    {
        nxt_beagle::MouseMonitorSensorData data;
        pln_minotaur::IPLNTrackingDevice *tmp;

        data.id = sensor->readPLNSettings().spiDevice;

        if (sensor->readStatusAndDisplacementAndSpeed(
                    data.x_speed,
                    data.y_speed,
                    data.x_disp,
                    data.y_disp)) {
            pubData.publish(data);
        }
    }

    void MouseMonitorNodeBeagle::publishSettings(pln_minotaur::IPLNTrackingDevice *sensor)
    {
        nxt_beagle::MouseMonitorSensorSettings settingsMsg;
        pln_minotaur::PLN2033_Settings settings;

        settings = sensor->readPLNSettings();

        settingsMsg.id = settings.spiDevice;
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

        pubSettings.publish(settingsMsg);
    }

}
