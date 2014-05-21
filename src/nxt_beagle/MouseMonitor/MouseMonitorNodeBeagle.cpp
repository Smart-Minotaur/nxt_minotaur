#include "nxt_beagle/MouseMonitorNodeBeagle.hpp"

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

    void MouseMonitorNodeBeagle::publishSettings(std::string sensor)
    {
        pln_minotaur::PLN2033_Settings settings;

        if (sensor == SENSOR1) {
            settings = sensor1->readPLNSettings();
        } else if (sensor == SENSOR2) {
            settings = sensor1->readPLNSettings();
        }

        nxt_beagle::MouseMonitorSensorSettings settingsMsg;

        settingsMsg.id = sensor;
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

        pubMouseSettings.publish(settingsMsg);
    }

}
