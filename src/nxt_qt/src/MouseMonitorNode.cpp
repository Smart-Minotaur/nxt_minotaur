#include "nxt_qt/MouseMonitorNode.hpp"
#include "nxt_beagle/MouseMonitorConfig.hpp"
#include "PLN2033_Settings.h"

namespace minotaur
{

    MouseMonitorNode::MouseMonitorNode()
    {
        ROS_INFO("Subscribing to topic \"%s\"", ROS_MOUSE_DATA_TOPIC);

        sensorData = nodeHandle.subscribe(ROS_MOUSE_DATA_TOPIC,
                                          100,
                                          &MouseMonitorNode::processMouseDataMessage,
                                          this);

        sensorSettings = nodeHandle.subscribe(ROS_MOUSE_SETTINGS_TOPIC,
                                              100,
                                              &MouseMonitorNode::processMouseSettingsMessage,
                                              this);
    }

    MouseMonitorNode::~MouseMonitorNode() {}

    void MouseMonitorNode::run()
    {
        ros::spin();
        Q_EMIT rosShutdown();
    }

    void MouseMonitorNode::processMouseDataMessage(
        const nxt_beagle::MouseMonitorSensorData& msg)
    {
        MouseData data;

        data.id = msg.id;
        data.x_disp = msg.x_disp;
        data.y_disp = msg.y_disp;
        data.x_speed = msg.x_speed;
        data.y_speed = msg.y_speed;

        Q_EMIT measuredMouseData(data);
    }

    void MouseMonitorNode::processMouseSettingsMessage(
        const nxt_beagle::MouseMonitorSensorSettings& settingsMsg)
    {
        pln_minotaur::PLN2033_Settings settings;

        settings.status_register = settingsMsg.status_register;
        settings.delta_x_disp_register = settingsMsg.delta_x_disp_register; // TODO
        settings.delta_y_disp_register = settingsMsg.delta_y_disp_register; // TODO
        settings.command_high_register = settingsMsg.command_high_register;
        settings.command_low_register = settingsMsg.command_low_register;
        settings.memory_pointer_register = settingsMsg.memory_pointer_register;
        settings.memory_data_register = settingsMsg.memory_data_register;
        settings.mode_control_register = settingsMsg.mode_control_register;
        settings.power_control_register = settingsMsg.power_control_register;
        settings.mode_status_register = settingsMsg.mode_status_register;
        settings.system_control_register = settingsMsg.system_control_register;
        settings.miscellaneous_register = settingsMsg.miscellaneous_register;
        settings.interrupt_output_register = settingsMsg.interrupt_output_register;

        Q_EMIT measuredMouseSettings(settingsMsg.id, settings);
    }

}
