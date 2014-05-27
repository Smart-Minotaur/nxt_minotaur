#include "nxt_qt/MouseMonitorNode.hpp"
#include "nxt_beagle/MouseMonitorConfig.hpp"
#include "PLN2033_Settings.h"

namespace minotaur
{

    MouseMonitorNode::MouseMonitorNode()
    {
        ROS_INFO("Subscribing to topic \"%s\"", ROS_MOUSE_DATA_TOPIC);

        subData = nodeHandle.subscribe(ROS_MOUSE_DATA_TOPIC,
                                       100,
                                       &MouseMonitorNode::processMouseDataMessage,
                                       this);

        subSettings = nodeHandle.subscribe(ROS_MOUSE_SETTINGS_TOPIC,
                                           100,
                                           &MouseMonitorNode::processMouseSettingsMessage,
                                           this);

        serviceData = nodeHandle.serviceClient<nxt_beagle::MouseMonitorSensorGetData>("getSensorData");
        serviceSettings= nodeHandle.serviceClient<nxt_beagle::MouseMonitorSensorGetSettings>("getSensorSettings");
    }

    MouseMonitorNode::~MouseMonitorNode() {}

    void MouseMonitorNode::run()
    {
        ros::spin();
        Q_EMIT rosShutdown();
    }

    MouseData MouseMonitorNode::getMouseData(std::string id)
    {
        nxt_beagle::MouseMonitorSensorGetData srv;
        srv.request.id = id;

        MouseData data;

        if (serviceData.call(srv))
            data = convertMouseDataMessage(srv.response.data);
        else {
            ROS_ERROR("Failed to call service getSensorData");
            data.id = "";
        }

        return data;
    }

    void MouseMonitorNode::processMouseDataMessage(
        const nxt_beagle::MouseMonitorSensorData& msg)
    {
        MouseData data = convertMouseDataMessage(msg);

        Q_EMIT measuredMouseData(data);
    }

    MouseData MouseMonitorNode::convertMouseDataMessage(
        const nxt_beagle::MouseMonitorSensorData& msg)
    {
        MouseData data;

        data.id = msg.id;
        data.x_disp = msg.x_disp;
        data.y_disp = msg.y_disp;
        data.x_speed = msg.x_speed;
        data.y_speed = msg.y_speed;

        return data;
    }

    pln_minotaur::PLN2033_Settings MouseMonitorNode::getMouseSettings(std::string id)
    {
        nxt_beagle::MouseMonitorSensorGetSettings srv;
        srv.request.id = id;

        pln_minotaur::PLN2033_Settings settings;

        if (serviceSettings.call(srv))
            settings = convertMouseSettingsMessage(srv.response.settings);
        else {
            ROS_ERROR("Failed to call service getSensorData");
            settings.spiDevice = "";
        }

        return settings;
    }

    void MouseMonitorNode::processMouseSettingsMessage(
        const nxt_beagle::MouseMonitorSensorSettings& settingsMsg)
    {
        pln_minotaur::PLN2033_Settings settings = convertMouseSettingsMessage(settingsMsg);

        Q_EMIT measuredMouseSettings(settings);
    }

    pln_minotaur::PLN2033_Settings MouseMonitorNode::convertMouseSettingsMessage(
        const nxt_beagle::MouseMonitorSensorSettings& settingsMsg)
    {
        pln_minotaur::PLN2033_Settings settings;

        settings.spiDevice = settingsMsg.spiDevice;
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

        return settings;
    }

}
