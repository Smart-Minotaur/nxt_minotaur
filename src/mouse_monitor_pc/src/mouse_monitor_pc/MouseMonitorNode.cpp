#include "mouse_monitor_pc/MouseMonitorNode.hpp"
#include "mouse_monitor_beagle/MouseMonitorConfig.hpp"
#include "PLN2033_Settings.h"

namespace minotaur
{

    MouseMonitorNode::MouseMonitorNode()
    {
        ROS_INFO("Creating service clients");

        serviceData = nodeHandle.serviceClient<mouse_monitor_beagle::MouseMonitorSensorGetData>("getSensorData");
        serviceSettings= nodeHandle.serviceClient<mouse_monitor_beagle::MouseMonitorSensorGetSettings>("getSensorSettings");
        serviceSetResolution = nodeHandle.serviceClient<mouse_monitor_beagle::MouseMonitorSensorSetResolution>("setResolution");
    }

    MouseMonitorNode::~MouseMonitorNode() {}

    void MouseMonitorNode::run()
    {
        ros::spin();
        Q_EMIT rosShutdown();
    }

    MouseData MouseMonitorNode::getMouseData(std::string id)
    {
        mouse_monitor_beagle::MouseMonitorSensorGetData srv;
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

    MouseData MouseMonitorNode::convertMouseDataMessage(
        const mouse_monitor_beagle::MouseMonitorSensorData& msg)
    {
        MouseData data;

        data.id = msg.id;
        data.x_disp = msg.x_disp;
        data.y_disp = msg.y_disp;
        data.x_speed = msg.x_speed;
        data.y_speed = msg.y_speed;

        return data;
    }

    bool MouseMonitorNode::sendResolution(std::string id, int resolution)
    {
        mouse_monitor_beagle::MouseMonitorSensorSetResolution srv;
        srv.request.id = id;
        srv.request.newResolution = resolution;

        if(serviceSetResolution.call(srv)) {
            return true;
        } else {
            ROS_ERROR("Failed to call service setResolution");
        }

        return false;
    }

    pln_minotaur::PLN2033_Settings MouseMonitorNode::getMouseSettings(std::string id)
    {
        mouse_monitor_beagle::MouseMonitorSensorGetSettings srv;
        srv.request.id = id;

        pln_minotaur::PLN2033_Settings settings;

        if (serviceSettings.call(srv))
            settings = convertMouseSettingsMessage(srv.response.settings);
        else {
            ROS_ERROR("Failed to call service getSensorSettings");
            settings.spiDevice = "";
        }

        return settings;
    }

    pln_minotaur::PLN2033_Settings MouseMonitorNode::convertMouseSettingsMessage(
        const mouse_monitor_beagle::MouseMonitorSensorSettings& settingsMsg)
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
