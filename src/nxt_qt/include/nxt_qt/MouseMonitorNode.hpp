#ifndef MOUSE_MONITOR_NODE_H
#define MOUSE_MONITOR_NODE_H

#include <ros/ros.h>
#include <QThread>
#include <QMetaType>

#include "nxt_beagle/MouseMonitorConfig.hpp"
#include "PLN2033_Settings.h"


namespace minotaur
{

    struct MouseData {
        std::string id;
        double x_disp;
        double y_disp;
        double x_speed;
        double y_speed;
    };

    class MouseMonitorNode : public QThread
    {
            Q_OBJECT

        private:
// 	    MouseMonitorWindow monitorWindow;
	    
            ros::NodeHandle nodeHandle;

            ros::Subscriber subData;
            ros::Subscriber subSettings;

            ros::ServiceClient serviceData;
            ros::ServiceClient serviceSettings;
	    ros::ServiceClient serviceSetResolution;

            void processMouseDataMessage(
                const nxt_beagle::MouseMonitorSensorData& msg);

            MouseData convertMouseDataMessage(
                const nxt_beagle::MouseMonitorSensorData& msg);

            void processMouseSettingsMessage(
                const nxt_beagle::MouseMonitorSensorSettings& settingsMsg);

            pln_minotaur::PLN2033_Settings convertMouseSettingsMessage(
                const nxt_beagle::MouseMonitorSensorSettings& settingsMsg);

        public:
            MouseMonitorNode();
            virtual ~MouseMonitorNode();

            void run();
            pln_minotaur::PLN2033_Settings getMouseSettings(std::string id);
            MouseData getMouseData(std::string id);
	    bool sendResolution(std::string id, int resolution);

        Q_SIGNALS:
            void rosShutdown();
            void measuredMouseData(const MouseData data);
            void measuredMouseSettings(const pln_minotaur::PLN2033_Settings settings);

      
    };

}

Q_DECLARE_METATYPE(minotaur::MouseData);
Q_DECLARE_METATYPE(pln_minotaur::PLN2033_Settings);

#endif
