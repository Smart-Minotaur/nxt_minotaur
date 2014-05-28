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
            ros::NodeHandle nodeHandle;

            ros::ServiceClient serviceData;
            ros::ServiceClient serviceSettings;

            MouseData convertMouseDataMessage(
                const nxt_beagle::MouseMonitorSensorData& msg);

            pln_minotaur::PLN2033_Settings convertMouseSettingsMessage(
                const nxt_beagle::MouseMonitorSensorSettings& settingsMsg);

        public:
            MouseMonitorNode();
            virtual ~MouseMonitorNode();

            void run();
            pln_minotaur::PLN2033_Settings getMouseSettings(std::string id);
            MouseData getMouseData(std::string id);

        Q_SIGNALS:
            void rosShutdown();
            void measuredMouseData(const MouseData data);
            void measuredMouseSettings(const pln_minotaur::PLN2033_Settings settings);
    };

}

Q_DECLARE_METATYPE(minotaur::MouseData);
Q_DECLARE_METATYPE(pln_minotaur::PLN2033_Settings);

#endif
