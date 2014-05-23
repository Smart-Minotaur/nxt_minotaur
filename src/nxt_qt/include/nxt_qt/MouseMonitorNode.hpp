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

            ros::Subscriber subData;
            ros::Subscriber subSettings;

            // Service
            ros::ServiceClient serviceData;
            ros::ServiceClient serviceSettings;

            void processMouseDataMessage(
                const nxt_beagle::MouseMonitorSensorData& msg);

            void processMouseSettingsMessage(
                const nxt_beagle::MouseMonitorSensorSettings& settingsMsg);

        public:
            MouseMonitorNode();
            virtual ~MouseMonitorNode();

            void run();

        Q_SIGNALS:
            void rosShutdown();
            void measuredMouseData(const MouseData data);
            void measuredMouseSettings(const std::string id,
                                       const pln_minotaur::PLN2033_Settings settings);
    };

}

Q_DECLARE_METATYPE(minotaur::MouseData);
Q_DECLARE_METATYPE(pln_minotaur::PLN2033_Settings);

#endif
