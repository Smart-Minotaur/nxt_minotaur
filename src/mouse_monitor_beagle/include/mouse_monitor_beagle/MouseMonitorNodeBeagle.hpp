#ifndef MOUSE_MONITOR_NODE_BEAGLE_H
#define MOUSE_MONITOR_NODE_BEAGLE_H

#include <ros/ros.h>
#include "mouse_monitor_beagle/MouseMonitorConfig.hpp"

// PLN2033 software
#include "IPLNTrackingDevice.h"
#include "PLN2033.h"
#include "PLN2033_Settings.h"

namespace minotaur
{

    class MouseMonitorNodeBeagle
    {
        private:
            ros::NodeHandle nodeHandle;
            ros::ServiceServer serviceData;
            ros::ServiceServer serviceSettings;
            ros::ServiceServer serviceSetResolution;

            pln_minotaur::IPLNTrackingDevice *sensor1;
            pln_minotaur::IPLNTrackingDevice *sensor2;

            bool sendData(mouse_monitor_beagle::MouseMonitorSensorGetData::Request &req,
                          mouse_monitor_beagle::MouseMonitorSensorGetData::Response &res);
            bool sendSettings(mouse_monitor_beagle::MouseMonitorSensorGetSettings::Request &req,
                              mouse_monitor_beagle::MouseMonitorSensorGetSettings::Response &res);

            bool setResolution(mouse_monitor_beagle::MouseMonitorSensorSetResolution::Request &req,
                                mouse_monitor_beagle::MouseMonitorSensorSetResolution::Response &res);

            mouse_monitor_beagle::MouseMonitorSensorSettings getSettings(
                pln_minotaur::IPLNTrackingDevice *sensor);

            mouse_monitor_beagle::MouseMonitorSensorData getData(
                pln_minotaur::IPLNTrackingDevice *sensor);

        public:
            MouseMonitorNodeBeagle();
            virtual ~MouseMonitorNodeBeagle();

            void run();
    };

}

#endif
