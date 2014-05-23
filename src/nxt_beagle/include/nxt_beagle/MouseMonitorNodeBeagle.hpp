#ifndef MOUSE_MONITOR_NODE_BEAGLE_H
#define MOUSE_MONITOR_NODE_BEAGLE_H

#include <ros/ros.h>
#include "nxt_beagle/MouseMonitorConfig.hpp"

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
            ros::Publisher pubData;
            ros::Publisher pubSettings;

            // Service
            ros::ServiceServer serviceData;
            ros::ServiceServer serviceSettings;

            pln_minotaur::IPLNTrackingDevice *sensor1;
            pln_minotaur::IPLNTrackingDevice *sensor2;

            bool sendData(nxt_beagle::MouseMonitorGetData::Request  &req,
                          nxt_beagle::MouseMonitorGetData::Response &res);
            bool sendSettings(nxt_beagle::MouseMonitorGetSettings::Request  &req,
                              nxt_beagle::MouseMonitorGetSettings::Response &res);
            void publishData(pln_minotaur::IPLNTrackingDevice *sensor);
            void publishSettings(pln_minotaur::IPLNTrackingDevice *sensor);

        public:
            MouseMonitorNodeBeagle();
            virtual ~MouseMonitorNodeBeagle();

            void run();
    };

}

#endif
