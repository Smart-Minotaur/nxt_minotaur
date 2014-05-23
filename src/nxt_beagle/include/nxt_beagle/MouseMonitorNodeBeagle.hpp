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
            ros::Publisher pubMouseData;
            ros::Publisher pubMouseSettings;

            pln_minotaur::IPLNTrackingDevice *sensor1;
            pln_minotaur::IPLNTrackingDevice *sensor2;

            void publishData(pln_minotaur::IPLNTrackingDevice * sensor)
            void publishSettings(pln_minotaur::IPLNTrackingDevice * sensor);

        public:
            MouseMonitorNodeBeagle();
            virtual ~MouseMonitorNodeBeagle();

            void run();
    };

}

#endif
