#ifndef MINOTAUR_MINOTAUR_LISTENER_HPP
#define MINOTAUR_MINOTAUR_LISTENER_HPP

#include <nav_msgs/Odometry.h>
#include "robot_control_beagle/UltrasonicData.h"

namespace minotaur
{   
    class IMinotaurListener
    {
    public:
        virtual void onReceiveOdometry(const nav_msgs::Odometry &p_odometry) = 0;
        virtual void onReceiveUltrasonicData(const robot_control_beagle::UltrasonicData &p_sensorData) = 0;
        virtual void onROSShutdown() = 0;
    };
}

#endif
