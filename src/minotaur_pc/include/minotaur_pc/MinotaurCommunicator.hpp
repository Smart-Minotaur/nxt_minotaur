#ifndef MINOTAUR_MINOTAUR_COMMUNICATOR_HPP_
#define MINOTAUR_MINOTAUR_COMMUNICATOR_HPP_

#include <ros/ros.h>
#include "minotaur_pc/RobotPosition.hpp"
#include "minotaur_pc/MinotaurState.hpp"
#include "nav_msgs/Odometry.h"
#include "nxt_beagle/UltraSensor.h"

namespace minotaur
{
    class MinotaurCommunicator
    {
    private:
        MinotaurState *state;
        
        ros::Subscriber odomSub;
        ros::Subscriber measureSensorSub;
        ros::Publisher targetPosPub;
        
        void processMeasureSensorMsg(const nxt_beagle::UltraSensor& p_msg);
        void processOdometryMsg(const nav_msgs::Odometry& p_msgs);
    public:
        MinotaurCommunicator() { }
        virtual ~MinotaurCommunicator() { }
        
        void init(ros::NodeHandle& p_handle, MinotaurState *p_state);
        void setTargetPosition(const RobotPosition& p_position);
    };
}

#endif