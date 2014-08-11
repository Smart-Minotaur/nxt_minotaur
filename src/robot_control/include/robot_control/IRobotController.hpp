#ifndef ROBOT_CONTROL_IROBOT_CONTROLLER_HPP
#define ROBOT_CONTROL_IROBOT_CONTROLLER_HPP

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include "robot_control/IPIDController.hpp"
#include "nav_msgs/Odometry.h"

namespace minotaur
{
    /** \brief This class provides an Interface to control the Robot
     * 
     * The robot is controlled by setting the linear- and angular velocity. 
     */
    class IRobotController
    {
    public:
        IRobotController() { }
        virtual ~IRobotController() { }
        
        virtual const nav_msgs::Odometry& getOdometry() = 0;
        virtual IPIDController& getPIDController() = 0;
        virtual float getWheelTrack() const = 0;
        
        virtual void setVelocity(const geometry_msgs::Twist& p_velocity) = 0;
        virtual void setWheelTrack(const float p_wheelTrack) = 0;
        
        virtual void setPose(const geometry_msgs::PoseWithCovariance& p_pose) = 0;
        
        virtual void step(const int p_samplingIntervalMSec) = 0;
    };
}

#endif
