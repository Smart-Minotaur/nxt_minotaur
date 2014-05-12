/*
 * Author: Fabian Meyer 
 */

#ifndef ROBOT_CONTROLLER_HPP_
#define ROBOT_CONTROLLER_HPP_

#include "nxt_beagle/IRobotController.hpp"
#include "nxt_beagle/PIDController.hpp"
#include <tf/transform_broadcaster.h>

namespace minotaur
{
    /* This class implments an Interface to control a robot.
     * The class receives linear and angular velocity and calculates the
     * velocity for each motor (left and right).
     * A pid-controller is used to keep the correct velocity on each motor.
     * The pid-controller has to be initialized before using the "step()"
     * method (see "PIDController.hpp").
     * The wheel-track must be ste before using the "step()" method. */
    class RobotController: public IRobotController
    {
    private:
        nav_msgs::Odometry odometry;
        geometry_msgs::Twist velocity;
        float wheelTrack;
        PIDController pidController;
        
        void calculateMotorVelocity();
        geometry_msgs::Twist getMeasuredVelocity(const double p_theta);
        void deadReckoning(const int p_samplingIntervallMsec);
    public:
        RobotController();
        
        virtual ~RobotController() { }
        
        const nav_msgs::Odometry& getOdometry();
        IPIDController& getPIDController();
        float getWheelTrack() const;
        
        void setVelocity(const geometry_msgs::Twist& p_velocity);
        void setWheelTrack(const float p_wheelTrack);
        
        void setPose(const geometry_msgs::PoseWithCovariance& p_pose);
        
        void step(const int p_samplingIntervallMSec);
    };
}

#endif