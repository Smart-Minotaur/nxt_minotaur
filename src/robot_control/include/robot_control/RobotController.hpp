#ifndef ROBOT_CONTROL_ROBOT_CONTROLLER_HPP
#define ROBOT_CONTROL_ROBOT_CONTROLLER_HPP

#include <tf/transform_broadcaster.h>
#include "robot_control/IRobotController.hpp"
#include "robot_control/PIDController.hpp"

namespace minotaur
{
    /* This class implments an Interface to control a robot.
     * The class receives linear and angular velocity and calculates the
     * velocity for each motor (left and right).
     * A pid-controller is used to keep the correct velocity on each motor.
     * The pid-controller has to be initialized before using the "step()"
     * method (see "PIDController.hpp").
     * The wheel-track must be set before using the "step()" method. */
    class RobotController: public IRobotController
    {
    private:
        nav_msgs::Odometry odometry;
        geometry_msgs::Twist velocity;
        float wheelTrack;
        PIDController pidController;
        
        void calculateMotorVelocity();
        geometry_msgs::Twist getMeasuredVelocity(const float p_theta);
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
