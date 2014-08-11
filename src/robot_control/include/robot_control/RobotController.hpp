#ifndef ROBOT_CONTROL_ROBOT_CONTROLLER_HPP
#define ROBOT_CONTROL_ROBOT_CONTROLLER_HPP

#include <tf/transform_broadcaster.h>
#include "robot_control/IRobotController.hpp"
#include "robot_control/PIDController.hpp"

namespace minotaur
{
  
    /**
     * \brief The RobotController calculates the correct velocity for each motor
     * 
     * Implements IRobotController.
     * This class receives linear and angular velocity and calculates the velocity
     * for each motor. A PIDController is used to keep the correct velocity on each
     * motor.
     */
    class RobotController: public IRobotController
    {
    private:
        nav_msgs::Odometry odometry;
        geometry_msgs::Twist velocity;
        float wheelTrack;
        PIDController pidController;
        
	/**
	 * Calculates the velocity for both motors
	 */
        void calculateMotorVelocity();
        geometry_msgs::Twist getMeasuredVelocity(const float p_theta);
	
	/**
	 * Calculates new position via deadReckoning
	 * @param p_samplingIntervalMsec duration of one sampling interval in Msec
	 */
        void deadReckoning(const int p_samplingIntervalMsec);
    public:
        RobotController();
        
        virtual ~RobotController() { }
        
        const nav_msgs::Odometry& getOdometry();
        IPIDController& getPIDController();
        float getWheelTrack() const;
        
        void setVelocity(const geometry_msgs::Twist& p_velocity);
        void setWheelTrack(const float p_wheelTrack);
        
        void setPose(const geometry_msgs::PoseWithCovariance& p_pose);
        
	/**
	 * Tells the PIDController to keep the correct velocity
	 * @param p_samplingIntervalMSec duration of one sampling interval in MSec
	 */
        void step(const int p_samplingIntervalMSec);
    };
}

#endif
