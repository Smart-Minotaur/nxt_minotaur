/*
 * Author: Fabian Meyer 
 */

#include "robot_control_beagle/RobotController.hpp"
#include "robot_control_beagle/MotorVelocity.hpp"

namespace minotaur
{
    RobotController::RobotController()
    : odometry(), velocity(), wheelTrack(0.0f), pidController()
    {
        odometry.pose.pose.position.x = 0;
        odometry.pose.pose.position.y = 0;
        odometry.pose.pose.position.z = 0;
        odometry.pose.pose.orientation = tf::createQuaternionMsgFromYaw(0);
        
        odometry.twist.twist.linear.x = 0;
        odometry.twist.twist.linear.y = 0;
        odometry.twist.twist.linear.z = 0;
        odometry.twist.twist.angular.x = 0;
        odometry.twist.twist.angular.y = 0;
        odometry.twist.twist.angular.z = 0;
        
        velocity.linear.x = 0;
        velocity.linear.y = 0;
        velocity.linear.z = 0;
        velocity.angular.x = 0;
        velocity.angular.y = 0;
        velocity.angular.z = 0; 
    }
    
    IPIDController& RobotController::getPIDController()
    {
        return pidController;
    }
    
    float RobotController::getWheelTrack() const
    {
        return wheelTrack;
    }
    
    const nav_msgs::Odometry& RobotController::getOdometry()
    {
        return odometry;
    }

    void RobotController::setVelocity(const geometry_msgs::Twist& p_velocity)
    {
        velocity = p_velocity;
        calculateMotorVelocity();
    }
    
    void RobotController::setWheelTrack(const float p_wheelTrack)
    {
        wheelTrack = p_wheelTrack;
        calculateMotorVelocity();
    }

    void RobotController::calculateMotorVelocity()
    {
        MotorVelocity targetVelocity;
        
        //to get the formula see kinematic of two wheeled robots
        double theta = tf::getYaw(odometry.pose.pose.orientation);
        float linearVelocity = velocity.linear.x / cos(theta);
        targetVelocity.leftMPS = linearVelocity - (velocity.angular.z * wheelTrack) / 2;
        targetVelocity.rightMPS = linearVelocity + (velocity.angular.z * wheelTrack) / 2;
        
        pidController.setVelocity(targetVelocity);
    }
    
    void RobotController::setPose(const geometry_msgs::PoseWithCovariance& p_pose)
    {
        odometry.pose = p_pose;
    }
    
    void RobotController::step(const int p_samplingIntervallMsec)
    {
        pidController.step(p_samplingIntervallMsec);
        deadReckoning(p_samplingIntervallMsec);
    }
    
    void RobotController::deadReckoning(const int p_samplingIntervallMsec)
    {
        float intervalSec = MS_TO_SEC(p_samplingIntervallMsec);
        
        double theta = tf::getYaw(odometry.pose.pose.orientation);
        
        // get measured Velocity from PID and transform to Twist
        geometry_msgs::Twist nextVelocity = getMeasuredVelocity(theta);
        
        //calculate new pose via deadReckoning
        odometry.pose.pose.position.x += ((odometry.twist.twist.linear.x + nextVelocity.linear.x) / 2) * intervalSec;
        odometry.pose.pose.position.y += ((odometry.twist.twist.linear.y + nextVelocity.linear.y) / 2) * intervalSec;
        odometry.pose.pose.orientation = tf::createQuaternionMsgFromYaw(theta + ((odometry.twist.twist.angular.z + nextVelocity.angular.z) / 2 ) * intervalSec);
        
        // set next measured velocity
        odometry.twist.twist = nextVelocity;
    }
    
    geometry_msgs::Twist RobotController::getMeasuredVelocity(const double p_theta)
    {
        geometry_msgs::Twist result;
        MotorVelocity motorVel = pidController.getMeasuredVelocity();
        //to get the formula see kinematic of two wheeled robots
        float linearVelocity = (motorVel.rightMPS + motorVel.leftMPS) / 2;
        float angularVelocity = (motorVel.rightMPS - motorVel.leftMPS) / wheelTrack;
        
        result.linear.x = cos(p_theta) * linearVelocity;
        result.linear.y = sin(p_theta) * linearVelocity;
        result.linear.z = 0;
        result.angular.x = 0;
        result.angular.y = 0;
        result.angular.z = angularVelocity;
        
        return result;
    }
}