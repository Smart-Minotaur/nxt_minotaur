#include "robot_control/RobotController.hpp"
#include "minotaur_common/MotorVelocity.hpp"
#include "minotaur_common/Math.hpp"

namespace minotaur
{
    RobotController::RobotController(nxt::Motor &p_leftMotor, nxt::Motor &p_rightMotor)
    : odometry(), pidController(p_leftMotor, p_rightMotor)
    {
        initOdometry(odometry);
    }
	
	RobotController::~RobotController()
	{
		
	}
    
    PIDController& RobotController::getPIDController()
    {
        return pidController;
    }
    
    const nav_msgs::Odometry& RobotController::getOdometry()
    {
        return odometry;
    }

    void RobotController::setVelocity(const geometry_msgs::Twist& p_velocity)
    {
		pidController.setVelocity(getLinearVelocity(p_velocity, getTheta(odometry)), getAngularVelocity(p_velocity));
    }
    
    void RobotController::setPose(const geometry_msgs::PoseWithCovariance& p_pose)
    {
        odometry.pose = p_pose;
    }
    
    void RobotController::step(const int p_samplingIntervalMsec)
    {
        pidController.step(p_samplingIntervalMsec);
        deadReckoning(p_samplingIntervalMsec);
    }
    
    void RobotController::deadReckoning(const int p_samplingIntervalMsec)
    {
        float intervalSec = msecToSec(p_samplingIntervalMsec);
        
        float theta = getTheta(odometry);
        
        // get measured Velocity from PID and transform to Twist
        geometry_msgs::Twist nextVelocity = getMeasuredVelocity(theta);
        //calculate new pose via deadReckoning
        odometry.pose.pose.position.x += ((odometry.twist.twist.linear.x + nextVelocity.linear.x) / 2) * intervalSec;
        odometry.pose.pose.position.y += ((odometry.twist.twist.linear.y + nextVelocity.linear.y) / 2) * intervalSec;
        setTheta(odometry, theta + ((getAngularVelocity(odometry) + getAngularVelocity(nextVelocity)) / 2 ) * intervalSec);
        
        // set next measured velocity
        odometry.twist.twist = nextVelocity;
    }
    
    geometry_msgs::Twist RobotController::getMeasuredVelocity(const float p_theta)
    {
        geometry_msgs::Twist result;
        
        initTwist(result);
        setLinearVelocity(result, p_theta, pidController.getLinearVelocity());
        setAngularVelocity(result, pidController.getAngularVelocity());
        
        return result;
    }
    
}
