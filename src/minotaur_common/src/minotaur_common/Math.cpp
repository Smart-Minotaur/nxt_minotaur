#include <tf/transform_broadcaster.h>
#include "minotaur_common/Math.hpp"

namespace minotaur
{
    float degreeToRadian(const float p_degree)
    {
        return (p_degree * M_PI) / 180.0f;
    }
    
    float radianToDegree(const float p_radian)
    {
        return (p_radian * 180.0f) / M_PI;
    }
    
    float usecToSec(const int p_usec)
    {
        return ((float) p_usec) / ((float) USEC_PER_SEC);
    }
    
    float msecToSec(const int p_msec)
    {
        return ((float) p_msec) / ((float) MSEC_PER_SEC);
    }
    
    int secToUsec(const float p_sec)
    {
        return (int) (p_sec * USEC_PER_SEC);
    }
    
    int secToMsec(const float p_sec)
    {
        return (int) (p_sec * MSEC_PER_SEC);
    }
    
    float msecToHz(const int p_msec)
    {
        return ((float) MSEC_PER_SEC) / ((float) p_msec);
    }
    
    int meterToCm(const float p_meter)
    {
        return (int) (p_meter * CM_PER_METER);
    }
    
    float cmToMeter(const int p_cm)
    {
        return ((float) p_cm) / ((float) CM_PER_METER);
    }
    
    float radiusToCircumference(const float p_radius)
    {
        return p_radius * 2 * M_PI;
    }
    
    float circumferenceToRadius(const float p_circumference)
    {
        return p_circumference / (2 * M_PI);
    }
    
    float normalizeRadian(const float p_radian)
    {
        float result = p_radian;
        
        while(result < 0)
            result += RAD_PER_CIRCLE;
        while(result > RAD_PER_CIRCLE)
            result -= RAD_PER_CIRCLE;
        
        return result;
    }
    
    float normalizeDegree(const float p_degree)
    {
        float result = p_degree;
        
        while(result < 0)
            result += DEG_PER_CIRCLE;
        while(result > DEG_PER_CIRCLE)
            result -= DEG_PER_CIRCLE;
        
        return result;
    }
    
    void initOdometry(nav_msgs::Odometry &p_odometry)
    {
        initPose(p_odometry.pose.pose);
        initTwist(p_odometry.twist.twist);        
    }
    
    void initTwist(geometry_msgs::Twist &p_twist)
    {
        p_twist.linear.x = 0;
        p_twist.linear.y = 0;
        p_twist.linear.z = 0;
        p_twist.angular.x = 0;
        p_twist.angular.y = 0;
        p_twist.angular.z = 0;
    }
    
    void initPose(geometry_msgs::Pose &p_pose)
    {
        p_pose.position.x = 0;
        p_pose.position.y = 0;
        p_pose.position.z = 0;
        setTheta(p_pose, 0);
    }
    
    float getTheta(const nav_msgs::Odometry &p_odometry)
    {
        return getTheta(p_odometry.pose.pose);
    }
    
    float getTheta(const geometry_msgs::Pose &p_pose)
    {
        return tf::getYaw(p_pose.orientation);
    }
    
    float getNormalizedTheta(const nav_msgs::Odometry &p_odometry)
    {
        return getNormalizedTheta(p_odometry.pose.pose);
    }
    
    float getNormalizedTheta(const geometry_msgs::Pose &p_pose)
    {
        return normalizeRadian(getTheta(p_pose));
    }
    
    void setTheta(nav_msgs::Odometry &p_odometry, const float p_theta)
    {
        setTheta(p_odometry.pose.pose, p_theta);
    }
    
    void setTheta(geometry_msgs::Pose &p_pose, const float p_theta)
    {
        p_pose.orientation = tf::createQuaternionMsgFromYaw(p_theta);
    }
    
    float getLinearVelocity(const nav_msgs::Odometry &p_odometry)
    {
        return getLinearVelocity(p_odometry.twist.twist, getTheta(p_odometry)); 
    }
    
    float getLinearVelocity(const geometry_msgs::Twist &p_twist, const float p_theta)
    {
        double cosVal = cos(p_theta);
        float result;
        if(cosVal == 0)
            result = p_twist.linear.y;
        else
            result = p_twist.linear.x / cosVal;
            
        return result;
    }
    
    void setLinearVelocity(nav_msgs::Odometry &p_odometry, const float p_linearVelocity)
    {
        setLinearVelocity(p_odometry.twist.twist, getTheta(p_odometry), p_linearVelocity);
    }
    
    void setLinearVelocity(geometry_msgs::Twist &p_twist, const float p_theta, const float p_linearVelocity)
    {
        p_twist.linear.x = cos(p_theta) * p_linearVelocity;
        p_twist.linear.y = sin(p_theta) * p_linearVelocity;
    }
    
    float getAngularVelocity(const nav_msgs::Odometry &p_odometry)
    {
        return getAngularVelocity(p_odometry.twist.twist);
    }
    
    float getAngularVelocity(const geometry_msgs::Twist &p_twist)
    {
        return p_twist.angular.z;
    }
    
    void setAngularVelocity(nav_msgs::Odometry &p_odometry, const float p_angularVelocity)
    {
        setAngularVelocity(p_odometry.twist.twist, p_angularVelocity);
    }
    
    void setAngularVelocity(geometry_msgs::Twist &p_twist, const float p_angularVelocity)
    {
        p_twist.angular.z = p_angularVelocity;
    }

	std::vector<float> rotateVec(const std::vector<float> &p_vector, const float p_radian)
	{
		return rotateVec(p_vector[0], p_vector[1], p_radian);
	}
	
	
	std::vector<float> rotateVec(const float x, const float y, const float p_radian)
	{
		std::vector<float> result(2);
		
		result[0] = x * cos(p_radian) + (y * -sin(p_radian));
		result[1] = x * sin(p_radian) + (y * cos(p_radian));
		
		return result;
	}
}
