#include <tf/transform_broadcaster.h>
#include "minotaur_common/Math.hpp"

namespace minotaur
{
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
    
    float getTheta(const nav_msgs::Odometry &p_odometry)
    {
        return tf::getYaw(p_odometry.pose.pose.orientation);
    }
    
    float getNormalizedTheta(const nav_msgs::Odometry &p_odometry)
    {
        return normalizeRadian(getTheta(p_odometry));
    }
    
    float getLinearVelocity(const nav_msgs::Odometry &p_odometry)
    {
        return p_odometry.twist.twist.linear.x / cos(getTheta(p_odometry));
    }
    
    float getAngularVelocity(const nav_msgs::Odometry &p_odometry)
    {
        return p_odometry.twist.twist.angular.z;
    }
    
    void setLinearVelocity(nav_msgs::Odometry &p_odometry, const float p_linearVelocity)
    {
        float theta = getTheta(p_odometry);
        p_odometry.twist.twist.linear.x = cos(theta) * p_linearVelocity;
        p_odometry.twist.twist.linear.y = sin(theta) * p_linearVelocity;
    }
    
    void setAngularVelocity(nav_msgs::Odometry &p_odometry, const float p_angularVelocity)
    {
        p_odometry.twist.twist.angular.z = p_angularVelocity;
    }
}
