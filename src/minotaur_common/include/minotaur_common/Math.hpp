#ifndef MINOTAUR_MATH_HPP
#define MINOTAUR_MATH_HPP

#include <cmath>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>

#define MIN(a,b) ((a) < (b) ? (a) : (b))
#define MAX(a,b) ((a) > (b) ? (a) : (b))

#define DEG_PER_CIRCLE 360
#define RAD_PER_CIRCLE (2 * M_PI) 

#define DEG_TO_RAD(deg) ((M_PI * deg) / 180)
#define RAD_TO_DEG(rad) ((rad * 180) / M_PI)

#define MSEC_PER_SEC 1000
#define USEC_PER_SEC 1000000

#define USEC_TO_SEC(usec) (((float) usec) / ((float) USEC_PER_SEC))
#define MSEC_TO_SEC(msec) (((float) msec) / ((float) MSEC_PER_SEC))
#define SEC_TO_USEC(sec) ((int) (sec * USEC_PER_SEC))
#define SEC_TO_MSEC(sec) ((int) (sec * MSEC_PER_SEC))

#define MSEC_TO_HZ(msec) (((double) MSEC_PER_SEC) / ((double) msec))

#define CM_PER_METER 100
#define CM_TO_METER(cm) (((float) cm) / ((float) CM_PER_METER))
#define METER_TO_CM(m) ((int) (m * CM_PER_METER))

#define RADIUS_TO_CIRCUMFERENCE(rad) (rad * 2 * M_PI)

namespace minotaur
{
    float normalizeRadian(const float p_radian);
    float normalizeDegree(const float p_degree);
    
    void initOdometry(nav_msgs::Odometry &p_odometry);
    void initTwist(geometry_msgs::Twist &p_twist);
    void initPose(geometry_msgs::Pose &p_pose);
    
    float getTheta(const nav_msgs::Odometry &p_odometry);
    float getTheta(const geometry_msgs::Pose &p_pose);
    float getNormalizedTheta(const nav_msgs::Odometry &p_odometry);
    float getNormalizedTheta(const geometry_msgs::Pose &p_pose);
    void setTheta(nav_msgs::Odometry &p_odometry, const float p_theta);
    void setTheta(geometry_msgs::Pose &p_pose, const float p_theta);
    
    float getLinearVelocity(const nav_msgs::Odometry &p_odometry);
    float getLinearVelocity(const geometry_msgs::Twist &p_twist, const float p_theta);
    void setLinearVelocity(nav_msgs::Odometry &p_odometry, const float p_linearVelocity);
    void setLinearVelocity(geometry_msgs::Twist &p_twist, const float p_theta, const float p_linearVelocity);
    
    float getAngularVelocity(const nav_msgs::Odometry &p_odometry);
    float getAngularVelocity(const geometry_msgs::Twist &p_twist);
    void setAngularVelocity(nav_msgs::Odometry &p_odometry, const float p_angularVelocity);
    void setAngularVelocity(geometry_msgs::Twist &p_twist, const float p_angularVelocity);
}

#endif
