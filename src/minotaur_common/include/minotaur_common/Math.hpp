/**
 * \file Math.hpp
 * \brief This file provides mathematical macros, symbols and functions 
 *        for common operations.
 * 
 * Most functions are used to convert values into different measuring
 * units.
 * 
 * There are also functions that provide a consistent usage of certain
 * ROS messages.
 */

#ifndef MINOTAUR_MATH_HPP
#define MINOTAUR_MATH_HPP

#include <cmath>
#include <vector>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include "minotaur_common/Vector2.hpp"

#define MIN(a,b) ((a) < (b) ? (a) : (b))
#define MAX(a,b) ((a) > (b) ? (a) : (b))

#define DEG_PER_CIRCLE 360
#define RAD_PER_CIRCLE (2 * M_PI) 

#define MSEC_PER_SEC 1000
#define USEC_PER_SEC 1000000

#define CM_PER_METER 100

namespace minotaur
{
    float degreeToRadian(const float p_degree);
    float radianToDegree(const float p_radian);
    
    float usecToSec(const int p_usec);
    float msecToSec(const int p_msec);
    int secToUsec(const float p_sec);
    int secToMsec(const float p_sec);
    float msecToHz(const int p_msec);
    
    int meterToCm(const float p_meter);
    float cmToMeter(const int p_cm);
    
    float radiusToCircumference(const float p_radius);
    float circumferenceToRadius(const float p_circumference);
    
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
	
	Vector2 rotateVec(const Vector2 &p_vector, const float p_radian);
	Vector2 rotateVec(const float x, const float y, const float p_radian);
}

#endif
