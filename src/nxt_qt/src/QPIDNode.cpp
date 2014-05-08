#include "nxt_beagle/Config.hpp"
#include "nxt_qt/QPIDNode.hpp"
#include "nxt_beagle/Config.hpp"
#include "nxt_beagle/PIDParam.h"
#include <tf/transform_broadcaster.h>

namespace minotaur
{
    QPIDNode::QPIDNode()
    {
        ROS_INFO("Subscribing to topic \"%s\"...", ROS_ODOM_TOPIC);
        odometrySub = nodeHandle.subscribe(ROS_ODOM_TOPIC,
                                                      50,
                                                      &QPIDNode::processOdometryMsg,
                                                      this);
        
        ROS_INFO("Publishing on topic \"%s\"...", ROS_VEL_TOPIC);
        robotVelocityPublisher = nodeHandle.advertise<geometry_msgs::Twist>(ROS_VEL_TOPIC, 50);
        ROS_INFO("Publishing on topic \"%s\"...", NXT_SET_PID_PARAMETER);
        pidPramPublisher = nodeHandle.advertise<nxt_beagle::PIDParam>(NXT_SET_PID_PARAMETER, 50);
    }
    
    void QPIDNode::setRobotVelocity(const float p_linVel, const float p_angVel)
    {
        geometry_msgs::Twist msg;
        
        mutex.lock();
        double theta = tf::getYaw(lastOdometry.pose.pose.orientation);
        mutex.unlock();
        
        msg.linear.x = cos(theta) * p_linVel;
        msg.linear.y = sin(theta) * p_angVel;
        msg.linear.z = 0;
        msg.angular.x = 0;
        msg.angular.y = 0;
        msg.angular.z = p_angVel;
        
        robotVelocityPublisher.publish(msg);
    }
    
    void QPIDNode::setPIDParameter(const float p_Kp, const float p_Ki, const float p_Kd)
    {
        nxt_beagle::PIDParam msg;
        msg.Kp = p_Kp;
        msg.Ki = p_Ki;
        msg.Kd = p_Kd;
        pidPramPublisher.publish(msg);
    }
    
    void QPIDNode::run()
    {
        ros::spin();
        Q_EMIT rosShutdown();
    }
    
    void QPIDNode::processOdometryMsg(const nav_msgs::Odometry& p_msg)
    {
        mutex.lock();
        lastOdometry = p_msg;
        mutex.unlock();
        
        QRobotVelocity vel;
        vel.linearVelocity = sqrt(p_msg.twist.twist.linear.x * p_msg.twist.twist.linear.x + p_msg.twist.twist.linear.y * p_msg.twist.twist.linear.y);
        vel.angularVelocity = p_msg.twist.twist.angular.z;
        
        Q_EMIT measuredVelocityUpdated(vel);
    }
}