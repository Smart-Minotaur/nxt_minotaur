#include "robot_control_beagle/Utils.hpp"
#include "pid_monitor_pc/QMinotaurNavigateNode.hpp"
#include "robot_control_beagle/SetPIDParameter.h"
#include <tf/transform_broadcaster.h>
#include <cmath>

#define DEG_TO_RAD(deg) ((deg / 180.0f) * M_PI)

namespace minotaur
{
    QMinotaurNavigateNode::QMinotaurNavigateNode()
    {
        ROS_INFO("Subscribing to topic \"%s\"...", ROS_ODOM_TOPIC);
        odometrySub = nodeHandle.subscribe(ROS_ODOM_TOPIC,
                                                      50,
                                                      &QMinotaurNavigateNode::processOdometryMsg,
                                                      this);
        ROS_INFO("Subscribing to topic \"%s\"...", NXT_ULTRA_SENSOR_TOPIC);
        ultrasensorSub = nodeHandle.subscribe(NXT_ULTRA_SENSOR_TOPIC,
                                                      50,
                                                      &QMinotaurNavigateNode::processSensorMsg,
                                                      this);
        
        ROS_INFO("Publishing on topic \"%s\"...", ROS_VEL_TOPIC);
        robotVelocityPublisher = nodeHandle.advertise<geometry_msgs::Twist>(ROS_VEL_TOPIC, 50);
        ROS_INFO("Publishing on topic \"%s\"...", NXT_SET_PID_PARAMETER);
        pidPramPublisher = nodeHandle.advertise<robot_control_beagle::SetPIDParameter>(NXT_SET_PID_PARAMETER, 50);
        
        int i = 0;
        float direction;
        std::string model;
        if(!ros::param::get(PARAM_CURRENT_MODEL(), model))
            throw std::logic_error("QMinotaurNavigateNode: No current model available");
        
        while(ros::param::has(PARAM_SENSOR(model, i)))
        {
            ros::param::get(PARAM_SENSOR_DIRECTION(model, i), direction);
            directions.push_back(DEG_TO_RAD(direction));
            ++i;
        }
    }
    
    void QMinotaurNavigateNode::setRobotVelocity(const float p_linVel, const float p_angVel)
    {
        geometry_msgs::Twist msg;
        
        mutex.lock();
        double theta = tf::getYaw(lastOdometry.pose.pose.orientation);
        mutex.unlock();
        
        msg.linear.x = cos(theta) * p_linVel;
        msg.linear.y = sin(theta) * p_linVel;
        msg.linear.z = 0;
        msg.angular.x = 0;
        msg.angular.y = 0;
        msg.angular.z = p_angVel;
        
        robotVelocityPublisher.publish(msg);
    }
    
    void QMinotaurNavigateNode::setPIDParameter(const float p_Kp, const float p_Ki, const float p_Kd)
    {
        robot_control_beagle::SetPIDParameter msg;
        msg.Kp = p_Kp;
        msg.Ki = p_Ki;
        msg.Kd = p_Kd;
        pidPramPublisher.publish(msg);
    }
    
    void QMinotaurNavigateNode::run()
    {
        ros::spin();
        Q_EMIT rosShutdown();
    }
    
    void QMinotaurNavigateNode::processOdometryMsg(const nav_msgs::Odometry& p_msg)
    {
        mutex.lock();
        lastOdometry = p_msg;
        mutex.unlock();
        
        QOdometry odom;
        
        odom.x = p_msg.pose.pose.position.x;
        odom.y = p_msg.pose.pose.position.y;
        odom.theta = tf::getYaw(p_msg.pose.pose.orientation);
        
        odom.linVelX = p_msg.twist.twist.linear.x;
        odom.linVelY = p_msg.twist.twist.linear.y;
        odom.angVel = p_msg.twist.twist.angular.z;
        
        Q_EMIT odometryUpdated(odom);
    }
    
    void QMinotaurNavigateNode::processSensorMsg(const robot_control_beagle::UltrasonicData p_msg)
    {
        QUltraSensor msg;
        msg.id = p_msg.sensorID;
        msg.direction = directions[p_msg.sensorID];
        msg.distance = p_msg.distance;
        
        Q_EMIT measuredSensor(msg);
    }
}
