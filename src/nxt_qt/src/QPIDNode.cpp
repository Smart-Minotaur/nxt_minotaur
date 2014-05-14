#include "nxt_beagle/Config.hpp"
#include "nxt_qt/QPIDNode.hpp"
#include "nxt_beagle/Config.hpp"
#include "nxt_beagle/PIDParam.h"
#include <tf/transform_broadcaster.h>
#include <cmath>

#define DEG_TO_RAD(deg) ((deg / 180.0f) * M_PI)

namespace minotaur
{
    QPIDNode::QPIDNode()
    {
        ROS_INFO("Subscribing to topic \"%s\"...", ROS_ODOM_TOPIC);
        odometrySub = nodeHandle.subscribe(ROS_ODOM_TOPIC,
                                                      50,
                                                      &QPIDNode::processOdometryMsg,
                                                      this);
        ROS_INFO("Subscribing to topic \"%s\"...", NXT_ULTRA_SENSOR_TOPIC);
        ultrasensorSub = nodeHandle.subscribe(NXT_ULTRA_SENSOR_TOPIC,
                                                      50,
                                                      &QPIDNode::processSensorMsg,
                                                      this);
        
        ROS_INFO("Publishing on topic \"%s\"...", ROS_VEL_TOPIC);
        robotVelocityPublisher = nodeHandle.advertise<geometry_msgs::Twist>(ROS_VEL_TOPIC, 50);
        ROS_INFO("Publishing on topic \"%s\"...", NXT_SET_PID_PARAMETER);
        pidPramPublisher = nodeHandle.advertise<nxt_beagle::PIDParam>(NXT_SET_PID_PARAMETER, 50);
        
        int i = 0;
        float direction;
        std::string model;
        if(!ros::param::get(PARAM_CURRENT_MODEL(), model))
            throw std::logic_error("QPIDNode: No current model available");
        
        while(ros::param::has(PARAM_SENSOR(model, i)))
        {
            ros::param::get(PARAM_SENSOR_DIRECTION(model, i), direction);
            directions.push_back(DEG_TO_RAD(direction));
            ++i;
        }
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
        double theta = tf::getYaw(p_msg.pose.pose.orientation);
        vel.linearVelocity = p_msg.twist.twist.linear.x / cos(theta);
        vel.angularVelocity = p_msg.twist.twist.angular.z;
        
        Q_EMIT measuredVelocityUpdated(vel);
    }
    
    void QPIDNode::processSensorMsg(const nxt_beagle::UltraSensor p_msg)
    {
        QUltraSensor msg;
        msg.id = p_msg.sensorID;
        msg.direction = directions[p_msg.sensorID];
        msg.distance = p_msg.distance;
        
        Q_EMIT measuredSensor(msg);
    }
}