#include <stdexcept>
#include <string>
#include <cmath>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include "robot_control_beagle/RAIILock.hpp"
#include "robot_control_beagle/Utils.hpp"
#include "minotaur_control_pc/MinotaurControlNode.hpp"
#include "robot_control_beagle/SetPIDParameter.h"

#define DEG_TO_RAD(deg) ((M_PI * deg) / 180)
#define SPIN_RATE 100
#define ROS_MESSAGE_QUEUE_LENGTH 20

namespace minotaur
{
    void MinotaurControlNode::connectToROS(ros::NodeHandle &p_nodeHandle)
    {
        if(!ros::master::check())
            throw std::logic_error("MinotaurControlNode: ROS master is not running. Cannot connect to ROS");
        
        ROS_INFO("Subscribing to topic \"%s\"...", ROS_ODOM_TOPIC);
        odometrySubscriber = p_nodeHandle.subscribe(ROS_ODOM_TOPIC,
                                                      ROS_MESSAGE_QUEUE_LENGTH,
                                                      &MinotaurControlNode::processOdometryMsg,
                                                      this);
        ROS_INFO("Subscribing to topic \"%s\"...", NXT_ULTRA_SENSOR_TOPIC);
        ultrasonicSubscriber = p_nodeHandle.subscribe(NXT_ULTRA_SENSOR_TOPIC,
                                                      ROS_MESSAGE_QUEUE_LENGTH,
                                                      &MinotaurControlNode::processSensorMsg,
                                                      this);
        
        ROS_INFO("Publishing on topic \"%s\"...", ROS_VEL_TOPIC);
        robotVelocityPublisher = p_nodeHandle.advertise<geometry_msgs::Twist>(ROS_VEL_TOPIC, ROS_MESSAGE_QUEUE_LENGTH);
        ROS_INFO("Publishing on topic \"%s\"...", NXT_SET_PID_PARAMETER);
        pidParamPublisher = p_nodeHandle.advertise<robot_control_beagle::SetPIDParameter>(NXT_SET_PID_PARAMETER, ROS_MESSAGE_QUEUE_LENGTH);
        ROS_INFO("Publishing on topic \"%s\"...", ROS_SIMPLE_GOAL);
        targetPosPub = p_nodeHandle.advertise<geometry_msgs::PoseStamped>(ROS_SIMPLE_GOAL, ROS_MESSAGE_QUEUE_LENGTH);
        connected = true;
    }
        
    void MinotaurControlNode::setVelocity(const float p_linearVelocity, const float p_angularVelocity)
    {
        if(!connected)
            throw std::logic_error("MinotaurControlNode: Not connected to ROS. Cannot call setVelocity()");
        
        geometry_msgs::Twist msg;
        
        double theta = getThetaFromLastOdom();
        
        msg.linear.x = cos(theta) * p_linearVelocity;
        msg.linear.y = sin(theta) * p_linearVelocity;
        msg.linear.z = 0;
        msg.angular.x = 0;
        msg.angular.y = 0;
        msg.angular.z = p_angularVelocity;
        
        robotVelocityPublisher.publish(msg);
    }
    
    double MinotaurControlNode::getThetaFromLastOdom()
    {
        RAIILock lock(&odomMutex);
        return tf::getYaw(lastOdometry.pose.pose.orientation);
    }
    
    void MinotaurControlNode::setPIDParameter(const float p_Kp, const float p_Ki, const float p_Kd)
    {
        if(!connected)
            throw std::logic_error("MinotaurControlNode: Not connected to ROS. Cannot call setPIDParameter()");
        
        robot_control_beagle::SetPIDParameter msg;
        msg.Kp = p_Kp;
        msg.Ki = p_Ki;
        msg.Kd = p_Kd;
        pidParamPublisher.publish(msg);
    }
    
    void MinotaurControlNode::setSimpleTarget(const float p_x, const float p_y, const float p_theta)
    {
        geometry_msgs::PoseStamped msg;
        
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = MINOTAUR_BASE_FRAME;
        
        msg.pose.position.x = p_x;
        msg.pose.position.y = p_y;
        msg.pose.position.z = 0;
        
        msg.pose.orientation = tf::createQuaternionMsgFromYaw(p_theta);
        
        targetPosPub.publish(msg);
    }
    
    void MinotaurControlNode::spin()
    {
        if(!connected)
            throw std::logic_error("MinotaurControlNode: Not connected to ROS. Cannot call spin()");
        
        keepRunning = true;
        ros::Rate rate(SPIN_RATE);
        
        while(keepRunning) {
            ros::spinOnce();
            rate.sleep();
        }
    }
    
    void MinotaurControlNode::stop()
    {
        keepRunning = false;
    }
    
    void MinotaurControlNode::setMinotaurListener(IMinotaurListener *p_listener)
    {
        listener = p_listener;
    }
    
    void MinotaurControlNode::processOdometryMsg(const nav_msgs::Odometry &p_msg)
    {
        updateLastOdometry(p_msg);
        listener->onReceiveOdometry(p_msg);
    }
    
    void MinotaurControlNode::updateLastOdometry(const nav_msgs::Odometry &p_msg)
    {
        RAIILock lock(&odomMutex);
        lastOdometry = p_msg;
    }
    
    void MinotaurControlNode::processSensorMsg(const robot_control_beagle::UltrasonicData &p_msg)
    {
        listener->onReceivedUltrasonicData(p_msg);
    }
    
    std::vector<SensorSetting> MinotaurControlNode::getSensorSettings()
    {
        if(!ros::master::check())
            throw std::logic_error("MinotaurControlNode: ROS master is not running. Cannot read sensor settings");
        
        std::vector<SensorSetting> result;
        std::string model;
        
        if(!ros::param::get(PARAM_CURRENT_MODEL(), model))
            throw std::logic_error("MinotaurControlNode: No current robot model available");
            
        int i = 0;
        while(ros::param::has(PARAM_SENSOR(model, i)))
        {
            result.push_back(SensorSetting());
            
            result.back().id = i;
            
            // get direction and convert to radian
            ros::param::get(PARAM_SENSOR_DIRECTION(model, i), result.back().direction);
            result.back().direction = DEG_TO_RAD(result.back().direction);
            
            // get x and y positions relative to robot
            ros::param::get(PARAM_SENSOR_DX(model, i), result.back().x);
            ros::param::get(PARAM_SENSOR_DY(model, i), result.back().y);
            
            ++i;
        }
        
        return result;
    }
        
}
