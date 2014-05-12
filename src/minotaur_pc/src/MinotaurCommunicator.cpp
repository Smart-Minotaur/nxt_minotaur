#include "minotaur_pc/MinotaurCommunicator.hpp"
#include "minotaur_pc/Movement.hpp"
#include "nxt_beagle/Config.hpp"

namespace minotaur
{
    
    void MinotaurCommunicator::init(ros::NodeHandle& p_handle, MinotaurState *p_state)
    {
        state = p_state;
        
        ROS_INFO("Subscribing on topic \"%s\"...", ROS_ODOM_TOPIC);
        odomSub = p_handle.subscribe(ROS_ODOM_TOPIC, 100, &MinotaurCommunicator::processOdometryMsg, this); 
        ROS_INFO("Subscribing on topic \"%s\"...", NXT_ULTRA_SENSOR_TOPIC);
        measureSensorSub = p_handle.subscribe(NXT_ULTRA_SENSOR_TOPIC, 100, &MinotaurCommunicator::processMeasureSensorMsg, this); 
        
        ROS_INFO("Publishing on topic \"%s\"...", ROS_SIMPLE_GOAL);
        targetPosPub = p_handle.advertise<geometry_msgs::PoseStamped>(ROS_SIMPLE_GOAL, 100);
    }
    
    void MinotaurCommunicator::setTargetPosition(const RobotPosition& p_position)
    {
        geometry_msgs::PoseStamped msg;
        
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = MINOTAUR_BASE_FRAME;
        
        msg.pose.position.x = p_position.point.x;
        msg.pose.position.y = p_position.point.y;
        msg.pose.position.z = 0;
        
        msg.pose.orientation = tf::createQuaternionMsgFromYaw(p_position.theta);
        
        targetPosPub.publish(msg);
    }
    
    void MinotaurCommunicator::processMeasureSensorMsg(const nxt_beagle::UltraSensor& p_msg)
    {
        state->setDistance(p_msg.sensorID, p_msg.distance);
    }
    
    void MinotaurCommunicator::processOdometryMsg(const nav_msgs::Odometry& p_msg)
    {
        RobotPosition position;
        Movement movement;
        
        position.convert(p_msg.pose);
        movement.convert(p_msg.twist);
        state->setOdometry(position, movement);
    }
}