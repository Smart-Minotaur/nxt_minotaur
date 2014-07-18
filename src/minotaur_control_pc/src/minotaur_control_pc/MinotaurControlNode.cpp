#include <stderr>
#include <string>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Twist.h>
#include "robot_control_beagle/Utils.hpp"
#include "minotaur_control_pc/MinotaurControlNode.h"
#include "robot_control_beagle/SetPIDParameter.h"

namespace minotaur
{
    void MinotaurControlNode::connectToROS(ros::NodeHandle &p_nodeHandle)
    {
        ROS_INFO("Subscribing to topic \"%s\"...", ROS_ODOM_TOPIC);
        odometrySub = p_nodeHandle.subscribe(ROS_ODOM_TOPIC,
                                                      50,
                                                      &MinotaurControlNode::processOdometryMsg,
                                                      this);
        ROS_INFO("Subscribing to topic \"%s\"...", NXT_ULTRA_SENSOR_TOPIC);
        ultrasensorSub = p_nodeHandle.subscribe(NXT_ULTRA_SENSOR_TOPIC,
                                                      50,
                                                      &MinotaurControlNode::processSensorMsg,
                                                      this);
        
        ROS_INFO("Publishing on topic \"%s\"...", ROS_VEL_TOPIC);
        robotVelocityPublisher = p_nodeHandle.advertise<geometry_msgs::Twist>(ROS_VEL_TOPIC, 50);
        ROS_INFO("Publishing on topic \"%s\"...", NXT_SET_PID_PARAMETER);
        pidPramPublisher = p_nodeHandle.advertise<robot_control_beagle::SetPIDParameter>(NXT_SET_PID_PARAMETER, 50);
        
        connected = true;
    }
        
    void MinotaurControlNode::setVelocity(const float p_linearVelocity, const float p_angularVelocity)
    {
        if(!connected)
            throw std::logic_error("MinotaurControlNode: not connected to ROS");
        
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
    
    void MinotaurControlNode::setPIDParameter(const float p_Kp, const float p_Ki, const float p_Kd)
    {
        if(!connected)
            throw std::logic_error("MinotaurControlNode: not connected to ROS");
        
        robot_control_beagle::SetPIDParameter msg;
        msg.Kp = p_Kp;
        msg.Ki = p_Ki;
        msg.Kd = p_Kd;
        pidPramPublisher.publish(msg);
    }
    
    void spin()
    {
        if(!connected)
            throw std::logic_error("MinotaurControlNode: not connected to ROS");
        
        ros::spin();
        listener->onROSShutdown();
    }
    
    void MinotaurControlNode::setMinotaurListener(IMinotaurListener *p_listener)
    {
        listener = p_listener;
    }
    
    void MinotaurControlNode::processOdometryMsg(const nav_msgs::Odometry &p_msg)
    {
        listener->onReceiveOdometry(p_msg);
    }
    
    void MinotaurControlNode::processSensorMsg(const robot_control_beagle::UltrasonicData &p_msg)
    {
        listener->onReceiveUltrasonicData(p_msg);
    }
    
    std::vector<SensorSetting> MinotaurControlNode::getSensorSettings()
    {
        std::vector<SensorSetting> result;
        std::string model;
        
        int i = 0;
        if(!ros::param::get(PARAM_CURRENT_MODEL(), model))
            throw std::logic_error("MinotaurControlNode: No current robot model available");
        
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