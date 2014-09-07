#include <stdexcept>
#include <string>
#include <cmath>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include "minotaur_common/MinotaurTopics.hpp"
#include "minotaur_common/RAIILock.hpp"
#include "minotaur_common/MinotaurControlNode.hpp"
#include "minotaur_common/PIDParameter.h"
#include "minotaur_common/Math.hpp"

#define SPIN_RATE 100
#define ROS_MESSAGE_QUEUE_LENGTH 20

namespace minotaur
{
	class DefaultMinotaurListener : public IMinotaurListener
    {
    public:
        void onReceiveOdometry(const nav_msgs::Odometry &p_odometry) { }
        void onReceiveUltrasonicData(const minotaur_common::UltrasonicData &p_sensorData) { }
    };
	
	static DefaultMinotaurListener defaultListener;
	
	MinotaurControlNode::MinotaurControlNode()
	: connected(false), listener(&defaultListener)
	{
		initOdometry(lastOdometry);
		pthread_mutex_init(&odomMutex, NULL);
	}
	
	MinotaurControlNode::~MinotaurControlNode()
	{
		pthread_mutex_destroy(&odomMutex);
	}
	
    void MinotaurControlNode::connectToROS(ros::NodeHandle &p_nodeHandle)
    {
        if(!ros::master::check())
            throw std::logic_error("MinotaurControlNode: ROS master is not running. Cannot connect to ROS");
        
        ROS_INFO("Subscribing to topic \"%s\"...", ROS_ODOM_TOPIC);
        odometrySubscriber = p_nodeHandle.subscribe(ROS_ODOM_TOPIC,
                                                      ROS_MESSAGE_QUEUE_LENGTH,
                                                      &MinotaurControlNode::processOdometryMsg,
                                                      this);
        ROS_INFO("Subscribing to topic \"%s\"...", MINOTAUR_MEASURE_SENSOR_TOPIC);
        ultrasonicSubscriber = p_nodeHandle.subscribe(MINOTAUR_MEASURE_SENSOR_TOPIC,
                                                      ROS_MESSAGE_QUEUE_LENGTH,
                                                      &MinotaurControlNode::processSensorMsg,
                                                      this);
        
        ROS_INFO("Publishing on topic \"%s\"...", ROS_VEL_TOPIC);
        robotVelocityPublisher = p_nodeHandle.advertise<geometry_msgs::Twist>(ROS_VEL_TOPIC, ROS_MESSAGE_QUEUE_LENGTH);
        ROS_INFO("Publishing on topic \"%s\"...", MINOTAUR_SET_PID_PARAMETER_TOPIC);
        pidParamPublisher = p_nodeHandle.advertise<minotaur_common::PIDParameter>(MINOTAUR_SET_PID_PARAMETER_TOPIC, ROS_MESSAGE_QUEUE_LENGTH);
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
        return getTheta(lastOdometry);
    }
    
    void MinotaurControlNode::setPIDParameter(const float p_Kp, const float p_Ki, const float p_Kd)
    {
        if(!connected)
            throw std::logic_error("Not connected to ROS. Cannot call setPIDParameter()");
        
        minotaur_common::PIDParameter msg;
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
            throw std::logic_error("Not connected to ROS. Cannot call spin()");
        
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
    
    void MinotaurControlNode::processSensorMsg(const minotaur_common::UltrasonicData &p_msg)
    {
        listener->onReceiveUltrasonicData(p_msg);
    }
        
}
