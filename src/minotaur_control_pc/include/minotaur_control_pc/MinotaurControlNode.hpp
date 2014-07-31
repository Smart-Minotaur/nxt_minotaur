/* The MinotaurControlNode class is used to control the minotaur via ROS.
 * It is an abstraction of the ROS communication, so no more ROS logic is needed
 * to control the minotaur. */

#ifndef MINOTAUR_MINOTAUR_CONTROL_NODE_HPP
#define MINOTAUR_MINOTAUR_CONTROL_NODE_HPP

#include <ros/ros.h>
#include <pthread.h>
#include <nav_msgs/Odometry.h>
#include "robot_control_beagle/UltrasonicData.h"
#include "minotaur_control_pc/IMinotaurListener.hpp"

namespace minotaur
{
    class DefaultMinotaurListener : public IMinotaurListener
    {
    public:
        void onReceiveOdometry(const nav_msgs::Odometry &p_odometry) { }
        void onReceivedUltrasonicData(const robot_control_beagle::UltrasonicData &p_sensorData) { }
    };
    
    class MinotaurControlNode
    {
    private:
        volatile bool keepRunning;
    
        ros::Subscriber odometrySubscriber;
        ros::Subscriber ultrasonicSubscriber;
        
        ros::Publisher robotVelocityPublisher;
        ros::Publisher pidParamPublisher;
        ros::Publisher targetPosPub;
        
        nav_msgs::Odometry lastOdometry;
        pthread_mutex_t odomMutex;
        
        DefaultMinotaurListener defaultListener;
        IMinotaurListener *listener;
        
        bool connected;
        
        void processOdometryMsg(const nav_msgs::Odometry &p_msg);
        void updateLastOdometry(const nav_msgs::Odometry &p_msg);
        double getThetaFromLastOdom();
        void processSensorMsg(const robot_control_beagle::UltrasonicData &p_msg);
        
    public:
        MinotaurControlNode(): connected(false), listener(&defaultListener) { pthread_mutex_init(&odomMutex, NULL); }
        ~MinotaurControlNode() { pthread_mutex_destroy(&odomMutex); }
        
        void connectToROS(ros::NodeHandle &p_nodeHandle);
        
        void setVelocity(const float p_linearVelocity, const float p_angularVelocity);
        void setPIDParameter(const float p_Kp, const float p_Ki, const float p_Kd);
        void setSimpleTarget(const float p_x, const float p_y, const float p_theta);
        
        void spin();
        void stop();
        
        void setMinotaurListener(IMinotaurListener *p_listener);
    };
}

#endif
