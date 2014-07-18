/* The MinotaurControlNode class is used to control the minotaur via ROS.
 * It is an abstraction of the ROS communication, so no more ROS logic is needed
 * to control the minotaur. */

#ifndef MINOTAUR_MINOTAUR_CONTROL_NODE_H
#define MINOTAUR_MINOTAUR_CONTROL_NODE_H

#include <ros/ros.h>
#include <vector>
#include "minotaur_control_pc/IMinotaurListener.h"
#include "minotaur_control_pc/SensorSetting.h"

namespace minotaur
{
    class DefaultMinotaurListener : public IMinotaurListener
    {
    public:
        void onReceiveOdometry(const nav_msgs::Odometry &p_odometry) { }
        void onReceiveUltrasonicData(const robot_control_beagle::UltrasonicData &p_sensorData) { }
        void onROSShutdown() { }
    };
    
    class MinotaurControlNode
    {
    private:
        ros::Subscriber odometrySubscriber;
        ros::Subscriber ultrasonicSubscriber;
        
        ros::Publisher robotVelocityPublisher;
        ros::Publisher pidParamPublisher;
        
        DefaultMinotaurListener defaultListener;
        IMinotaurListener *listener;
        
        bool connected;
        
        void processOdometryMsg(const nav_msgs::Odometry& p_msg);
        void processSensorMsg(const robot_control_beagle::UltrasonicData p_msg);
        
    public:
        MinotaurControlNode(): connected(false), listener(&defaultListener) { }
        ~MinotaurControlNode() { }
        
        void connectToROS(ros::NodeHandle &p_nodeHandle);
        
        void setVelocity(const float p_linearVelocity, const float p_angularVelocity);
        void setPIDParameter(const float p_Kp, const float p_Ki, const float p_Kd);
        void spin();
        
        void setMinotaurListener(IMinotaurListener *p_listener);
        
        std::vector<SensorSetting> getSensorSettings();
    };
}

#endif