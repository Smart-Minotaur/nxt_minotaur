#ifndef ROBOT_CONTROL_ROBOT_COMMUNICATOR_HPP_
#define ROBOT_CONTROL_ROBOT_COMMUNICATOR_HPP_

#include <pthread.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include "nxt_control/Motor.hpp"
#include "robot_control_beagle/RobotController.hpp"
#include "robot_control_beagle/SetPIDParameter.h"
#include "move_base_msgs/MoveBaseActionFeedback.h"

namespace minotaur
{
    class RobotCommunicator
    {
    private:
        nxtcon::Motor leftMotor;
        nxtcon::Motor rightMotor;
        RobotController robotController;
        
        ros::Publisher odometryPub;
        
        ros::Subscriber setPIDParamSub;
        ros::Subscriber cmdVelSub;
        
        tf::TransformBroadcaster *odomBroadcaster;
        
        pthread_mutex_t robotMutex;
        
        void processSetVelocityMsg(const geometry_msgs::Twist& p_msg);
        void processPIDParamMsg(const robot_control_beagle::SetPIDParameter &p_msg);
    public:
        RobotCommunicator();
        virtual ~RobotCommunicator() { }
        
        void init(ros::NodeHandle &p_handle, nxtcon::Brick *p_brick);
        void setTransformBroadcaster(tf::TransformBroadcaster *p_odomBroadcaster);
        
        void publish();
        
        void lock() { pthread_mutex_lock(&robotMutex); }
        void unlock() { pthread_mutex_unlock(&robotMutex); }
        
        RobotController& getRobotController();
    };
}

#endif