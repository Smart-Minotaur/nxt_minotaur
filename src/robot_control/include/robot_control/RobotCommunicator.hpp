#ifndef ROBOT_CONTROL_ROBOT_COMMUNICATOR_HPP
#define ROBOT_CONTROL_ROBOT_COMMUNICATOR_HPP

#include <pthread.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <move_base_msgs/MoveBaseActionFeedback.h>
#include "nxt_control/Motor.hpp"
#include "robot_control/RobotController.hpp"
#include "minotaur_common/PIDParameter.h"

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
        void processPIDParamMsg(const minotaur_common::PIDParameter &p_msg);
    public:
        RobotCommunicator();
        virtual ~RobotCommunicator() { }
        
        void init(ros::NodeHandle &p_handle, nxtcon::Brick *p_brick);
        void setTransformBroadcaster(tf::TransformBroadcaster *p_odomBroadcaster);
        
        void publish();
        
        pthread_mutex_t* mutex() { return &robotMutex; }
        
        RobotController& getRobotController();
    };
}

#endif
