#ifndef NXT_BEAGLE_ROBOT_COMMUNICATOR_HPP_
#define NXT_BEAGLE_ROBOT_COMMUNICATOR_HPP_

#include <pthread.h>
#include <ros/ros.h>
#include "nxt_control/Motor.hpp"
#include "nxt_beagle/RobotController.hpp"
#include "nxt_beagle/RVelocity.h"
#include "nxt_beagle/MVelocity.h"
#include "nxt_beagle/PIDParam.h"

namespace minotaur
{
    class RobotCommunicator
    {
    private:
        nxtcon::Motor leftMotor;
        nxtcon::Motor rightMotor;
        RobotController robotController;
        
        ros::Publisher targetMVelPub;
        ros::Publisher measuredMVelPub;
        ros::Publisher targetRVelPub;
        ros::Publisher measuredRVelPub;
        
        ros::Subscriber setRVelSub;
        ros::Subscriber setPIDParamSub;
        
        pthread_mutex_t robotMutex;
        
        void publishAll();
        nxt_beagle::MVelocity motorVelocityToMsg(const minotaur::MotorVelocity& p_velocity);
        nxt_beagle::RVelocity robotVelocityToMsg(const minotaur::RobotVelocity& p_velocity);
        
        void processRobotVelocityMsg(const nxt_beagle::RVelocity &p_msg);
        void processPIDParamMsg(const nxt_beagle::PIDParam &p_msg);
    public:
        bool pubTargetMVel;
        bool pubMeasuredMVel;
        bool pubTargetRVel;
        bool pubMeasuredRVel;
        
        RobotCommunicator();
        virtual ~RobotCommunicator() { }
        
        void init(ros::NodeHandle &p_handle, nxtcon::Brick *p_brick);
        
        void publish();
        
        void lock() { pthread_mutex_lock(&robotMutex); }
        void unlock() { pthread_mutex_unlock(&robotMutex); }
        
        RobotController& getRobotController();
    };
}

#endif