#ifndef MINOTAUR_MINOTAUR_COMMUNICATOR_HPP_
#define MINOTAUR_MINOTAUR_COMMUNICATOR_HPP_

#include <ros/ros.h>
#include "minotaur_pc/RobotPosition.hpp"
#include "minotaur_pc/RobotOdometry.hpp"
#include "minotaur_pc/SensorMeasurement.hpp"
#include "minotaur_pc/BlockingQueue.hpp"
#include "nav_msgs/Odometry.h"
#include "robot_control_beagle/UltrasonicData.h"

namespace minotaur
{
    class MinotaurCommunicator
    {
    private:
        RobotOdometry *odometry;
        BlockingQueue<SensorMeasurement> *queue;
        
        ros::Subscriber odomSub;
        ros::Subscriber measureSensorSub;
        ros::Publisher targetPosPub;
        
        void processMeasureSensorMsg(const robot_control_beagle::UltrasonicData& p_msg);
        void processOdometryMsg(const nav_msgs::Odometry& p_msgs);
    public:
        MinotaurCommunicator() { }
        virtual ~MinotaurCommunicator() { }
        
        void init(ros::NodeHandle& p_handle, RobotOdometry *p_robotOdom, BlockingQueue<SensorMeasurement> *p_queue);
        void setTargetPosition(const RobotPosition& p_position);
    };
}

#endif