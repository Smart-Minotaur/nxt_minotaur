#ifndef MINOTAUR_MAP_MINOTAUR_COMMUNICATOR_HPP_
#define MINOTAUR_MAP_MINOTAUR_COMMUNICATOR_HPP_

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include "minotaur_map/RobotPosition.hpp"
#include "minotaur_map/RobotOdometry.hpp"
#include "minotaur_map/SensorMeasurement.hpp"
#include "minotaur_common/BlockingQueue.hpp"
#include "minotaur_common/MinotaurControlNode.hpp"

namespace minotaur
{
    /**
     * \brief Processes all ROS navigation stack communcication.
     * 
     * This class provides functionality to set the target position of
     * the robot in the ROS navigation stack.
     */
    class MinotaurCommunicator: public IMinotaurListener
    {
    private:
        MinotaurControlNode controlNode;
        RobotOdometry *odometry;
        BlockingQueue<SensorMeasurement> *queue;
        
        ros::Subscriber odomSub;
        ros::Subscriber measureSensorSub;
        ros::Publisher targetPosPub;
    public:
        MinotaurCommunicator();
        ~MinotaurCommunicator();
        
        void init(ros::NodeHandle& p_handle, RobotOdometry *p_robotOdom, BlockingQueue<SensorMeasurement> *p_queue);
        
        /**
         * Sets the target position of the robot, to which it should
         * move. The movement to the position is handled by the ROS
         * navigation stack.
         * @param p_position target position of the robot
         */
        void setTargetPosition(const RobotPosition& p_position);
        
        void onReceiveOdometry(const nav_msgs::Odometry &p_odometry);
        void onReceiveUltrasonicData(const minotaur_common::UltrasonicData &p_sensorData);
    };
}

#endif
