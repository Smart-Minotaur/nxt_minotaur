#include "minotaur_map/MinotaurCommunicator.hpp"
#include "minotaur_map/Movement.hpp"

namespace minotaur
{
    MinotaurCommunicator::MinotaurCommunicator()
    {
        
    }
    
    MinotaurCommunicator::~MinotaurCommunicator()
    {
        
    }
        
    void MinotaurCommunicator::init(ros::NodeHandle& p_handle, RobotOdometry *p_robotOdom, BlockingQueue<SensorMeasurement> *p_queue)
    {
        odometry = p_robotOdom;
        queue = p_queue;
        
        controlNode.setMinotaurListener(this);
        controlNode.connectToROS(p_handle);
    }
    
    void MinotaurCommunicator::setTargetPosition(const RobotPosition& p_position)
    {
        controlNode.setSimpleTarget(p_position.point.x, p_position.point.y, p_position.theta);
    }
    
    void MinotaurCommunicator::onReceiveOdometry(const nav_msgs::Odometry &p_odometry)
    {
        RobotPosition position;
        Movement movement;
        
        position.convert(p_odometry.pose);
        movement.convert(p_odometry.twist);
        odometry->setOdometry(position, movement);
    }
    
    void MinotaurCommunicator::onReceiveUltrasonicData(const minotaur_common::UltrasonicData &p_sensorData)
    {
        queue->enqueue(SensorMeasurement(p_sensorData.sensorID, p_sensorData.distance));
    }
}
