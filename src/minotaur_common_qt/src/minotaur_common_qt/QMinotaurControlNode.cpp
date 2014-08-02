#include "minotaur_common_qt/QMinotaurControlNode.hpp"

namespace minotaur
{
    QMinotaurListener::QMinotaurListener(QMinotaurControlNode &p_controlNode)
    :controlNode(p_controlNode) { }
    QMinotaurListener::~QMinotaurListener() { }

    void QMinotaurListener::onReceiveOdometry(const nav_msgs::Odometry &p_odometry)
    {
        controlNode.onReceiveOdometry(p_odometry);
    }
    
    void QMinotaurListener::onReceiveUltrasonicData(const minotaur_common::UltrasonicData &p_sensorData)
    {
        controlNode.onReceiveUltrasonicData(p_sensorData);
    }
    
    QMinotaurControlNode::QMinotaurControlNode()
    : controlNode(), qListener(*this)
    {
        controlNode.setMinotaurListener(&qListener);
    }
    
    QMinotaurControlNode::~QMinotaurControlNode()
    {
        
    }
    
    void QMinotaurControlNode::onReceiveOdometry(const nav_msgs::Odometry &p_odometry)
    {
        QOdometry msg;
        msg.odometry = p_odometry;
        
        Q_EMIT receivedOdometry(msg);
    }
    
    void QMinotaurControlNode::onReceiveUltrasonicData(const minotaur_common::UltrasonicData &p_sensorData)
    {
        QUltrasonicData msg;
        msg.data = p_sensorData;
        
        Q_EMIT receivedUltrasonicData(msg);
    }
    
    void QMinotaurControlNode::connectToROS(ros::NodeHandle &p_nodeHandle)
    {
        controlNode.connectToROS(p_nodeHandle);
    }
    
    void QMinotaurControlNode::setVelocity(const float p_linearVelocity, const float p_angularVelocity)
    {
        controlNode.setVelocity(p_linearVelocity, p_angularVelocity);
    }
    
    void QMinotaurControlNode::setPIDParameter(const float p_Kp, const float p_Ki, const float p_Kd)
    {
        controlNode.setPIDParameter(p_Kp, p_Ki, p_Kd);
    }
    
    void QMinotaurControlNode::setSimpleTarget(const float p_x, const float p_y, const float p_theta)
    {
        controlNode.setSimpleTarget(p_x, p_y, p_theta);
    }
    
    void QMinotaurControlNode::spin()
    {
        controlNode.spin();
    }
    
    void QMinotaurControlNode::stop()
    {
        controlNode.stop();
    }
}
