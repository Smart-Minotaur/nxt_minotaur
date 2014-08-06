#ifndef QMINOTAUR_CONTROL_NODE_HPP
#define QMINOTAUR_CONTROL_NODE_HPP

#include <ros/ros.h>
#include "minotaur_common/MinotaurControlNode.hpp"
#include "minotaur_common_qt/MetaTypes.hpp"

namespace minotaur
{
    class QMinotaurControlNode;
    
    class QMinotaurListener: public IMinotaurListener
    {
    private:
        QMinotaurControlNode &controlNode;
    public:
        QMinotaurListener(QMinotaurControlNode &p_controlNode);
        ~QMinotaurListener();
    
        void onReceiveOdometry(const nav_msgs::Odometry &p_odometry);
        void onReceiveUltrasonicData(const minotaur_common::UltrasonicData &p_sensorData);
    };
    
    /**
     * \brief The QMinotaurControlNode class is a QT extension of the
     *        MinotaurControlNode class.
     * 
     * It wraps the callbacks of IMinotaurListener, so QSignals can be
     * used to listen for new incoming messages.
     */ 
    class QMinotaurControlNode : public QObject
    {
        Q_OBJECT
    private:
        MinotaurControlNode controlNode;
        QMinotaurListener qListener;
    public:
        QMinotaurControlNode();
        ~QMinotaurControlNode();
        
        void connectToROS(ros::NodeHandle &p_nodeHandle);
        
        void setVelocity(const float p_linearVelocity, const float p_angularVelocity);
        void setPIDParameter(const float p_Kp, const float p_Ki, const float p_Kd);
        void setSimpleTarget(const float p_x, const float p_y, const float p_theta);
        
        void spin();
        void stop();
        
        void onReceiveOdometry(const nav_msgs::Odometry &p_odometry);
        void onReceiveUltrasonicData(const minotaur_common::UltrasonicData &p_sensorData);
        
    Q_SIGNALS:
        void receivedOdometry(const QOdometry &p_msg);
        void receivedUltrasonicData(const QUltrasonicData &p_msg);
    };
}

#endif
