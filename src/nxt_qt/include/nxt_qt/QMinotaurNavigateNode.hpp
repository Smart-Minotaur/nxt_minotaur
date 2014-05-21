#ifndef QPID_NODE_HPP_
#define QPID_NODE_HPP_

#include <string>
#include <ros/ros.h>
#include <QThread>
#include <QMetaType>
#include <QMutex>
#include <vector>
#include "nxt_beagle/MotorVelocity.hpp"
#include "nxt_beagle/UltraSensor.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"

namespace minotaur
{
    class QOdometry
    {
    public:
        float linVelX;
        float linVelY;
        float angVel;
        
        float sigmaVel[2][2];
        
        float x;
        float y;
        float theta;
        
        float sigmaPos[3][3];
        
        QOdometry() { }
        virtual ~QOdometry() { }
    };
    
    class QUltraSensor
    {
    public:
        int id;
        int distance;
        float direction;
        
        QUltraSensor() { }
        virtual ~QUltraSensor() { }
    };
    
    class QMinotaurNavigateNode : public QThread
    {
        Q_OBJECT
        
    private:
        QMutex mutex;
        
        std::vector<float> directions;
        
        ros::NodeHandle nodeHandle;
        
        ros::Subscriber odometrySub;
        ros::Subscriber ultrasensorSub;
        
        ros::Publisher robotVelocityPublisher;
        ros::Publisher pidPramPublisher;
        
        nav_msgs::Odometry lastOdometry;
        
        void processOdometryMsg(const nav_msgs::Odometry& p_msg);
        void processSensorMsg(const nxt_beagle::UltraSensor p_msg);
    public:
        QMinotaurNavigateNode();
        virtual ~QMinotaurNavigateNode() { }
        
        void setRobotVelocity(const float p_linVel, const float p_angVel);
        void setPIDParameter(const float p_Kp, const float p_Ki, const float p_Kd);
        
        void run();
        
    Q_SIGNALS:
        void rosShutdown();
        void odometryUpdated(const QOdometry p_msg);
        void measuredSensor(const QUltraSensor p_msg);
    };
}

Q_DECLARE_METATYPE(minotaur::QOdometry);
Q_DECLARE_METATYPE(minotaur::QUltraSensor);

#endif