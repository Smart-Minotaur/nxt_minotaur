#include "nxt_beagle/Config.hpp"
#include "nxt_qt/QPIDNode.hpp"
#include "nxt_beagle/Config.hpp"
#include "nxt_beagle/SamplingInterval.h"
#include "nxt_beagle/RVelocity.h"
#include "nxt_beagle/PIDParam.h"

using namespace nxt_beagle;

namespace minotaur
{
    QPIDNode::QPIDNode()
    {
        targetMVelSubscriber = nodeHandle.subscribe(NXT_TARGET_MOTOR_VELOCITY_TOPIC,
                                                    1000,
                                                    &QPIDNode::processTargetMotorVelocity,
                                                    this);
        measuredMVelSubscriber = nodeHandle.subscribe(NXT_MEASURE_MOTOR_VELOCITY_TOPIC,
                                                      1000,
                                                      &QPIDNode::processMeasuredMotorVelocity,
                                                      this);
        
        samplingIntervalPublisher = nodeHandle.advertise<SamplingInterval>(NXT_SET_SAMPLING_INTERVAL_TOPIC, 1000);
        robotVelocityPublisher = nodeHandle.advertise<RVelocity>(NXT_SET_ROBOT_VELOCITY_TOPIC, 1000);
        pidPramPublisher = nodeHandle.advertise<PIDParam>(NXT_SET_PID_PARAMETER, 1000);
    }
    
    void QPIDNode::setSamplingIntervall(const float p_sec)
    {
        SamplingInterval msg;
        msg.sec = p_sec;
        samplingIntervalPublisher.publish(msg);
    }
    
    void QPIDNode::setRobotVelocity(const float p_linVel, const float p_angVel)
    {
        RVelocity msg;
        msg.linearVelocity = p_linVel;
        msg.angularVelocity = p_angVel;
        robotVelocityPublisher.publish(msg);
    }
    
    void QPIDNode::setPIDParameter(const float p_Kp, const float p_Ki, const float p_Kd)
    {
        PIDParam msg;
        msg.Kp = p_Kp;
        msg.Ki = p_Ki;
        msg.Kd = p_Kd;
        pidPramPublisher.publish(msg);
    }
    
    void QPIDNode::run()
    {
        ros::spin();
        Q_EMIT rosShutdown();
    }
    
    void QPIDNode::processTargetMotorVelocity(const MVelocity& p_msg)
    {
       Q_EMIT targetMotorVelocityUpdated(p_msg);
    }
    
    void QPIDNode::processMeasuredMotorVelocity(const MVelocity& p_msg)
    {
        Q_EMIT measuredMotorVelocityUpdated(p_msg);
    }
}