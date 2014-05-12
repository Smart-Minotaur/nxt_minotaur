#include "nxt_beagle/RobotCommunicator.hpp"
#include "nxt_beagle/Config.hpp"
#include "nxt_control/NxtOpcodes.hpp"

#define WHEEL_TRACK 0.12f
#define WHEEL_CIRCUMFERENCE 0.16f
#define LEFT_PORT PORT_A
#define RIGHT_PORT PORT_B

namespace minotaur
{
    RobotCommunicator::RobotCommunicator()
    {
        pthread_mutex_init(&robotMutex, NULL);
    }
    
    void RobotCommunicator::init(ros::NodeHandle &p_handle, nxtcon::Brick *p_brick)
    {
        leftMotor.setBrick(p_brick);
        leftMotor.setPort(LEFT_PORT);
        rightMotor.setBrick(p_brick);
        rightMotor.setPort(RIGHT_PORT);
        
        ROS_INFO("Setting up RobotController...");
        robotController.setWheelTrack(WHEEL_TRACK);
        robotController.getPIDController().setWheelCircumference(WHEEL_CIRCUMFERENCE);
        robotController.getPIDController().setLeftMotor(&leftMotor);
        robotController.getPIDController().setRightMotor(&rightMotor);
        
        ROS_INFO("Subscribing to topic \"%s\"...", NXT_SET_PID_PARAMETER);
        setPIDParamSub = p_handle.subscribe(NXT_SET_PID_PARAMETER, 100, &RobotCommunicator::processPIDParamMsg, this);
        ROS_INFO("Subscribing to topic \"%s\"...", ROS_VEL_TOPIC);
        cmdVelSub = p_handle.subscribe(ROS_VEL_TOPIC, 100, &RobotCommunicator::processSetVelocityMsg, this);
        
        ROS_INFO("Publishing on topic \"%s\"...", ROS_ODOM_TOPIC);
        odometryPub = p_handle.advertise<nav_msgs::Odometry>(ROS_ODOM_TOPIC, 100);
    }
    
    void RobotCommunicator::setTransformBroadcaster(tf::TransformBroadcaster *p_odomBroadcaster)
    {
        odomBroadcaster = p_odomBroadcaster;
    }
        
    void RobotCommunicator::publish()
    {
        nav_msgs::Odometry odom = robotController.getOdometry();
        ros::Time currentTime = ros::Time::now();
        
        // create Transformation and send it
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = currentTime;
        odom_trans.header.frame_id = MINOTAUR_ODOM_FRAME;
        odom_trans.child_frame_id = MINOTAUR_BASE_FRAME;

        odom_trans.transform.translation.x = odom.pose.pose.position.x;
        odom_trans.transform.translation.y = odom.pose.pose.position.y;
        odom_trans.transform.translation.z = odom.pose.pose.position.z;
        odom_trans.transform.rotation = odom.pose.pose.orientation;

        odomBroadcaster->sendTransform(odom_trans);
        
        // prepare odometry message and send it
        odom.header.stamp = currentTime;
        odom.header.frame_id = MINOTAUR_ODOM_FRAME;
        odom.child_frame_id = MINOTAUR_BASE_FRAME;
        
        odometryPub.publish(odom);
    }
    
    RobotController& RobotCommunicator::getRobotController()
    {
        return robotController;
    }
    
    /* callbacks */
    void RobotCommunicator::processSetVelocityMsg(const geometry_msgs::Twist& p_msg)
    {
        lock();
        //always try catch between mutex lock / unlock to prevent deadlock
        try
        {
            robotController.setVelocity(p_msg);
        }
        catch(std::exception const &e)
        {
            ROS_ERROR("SetRobotVelocity: %s.", e.what());
        }
        unlock();
    }
    
    void RobotCommunicator::processPIDParamMsg(const nxt_beagle::PIDParam& p_msg)
    {
        minotaur::PIDParameter params(p_msg.Kp, p_msg.Ki, p_msg.Kd);
        ROS_INFO("Changing PIDParameter to: Kp = %.4f; Ki = %.4f; Kd = %.4f.",params.Kp, params.Ki, params.Kd);  
        
        lock();
        //always try catch between mutex lock / unlock to prevent deadlock
        try
        {
            robotController.getPIDController().setPIDParameter(params);
        }
        catch(std::exception const &e)
        {
            ROS_ERROR("SetPIDParam: %s.", e.what());
        }
        
        unlock();
    }
}