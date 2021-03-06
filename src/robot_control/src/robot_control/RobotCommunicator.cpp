#include <nxt/NXTControl.hpp>
#include "robot_control/RobotCommunicator.hpp"
#include "minotaur_common/MinotaurTopics.hpp"
#include "minotaur_common/RAIILock.hpp"
#include "minotaur_common/Math.hpp"

#define WHEEL_TRACK 0.12f
#define WHEEL_RADIUS 0.025f
#define LEFT_PORT PORT_A
#define RIGHT_PORT PORT_B

#define CHECK_BRICK_INTERVAL (2 * MSEC_PER_SEC)
#define TONE_FREQUENCY 1000
#define TONE_DURATION_MS 1000
#define MIN_VOLTAGE_MV 6100 

#define ROS_MSG_QUEUE_LENGTH 20

namespace minotaur
{
    RobotCommunicator::RobotCommunicator(nxt::Brick *p_brick)
    : statusMsec(0), brick(p_brick), leftMotor(brick, LEFT_PORT), rightMotor(brick, RIGHT_PORT), robotController(leftMotor, rightMotor)
    {
        pthread_mutex_init(&robotMutex, NULL);
    }
    
    RobotCommunicator::~RobotCommunicator()
    {
        pthread_mutex_destroy(&robotMutex);
    }
    
    void RobotCommunicator::init(ros::NodeHandle &p_handle)
    {
        RAIILock lock(&robotMutex);
        
        ROS_INFO("-- Setting up PIDController.");
        robotController.getPIDController().setWheelTrack(WHEEL_TRACK);
        robotController.getPIDController().setWheelRadius(WHEEL_RADIUS);
        
        ROS_INFO("-- Subscribing to topic \"%s\".", MINOTAUR_SET_PID_PARAMETER_TOPIC);
        setPIDParamSub = p_handle.subscribe(MINOTAUR_SET_PID_PARAMETER_TOPIC, ROS_MSG_QUEUE_LENGTH, &RobotCommunicator::processPIDParamMsg, this);
        ROS_INFO("-- Subscribing to topic \"%s\".", ROS_VEL_TOPIC);
        cmdVelSub = p_handle.subscribe(ROS_VEL_TOPIC, ROS_MSG_QUEUE_LENGTH, &RobotCommunicator::processSetVelocityMsg, this);
        
        ROS_INFO("-- Publishing on topic \"%s\".", ROS_ODOM_TOPIC);
        odometryPub = p_handle.advertise<nav_msgs::Odometry>(ROS_ODOM_TOPIC, ROS_MSG_QUEUE_LENGTH);
    }
    
    void RobotCommunicator::setTransformBroadcaster(tf::TransformBroadcaster *p_odomBroadcaster)
    {
        RAIILock lock(&robotMutex);
        odomBroadcaster = p_odomBroadcaster;
    }
        
    void RobotCommunicator::publish()
    {
        RAIILock lock(&robotMutex);
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
    
    void RobotCommunicator::stepController(const int p_samplingIntervalMsec)
    {
        RAIILock lock(&robotMutex);
        robotController.step(p_samplingIntervalMsec);
        checkBrickStatus(p_samplingIntervalMsec);
    }
    
    void RobotCommunicator::checkBrickStatus(const int p_samplingIntervalMsec)
    {
        statusMsec += p_samplingIntervalMsec;
        if(statusMsec >= CHECK_BRICK_INTERVAL) {
            statusMsec = 0;
            
            uint16_t voltage = brick->getBatteryLevel().voltage;
            brick->playTone(TONE_FREQUENCY, TONE_DURATION_MS);
            
            if(voltage < MIN_VOLTAGE_MV)
                ROS_WARN("Low Brick Battery: %d.", voltage);
        }
    }
    
    /* callbacks */
    void RobotCommunicator::processSetVelocityMsg(const geometry_msgs::Twist& p_msg)
    {
        RAIILock lock(&robotMutex);
        robotController.setVelocity(p_msg);
    }
    
    void RobotCommunicator::processPIDParamMsg(const minotaur_common::PIDParameter& p_msg)
    {
        ROS_INFO("Changing PIDParameter to: Kp = %.4f; Ki = %.4f; Kd = %.4f.",p_msg.Kp, p_msg.Ki, p_msg.Kd);  
        
        RAIILock lock(&robotMutex);
        robotController.getPIDController().setPIDParameter(p_msg);
    }
    
    void RobotCommunicator::applySettings(const RobotSettings &p_settings)
    {
        RAIILock lock(&robotMutex);
        robotController.getPIDController().setPIDParameter(p_settings.pidParameter);
        robotController.getPIDController().setWheelRadius(p_settings.wheelRadius);
        robotController.getPIDController().setWheelTrack(p_settings.wheelTrack);
    }
	
	void RobotCommunicator::applySettings(const MouseSensorSettings &p_settings)
	{
		RAIILock lock(&robotMutex);
		robotController.getPIDController().setMouseSensorSettings(p_settings);
	}
    
    void RobotCommunicator::shutdown()
    {
        geometry_msgs::Twist velocity;
        initTwist(velocity);
        robotController.setVelocity(velocity);
    }
}
