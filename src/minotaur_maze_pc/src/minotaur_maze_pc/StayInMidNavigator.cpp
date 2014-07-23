#include <tf/transform_broadcaster.h>
#include <cmath>
#include "minotaur_maze_pc/StayInMidNavigator.hpp"
#include "robot_control_beagle/RAIILock.hpp"

#define MAX_LIN_VELOCITY 0.15f
#define MAX_ANG_VELOCITY 1.2f

#define MAX(a,b) (((a) > (b)) ? (a) : (b))
#define CM_TO_M(cm) (((float) (cm)) / 100.0f)
#define POSITION_EPSILON 0.05f
#define DIRECTION_EPSILON 0.1f

namespace minotaur
{
    static float normalizeAngle(const float p_angle)
    {
        float result = p_angle;
        while(result < -(2 * M_PI))
            result += (2 * M_PI);
        while(result > (2 * M_PI))
            result -= (2 * M_PI);
        return result;
    }
    
    static bool sameFloat(float a, float b, float eps)
    {
        return fabs(a - b) < eps;
    }
    
    /* No race condition between receivedOdometry() and receivedUltrasonicData()
     * because ros::spin() executes  in the same thread. */
    void StayInMidNavigator::receivedOdometry(const nav_msgs::Odometry &p_odometry)
    {
        if(mode == BEGIN_MOVE) {
            setTargetPosition(p_odometry);
            mode = MOVE;
        }
        
        if(mode == MOVE && !frontObstacle) {
            if(reachedTargetPosition(p_odometry))
                stopMovement();
        }
        
        if(mode == BEGIN_TURN) {
            setTargetTheta(p_odometry);
            mode = TURN;
        }
        
        if(mode == TURN)
            if(reachedTargetTheta(p_odometry))
                stopMovement();
    }
    
    void StayInMidNavigator::setTargetPosition(const nav_msgs::Odometry &p_odometry)
    {
        //determine the distance until the robot is in the next cell
        float distanceToNextCell;
        if(isMovingVertically())
            distanceToNextCell = map->getNodeHeight();
        else
            distanceToNextCell = map->getNodeWidth();
        
        // get current direction of robot
        float theta = tf::getYaw(p_odometry.pose.pose.orientation);
        
        // determine target position
        targetX = p_odometry.pose.pose.position.x + cos(theta) * distanceToNextCell;
        targetY = p_odometry.pose.pose.position.y + sin(theta) * distanceToNextCell;
    }
    
    bool StayInMidNavigator::reachedTargetPosition(const nav_msgs::Odometry &p_odometry)
    {
        return sameFloat(targetX, p_odometry.pose.pose.position.x, POSITION_EPSILON) &&
               sameFloat(targetY, p_odometry.pose.pose.position.y, POSITION_EPSILON);
    }
    
    void StayInMidNavigator::setTargetTheta(const nav_msgs::Odometry &p_odometry)
    {
        float theta = tf::getYaw(p_odometry.pose.pose.orientation);
        int directionDiff = getDirectionDiff(currentDirection, targetDirection);
        
        targetTheta = theta + ((directionDiff * M_PI) / 2);
        targetTheta = normalizeAngle(targetTheta);
    }
    
    bool StayInMidNavigator::reachedTargetTheta(const nav_msgs::Odometry &p_odometry)
    {
        float theta = tf::getYaw(p_odometry.pose.pose.orientation);
        theta = normalizeAngle(theta);
        return sameFloat(targetTheta, theta, DIRECTION_EPSILON);
    }
        
    /* No race condition between receivedOdometry() and receivedUltrasonicData()
     * because ros::spin() executes everything in the same thread. */
    void StayInMidNavigator::receivedUltrasonicData(const robot_control_beagle::UltrasonicData &p_sensorData)
    {
        // check if there is a front obstacle
        checkFrontObstacle(p_sensorData);
        updateDistances(p_sensorData);
        
        if(mode == MOVE)
            setMovementVelocity(p_sensorData);
        if(mode == TURN)
            setTurnVelocity(p_sensorData);
    }
    
    void StayInMidNavigator::checkFrontObstacle(const robot_control_beagle::UltrasonicData &p_sensorData)
    {
        if(isFrontSensor(p_sensorData.sensorID)) {
            // only recognize if it is close enough
            frontObstacle = CM_TO_M(p_sensorData.distance) <= getSensorDistanceThreshold();
        }
    }
    
    void StayInMidNavigator::updateDistances(const robot_control_beagle::UltrasonicData &p_sensorData)
    {
        if(sensorSettings[p_sensorData.sensorID].direction < 0) {
            // right sensor, we need data in meter
            rightDistance = CM_TO_M(p_sensorData.distance);
        } else if(sensorSettings[p_sensorData.sensorID].direction > 0) {
            // left sensor, we need data in meter
            leftDistance = CM_TO_M(p_sensorData.distance);
        }
    }
    
    void StayInMidNavigator::setMovementVelocity(const robot_control_beagle::UltrasonicData &p_sensorData)
    {
        // if we are at the right position, stop the movement
        if(frontObstacle && obstacleIsCloseEnough(p_sensorData)) {
                stopMovement();
                return;
        }
        
        if(isFrontSensor(p_sensorData.sensorID))
            return;
        
        // sensors should be mirrored
        float sensorOffset = getSensorOffset(p_sensorData.sensorID);
        
        //calculate threshold for sensor distances
        float distanceThreshold = getSensorDistanceThreshold();
        
        // calculate distance that should be kept
        float distanceToHold;
        if(isMovingVertically())
            distanceToHold = map->getNodeWidth() / 2 - sensorOffset;
        else
            distanceToHold = map->getNodeHeight() / 2 - sensorOffset;
            
        // set velocities depending on distance to obstacles left and right
        float angVelFactor = 0;
        int distanceCount = 0;
        if(leftDistance <= distanceThreshold) {
            angVelFactor += ((leftDistance / distanceToHold) - 1);
            distanceCount++;
        }
        if(rightDistance <= distanceThreshold) {
            angVelFactor += (1 - (rightDistance / distanceToHold));
            distanceCount++;
        }
        if(distanceCount != 0)
            angVelFactor /= distanceCount;
        
        if(angVelFactor > 1)
            angVelFactor = 1;
        if(angVelFactor < -1)
            angVelFactor = -1;
        
        float angVelocity = angVelFactor * MAX_ANG_VELOCITY;
        float linVelocity = (1 - fabs(angVelFactor)) * MAX_LIN_VELOCITY;
        
        ROS_INFO("AngVelFac: %.2f AngVel: %.2f LinVel: %.2f", angVelFactor, angVelocity, linVelocity);
        ROS_INFO("DistToHold: %.2f Threshold: %.2f", distanceToHold, distanceThreshold);
        ROS_INFO("Right: %.2f Left: %.2f Offset: %.2f\n", rightDistance, leftDistance, sensorOffset);
        
        controlNode->setVelocity(linVelocity, angVelocity);
    }
    
    bool StayInMidNavigator::obstacleIsCloseEnough(const robot_control_beagle::UltrasonicData &p_sensorData)
    {
        if(!isFrontSensor(p_sensorData.sensorID))
            return false;
        
        // calculate distance of sensor to robot center
        float sensorOffset = getSensorOffset(p_sensorData.sensorID);
        
        // calculate distance at which robot should stop
        float maxDistanceToObstalce;
        if(isMovingVertically())
            maxDistanceToObstalce = (map->getNodeHeight() / 2) - sensorOffset;
        else
            maxDistanceToObstalce = (map->getNodeWidth() / 2) - sensorOffset;

        return CM_TO_M(p_sensorData.distance) <= maxDistanceToObstalce;
    }
    
    void StayInMidNavigator::setTurnVelocity(const robot_control_beagle::UltrasonicData &p_sensorData)
    {
        int directionDiff = getDirectionDiff(currentDirection, targetDirection);
        
        float angVelocity = (directionDiff / abs(directionDiff)) * MAX_ANG_VELOCITY;
        
        controlNode->setVelocity(0, angVelocity);
    }
    
    bool StayInMidNavigator::isFrontSensor(int p_id)
    {
        return sensorSettings[p_id].direction == 0;
    }
    
    bool StayInMidNavigator::isMovingVertically()
    {
        return currentDirection == NORTH || currentDirection == SOUTH;
    }
    
    void StayInMidNavigator::stopMovement()
    {
        controlNode->setVelocity(0,0);
        mode = WAITING;
        pthread_cond_signal(&condition);
    }
    
    float StayInMidNavigator::getSensorDistanceThreshold()
    {
        return MAX(map->getNodeHeight(), map->getNodeWidth()) * 0.7f;
    }
    
    float StayInMidNavigator::getSensorOffset(int p_id)
    {
        return fabs(sensorSettings[p_id].y);
    }
    
    void StayInMidNavigator::moveToNextNode(Direction p_currentDirection)
    {
        RAIILock lock(&mutex);
        currentDirection = p_currentDirection;
        mode = BEGIN_MOVE;
        pthread_cond_wait(&condition, &mutex);
    }
    void StayInMidNavigator::turnRobotTo(Direction p_currentDirection, Direction p_newDirection)
    {
        RAIILock lock(&mutex);
        currentDirection = p_currentDirection;
        targetDirection = p_newDirection;
        mode = BEGIN_TURN;
        pthread_cond_wait(&condition, &mutex);
    }
    
    void StayInMidNavigator::shutdown()
    {
        RAIILock lock(&mutex);
        mode = WAITING;
        pthread_cond_signal(&condition);
    }
        
}
