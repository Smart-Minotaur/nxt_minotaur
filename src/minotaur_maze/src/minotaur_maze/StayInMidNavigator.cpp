#include <cmath>
#include "minotaur_maze/StayInMidNavigator.hpp"
#include "minotaur_common/RAIILock.hpp"
#include "minotaur_common/Math.hpp"

#define MAX_LIN_VELOCITY 0.12f
#define MAX_ANG_VELOCITY 1.2f
#define SENSOR_MEDIAN_SIZE 5
#define ANG_VEL_MEDIAN_SIZE 7

#define THRESHOLD_FACTOR 0.7f
#define FUNCTION_FACTOR 5830.0f
#define X_AXIS_DISPLACEMENT 0.005f
#define Y_AXIS_DISPLACEMENT 0.0f

#define IS_FRONT_SENSOR(id) (sensorSettings[id].direction == 0)
#define IS_LEFT_SENSOR(id) (sensorSettings[id].direction > 0 && sensorSettings[id].direction < M_PI)
#define IS_RIGHT_SENSOR(id) (sensorSettings[id].direction < RAD_PER_CIRCLE && sensorSettings[id].direction > M_PI)

namespace minotaur
{
    StayInMidNavigator::StayInMidNavigator()
    :mode(WAITING), leftMedian(SENSOR_MEDIAN_SIZE), rightMedian(SENSOR_MEDIAN_SIZE), angVelFactorMedian(ANG_VEL_MEDIAN_SIZE)
    {
        pthread_mutex_init(&mutex, NULL);
        pthread_cond_init(&condition, NULL);
    }
    
    StayInMidNavigator::~StayInMidNavigator()
    {
        pthread_mutex_destroy(&mutex);
        pthread_cond_destroy(&condition);
    }
    
    /* No race condition between receivedOdometry() and receivedUltrasonicData()
     * because ros::spin() executes  in the same thread. */
    void StayInMidNavigator::receivedOdometry(const nav_msgs::Odometry &p_odometry)
    {
        RAIILock lock(&mutex);
        if(mode == BEGIN_MOVE)
            initMovement(p_odometry);
        
        if(mode == MOVE && !frontObstacle) {
            if(reachedTargetPosition(p_odometry))
                stopMovement();
        }
        
        if(mode == BEGIN_TURN)
            initTurning(p_odometry);
        
        if(mode == TURN)
            if(reachedTargetTheta(p_odometry))
                stopMovement();
    }
    
    void StayInMidNavigator::initMovement(const nav_msgs::Odometry &p_odometry)
    {
        startX = p_odometry.pose.pose.position.x;
        startY = p_odometry.pose.pose.position.y;
        mode = MOVE;
    }
    
    bool StayInMidNavigator::reachedTargetPosition(const nav_msgs::Odometry &p_odometry)
    {
        float diffX = p_odometry.pose.pose.position.x - startX;
        float diffY = p_odometry.pose.pose.position.y - startY;
        float distanceSq = diffX * diffX + diffY * diffY;
        float targetDistanceSq;
        
        if(isMovingVertically())
            targetDistanceSq = map->getNodeHeight() * map->getNodeHeight();
        else
            targetDistanceSq = map->getNodeWidth() * map->getNodeWidth();
            
        return distanceSq >= targetDistanceSq;
    }
    
    void StayInMidNavigator::initTurning(const nav_msgs::Odometry &p_odometry)
    {
        startTheta = getNormalizedTheta(p_odometry);
        mode = TURN;
    }
    
    bool StayInMidNavigator::reachedTargetTheta(const nav_msgs::Odometry &p_odometry)
    {
        float theta = getNormalizedTheta(p_odometry);
        
        int directionDiff = getDirectionDiff(currentDirection, targetDirection);
        
        if(directionDiff < 0)
            while(startTheta < theta)
                theta -= 2 * M_PI;
                
        if(directionDiff > 0)
            while(startTheta > theta)
                theta += 2 * M_PI;
        float diffTheta = theta - startTheta;
        return fabs(diffTheta) >= fabs(directionDiff * (M_PI / 2));
    }
        
    /* No race condition between receivedOdometry() and receivedUltrasonicData()
     * because ros::spin() executes everything in the same thread. */
    void StayInMidNavigator::receivedUltrasonicData(const minotaur_common::UltrasonicData &p_sensorData)
    {
        RAIILock lock(&mutex);
        updateDistances(p_sensorData);
        // check if there is a front obstacle
        checkFrontObstacle(p_sensorData);
        
        if(mode == MOVE)
            setMovementVelocity(p_sensorData);
        if(mode == TURN)
            setTurnVelocity(p_sensorData);
    }
    
    void StayInMidNavigator::updateDistances(const minotaur_common::UltrasonicData &p_sensorData)
    {
        // all data has to be in meter
        if(IS_FRONT_SENSOR(p_sensorData.sensorID))
            frontDistance = cmToMeter(p_sensorData.distance);
        else if(IS_RIGHT_SENSOR(p_sensorData.sensorID))
            rightMedian.add(cmToMeter(p_sensorData.distance));
        else if(IS_LEFT_SENSOR(p_sensorData.sensorID))
            leftMedian.add(cmToMeter(p_sensorData.distance));
    }
    
    void StayInMidNavigator::checkFrontObstacle(const minotaur_common::UltrasonicData &p_sensorData)
    {
        if(IS_FRONT_SENSOR(p_sensorData.sensorID)) {
            // only recognize if it is close enough
            frontObstacle = frontDistance <= getSensorDistanceThreshold();
        }
    }
    
    void StayInMidNavigator::setMovementVelocity(const minotaur_common::UltrasonicData &p_sensorData)
    {
        if(IS_FRONT_SENSOR(p_sensorData.sensorID)) {
            // if we are at the right position, stop the movement
            if(frontObstacle && obstacleIsCloseEnough(p_sensorData))
                stopMovement();
            return;
        }
        
        // it is no front sensor
        // sensors should be mirrored
        float sensorOffset = getHorizontalSensorOffset(p_sensorData.sensorID);
        // calculate distance that should be kept
        float distanceToHold;
        if(isMovingVertically())
            distanceToHold = map->getNodeWidth() / 2 - sensorOffset;
        else
            distanceToHold = map->getNodeHeight() / 2 - sensorOffset;
            
        //calculate threshold for sensor distances
        float distanceThreshold = getSensorDistanceThreshold();
        // get sensor distances
        float leftDistance = leftMedian.value();
        float rightDistance = rightMedian.value();
        
        // set velocities depending on distance to obstacles left and right
        float angVelFactor = 0;
        int distanceCount = 0;
        if(leftDistance <= distanceThreshold) {
            angVelFactor += calcAngularVelocityFactor(leftDistance - distanceToHold);
            distanceCount++;
        }
        if(rightDistance <= distanceThreshold) {
            angVelFactor += calcAngularVelocityFactor(distanceToHold - rightDistance);
            distanceCount++;
        }
        if(distanceCount > 1)
            angVelFactor /= distanceCount;
        
        /*if(distanceCount != 0)
            angVelFactorMedian.add(angVelFactor);
        if(!angVelFactorMedian.isEmpty())
            angVelFactor = angVelFactor - angVelFactorMedian.value();*/
            
        if(angVelFactor > 1)
            angVelFactor = 1;
        if(angVelFactor < -1)
            angVelFactor = -1;
            
        float angVelocity = angVelFactor * MAX_ANG_VELOCITY;
        float linVelocity = (1 - fabs(angVelFactor)) * MAX_LIN_VELOCITY;
        controlNode->setVelocity(linVelocity, angVelocity);
        
        ROS_INFO("Median: left=%.2fm right=%.2fm angVel=%.2f.", leftDistance, rightDistance, angVelocity);
    }
    
    bool StayInMidNavigator::obstacleIsCloseEnough(const minotaur_common::UltrasonicData &p_sensorData)
    {
        // calculate distance of sensor to robot center
        float sensorOffset = getVerticalSensorOffset(p_sensorData.sensorID);
        
        // calculate distance at which robot should stop
        float maxDistanceToObstalce;
        if(isMovingVertically())
            maxDistanceToObstalce = (map->getNodeHeight() / 2) - sensorOffset;
        else
            maxDistanceToObstalce = (map->getNodeWidth() / 2) - sensorOffset;

        return frontDistance <= maxDistanceToObstalce;
    }
    
    float StayInMidNavigator::calcAngularVelocityFactor(const float p_distanceDiff)
    {
        float xVal = fabs(p_distanceDiff) - X_AXIS_DISPLACEMENT;
        return ((p_distanceDiff / fabs(p_distanceDiff)) * FUNCTION_FACTOR * (xVal * xVal * xVal)) + Y_AXIS_DISPLACEMENT;
    }
    
    void StayInMidNavigator::setTurnVelocity(const minotaur_common::UltrasonicData &p_sensorData)
    {
        int directionDiff = getDirectionDiff(currentDirection, targetDirection);
        
        float angVelocity = (directionDiff / abs(directionDiff)) * MAX_ANG_VELOCITY;
        
        controlNode->setVelocity(0, angVelocity);
    }
    
    bool StayInMidNavigator::isMovingVertically()
    {
        return currentDirection == NORTH || currentDirection == SOUTH;
    }
    
    void StayInMidNavigator::stopMovement()
    {
        mode = WAITING;
        controlNode->setVelocity(0,0);
        pthread_cond_signal(&condition);
    }
    
    float StayInMidNavigator::getSensorDistanceThreshold()
    {
        return MAX(map->getNodeHeight(), map->getNodeWidth()) * THRESHOLD_FACTOR;
    }
    
    float StayInMidNavigator::getHorizontalSensorOffset(const int p_id)
    {
        return fabs(sensorSettings[p_id].y);
    }
    
    float StayInMidNavigator::getVerticalSensorOffset(const int p_id)
    {
        return fabs(sensorSettings[p_id].x);
    }
    
    void StayInMidNavigator::moveToNextNode(const Direction p_currentDirection)
    {
        RAIILock lock(&mutex);
        currentDirection = p_currentDirection;
        mode = BEGIN_MOVE;
        pthread_cond_wait(&condition, &mutex);
    }
    void StayInMidNavigator::turnRobotTo(const Direction p_currentDirection, const Direction p_newDirection)
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
        stopMovement();
    }
        
}
