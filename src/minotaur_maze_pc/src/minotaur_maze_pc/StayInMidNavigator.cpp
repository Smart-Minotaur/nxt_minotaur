#include <tf/transform_broadcaster.h>
#include <cmath>
#include "minotaur_maze_pc/StayInMidNavigator.hpp"
#include "robot_control_beagle/RAIILock.hpp"

#define MAX_LIN_VELOCITY 0.15f
#define MAX_ANG_VELOCITY 1.2f
#define SENSOR_MEDIAN_SIZE 5
#define ANG_VEL_FACTOR_MEDIAN_SIZE 10

#define THRESHOLD_FACTOR 0.8f
#define PARABEL_FACTOR 25.0f

#define MAX(a,b) (((a) > (b)) ? (a) : (b))
#define CM_TO_M(cm) (((float) (cm)) / 100.0f)

#define IS_FRONT_SENSOR(id) (sensorSettings[id].direction == 0)
#define IS_LEFT_SENSOR(id) (sensorSettings[id].direction > 0)
#define IS_RIGHT_SENSOR(id) (sensorSettings[id].direction < 0)

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
    
    StayInMidNavigator::StayInMidNavigator()
    :mode(WAITING), leftMedian(SENSOR_MEDIAN_SIZE), rightMedian(SENSOR_MEDIAN_SIZE), frontMedian(SENSOR_MEDIAN_SIZE), angVelFactorMedian(ANG_VEL_FACTOR_MEDIAN_SIZE)
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
        startTheta = normalizeAngle(tf::getYaw(p_odometry.pose.pose.orientation));
        mode = TURN;
    }
    
    bool StayInMidNavigator::reachedTargetTheta(const nav_msgs::Odometry &p_odometry)
    {
        float theta = normalizeAngle(tf::getYaw(p_odometry.pose.pose.orientation));
        
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
    void StayInMidNavigator::receivedUltrasonicData(const robot_control_beagle::UltrasonicData &p_sensorData)
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
    
    void StayInMidNavigator::checkFrontObstacle(const robot_control_beagle::UltrasonicData &p_sensorData)
    {
        if(IS_FRONT_SENSOR(p_sensorData.sensorID)) {
            // only recognize if it is close enough
            frontObstacle = frontMedian.value() <= getSensorDistanceThreshold();
        }
    }
    
    void StayInMidNavigator::updateDistances(const robot_control_beagle::UltrasonicData &p_sensorData)
    {
        // all data has to be in meter
        if(IS_FRONT_SENSOR(p_sensorData.sensorID))
            frontMedian.add(CM_TO_M(p_sensorData.distance));
        else if(IS_RIGHT_SENSOR(p_sensorData.sensorID))
            rightMedian.add(CM_TO_M(p_sensorData.distance));
        else if(IS_LEFT_SENSOR(p_sensorData.sensorID))
            leftMedian.add(CM_TO_M(p_sensorData.distance));
    }
    
    void StayInMidNavigator::setMovementVelocity(const robot_control_beagle::UltrasonicData &p_sensorData)
    {
        if(IS_FRONT_SENSOR(p_sensorData.sensorID)) {
            // if we are at the right position, stop the movement
            if(frontObstacle && obstacleIsCloseEnough(p_sensorData))
                stopMovement();
            return;
        }
        
        // it is no front sensor
        // sensors should be mirrored
        float sensorOffset = getSensorOffset(p_sensorData.sensorID);
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
            
        if(angVelFactor > 1)
            angVelFactor = 1;
        if(angVelFactor < -1)
            angVelFactor = -1;
        
        // use median filter to prevent robot from turning too much    
        if(angVelFactor != 0)
            angVelFactorMedian.add(angVelFactor);
        angVelFactor = angVelFactor - angVelFactorMedian.value();
        
        float angVelocity = angVelFactor * MAX_ANG_VELOCITY;
        float linVelocity = (1 - fabs(angVelFactor)) * MAX_LIN_VELOCITY;
        
        controlNode->setVelocity(linVelocity, angVelocity);
    }
    
    bool StayInMidNavigator::obstacleIsCloseEnough(const robot_control_beagle::UltrasonicData &p_sensorData)
    {
        // calculate distance of sensor to robot center
        float sensorOffset = getSensorOffset(p_sensorData.sensorID);
        
        // calculate distance at which robot should stop
        float maxDistanceToObstalce;
        if(isMovingVertically())
            maxDistanceToObstalce = (map->getNodeHeight() / 2) - sensorOffset;
        else
            maxDistanceToObstalce = (map->getNodeWidth() / 2) - sensorOffset;

        return frontMedian.value() <= maxDistanceToObstalce;
    }
    
    float StayInMidNavigator::calcAngularVelocityFactor(const float p_distanceDiff)
    {
        return PARABEL_FACTOR * (p_distanceDiff * fabs(p_distanceDiff));
    }
    
    void StayInMidNavigator::setTurnVelocity(const robot_control_beagle::UltrasonicData &p_sensorData)
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
        controlNode->setVelocity(0,0);
        mode = WAITING;
        pthread_cond_signal(&condition);
    }
    
    float StayInMidNavigator::getSensorDistanceThreshold()
    {
        return MAX(map->getNodeHeight(), map->getNodeWidth()) * THRESHOLD_FACTOR;
    }
    
    float StayInMidNavigator::getSensorOffset(const int p_id)
    {
        return fabs(sensorSettings[p_id].y);
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
        mode = WAITING;
        pthread_cond_signal(&condition);
    }
        
}
