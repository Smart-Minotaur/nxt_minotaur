#ifndef MINOTAUR_STAY_IN_MID_NAVIGATOR_HPP
#define MINOTAUR_STAY_IN_MID_NAVIGATOR_HPP

#include <pthread.h>
#include "minotaur_maze/MazeNavigator.hpp"
#include "minotaur_common/MedianFilter.hpp"

namespace minotaur
{
    enum MovementMode { WAITING, BEGIN_MOVE, MOVE, BEGIN_TURN, TURN };
    
    /**
     * \brief The StayInMidNavigator class is an implementation of
     *        the MazeNavigator interface.
     * 
     * It processes incoming UltrasonicData and Odometry message.
     * Depending on the results the robot tries to stay in the mid
     * between the walls on the left and right when he moves to the next
     * node.
     */
    class StayInMidNavigator : public MazeNavigator
    {
    private:
        pthread_mutex_t mutex;
        pthread_cond_t condition;
        
        Direction currentDirection;
        Direction targetDirection;
        
        MedianFilter leftMedian, rightMedian, angVelFactorMedian;
        float frontDistance;
        
        volatile MovementMode mode;
        volatile bool frontObstacle;
        
        float startX, startY;
        float startTheta;
        
        void initMovement(const nav_msgs::Odometry &p_odometry);
        bool reachedTargetPosition(const nav_msgs::Odometry &p_odometry);
        void initTurning(const nav_msgs::Odometry &p_odometry);
        bool reachedTargetTheta(const nav_msgs::Odometry &p_odometry);
        
        void checkFrontObstacle(const minotaur_common::UltrasonicData &p_sensorData);
        void updateDistances(const minotaur_common::UltrasonicData &p_sensorData);
        
        void setMovementVelocity(const minotaur_common::UltrasonicData &p_sensorData);
        bool obstacleIsCloseEnough(const minotaur_common::UltrasonicData &p_sensorData);
        float calcAngularVelocityFactor(const float p_distanceDiff);
		float linearFunction(const float p_x);
        
        void setTurnVelocity(const minotaur_common::UltrasonicData &p_sensorData);
        
        bool isMovingVertically();
        float getSensorDistanceThreshold();
        float getHorizontalSensorOffset(const int p_id);
        float getVerticalSensorOffset(const int p_id);
     
        void stopMovement();
    public:
        StayInMidNavigator();
        ~StayInMidNavigator();
    
        void receivedOdometry(const nav_msgs::Odometry &p_odometry);
        void receivedUltrasonicData(const minotaur_common::UltrasonicData &p_sensorData) ;
        
        void moveToNextNode(const Direction p_currentDirection);
        void turnRobotTo(const Direction p_currentDirection, const Direction p_newDirection);
        
        void shutdown();
    };
}

#endif
