#include "minotaur_maze/MinotaurMazeMapping.hpp"
#include "minotaur_common/RAIILock.hpp"

#define MIN_MEASUREMENTS 10
#define MAX_TIMEOUT 5
#define IS_FRONT_SENSOR(dir) (dir == 0)
#define IS_LEFT_SENSOR(dir) (dir > 0)
#define IS_RIGHT_SENSOR(dir) (dir < 0)

#define CM_TO_M(cm) (((float) (cm)) / 100.0f)
#define MAX(a,b) (((a) > (b)) ? (a) : (b))

namespace minotaur
{
    MinotaurMazeMapping::MinotaurMazeMapping()
    : leftObstacle(MIN_MEASUREMENTS), rightObstacle(MIN_MEASUREMENTS), frontObstacle(MIN_MEASUREMENTS), measure(false)
    { }
    
    MinotaurMazeMapping::~MinotaurMazeMapping()
    { }
    
    void MinotaurMazeMapping::receivedUltrasonicData(const minotaur_common::UltrasonicData &p_sensorData)
    {
        if(measure)
            processSensorData(p_sensorData);
    }
    
    void MinotaurMazeMapping::processSensorData(const minotaur_common::UltrasonicData &p_sensorData)
    {
        RAIILock lock(&mutex);
        float sensorDir = sensorSettings[p_sensorData.sensorID].direction;
        std::vector<bool> *obstacle;
        
        if(timeout == MAX_TIMEOUT || hasEnoughMeasurements())
            stopMapping();
        
        if(IS_FRONT_SENSOR(sensorDir))
            obstacle = &frontObstacle;
        else if(IS_LEFT_SENSOR(sensorDir))
            obstacle = &leftObstacle;
        else if(IS_RIGHT_SENSOR(sensorDir))
            obstacle = &rightObstacle;
        else {
            ++timeout;
            return;
        }
        
        if(obstacle->size() >= MIN_MEASUREMENTS)
            ++timeout;
        else
            timeout = 0;
        
        
        float distance = CM_TO_M(p_sensorData.distance);
        obstacle->push_back(distance <= getSensorDistanceThreshold());
    }
    
    bool MinotaurMazeMapping::hasEnoughMeasurements()
    {
        return leftObstacle.size() >= MIN_MEASUREMENTS && rightObstacle.size() >= MIN_MEASUREMENTS && frontObstacle.size() >= MIN_MEASUREMENTS;
    }
    
    void MinotaurMazeMapping::stopMapping()
    {
        evaluateMeasurements();
        measure = false;
        pthread_cond_signal(&condition);
    }
    
    void MinotaurMazeMapping::evaluateMeasurements()
    {
        int blocked;
        Direction direction;
        
        blocked = 0;
        for(int i = 0; i < frontObstacle.size(); ++i)
            if(frontObstacle[i])
                ++blocked;
        if(blocked > frontObstacle.size() / 2)
            map->node(currentX, currentY).setBlocked(currentDirection, true);
            
        direction = turnDirection(currentDirection, 1);
        blocked = 0;
        
        for(int i = 0; i < leftObstacle.size(); ++i)
            if(leftObstacle[i])
                ++blocked;
        if(blocked > leftObstacle.size() / 2)
            map->node(currentX, currentY).setBlocked(direction, true);
            
        direction = turnDirection(currentDirection, -1);
        blocked = 0;
        
        for(int i = 0; i < rightObstacle.size(); ++i)
            if(rightObstacle[i])
                ++blocked;
        if(blocked > rightObstacle.size() / 2)
            map->node(currentX, currentY).setBlocked(direction, true);
    }
    
    float MinotaurMazeMapping::getSensorDistanceThreshold()
    {
        return MAX(map->getNodeHeight(), map->getNodeWidth()) * 0.7f;
    }
        
    void MinotaurMazeMapping::mapNode(unsigned int p_x, unsigned int p_y, Direction p_direction)
    {
        RAIILock lock(&mutex);
        frontObstacle.clear();
        leftObstacle.clear();
        rightObstacle.clear();
        
        currentX = p_x;
        currentY = p_y;
        currentDirection = p_direction;
        measure = true;
        
        pthread_cond_wait(&condition, &mutex);
    }
}
