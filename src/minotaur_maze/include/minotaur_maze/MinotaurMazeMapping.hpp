#ifndef MINOTAUR_MAZE_MAPPING_HPP
#define MINOTAUR_MAZE_MAPPING_HPP

#include <vector>
#include <pthread.h>
#include "minotaur_maze/MazeMapping.hpp"

namespace minotaur
{
    /**
     * \brief The MinotaurMazeMapping class is an implementation of
     *        the MazeMapping interface.
     * 
     * It processes incoming UltrasonicData messages and calculates the
     * blocked properties of the MazeNode, in which the robot currently
     * is in. The robot is not moving during this procedure.
     */
    class MinotaurMazeMapping: public MazeMapping
    {
    private:
        pthread_mutex_t mutex;
        pthread_cond_t condition;
    
    
        volatile bool measure;
        std::vector<bool> leftObstacle, rightObstacle, frontObstacle;
        int timeout;
        
        int currentX, currentY;
        Direction currentDirection;
        
        void processSensorData(const minotaur_common::UltrasonicData &p_sensorData);
        bool hasEnoughMeasurements();
        void stopMapping();
        void evaluateMeasurements();
        void setBlocked(std::vector<bool> &p_hasObstacle, const Direction p_direction);
        float getSensorDistanceThreshold();
    public:
        MinotaurMazeMapping();
        ~MinotaurMazeMapping();
        
        void receivedUltrasonicData(const minotaur_common::UltrasonicData &p_sensorData);
        void mapNode(const unsigned int p_x, const unsigned int p_y, const Direction p_direction);
    };
}

#endif
