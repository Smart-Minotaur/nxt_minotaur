#ifndef MINOTAUR_MAZE_MAPPING_HPP
#define MINOTAUR_MAZE_MAPPING_HPP

#include <vector>
#include <pthread.h>
#include "minotaur_maze_pc/MazeMapping.hpp"

namespace minotaur
{
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
        
        void processSensorData(const robot_control_beagle::UltrasonicData &p_sensorData);
        bool hasEnoughMeasurements();
        void stopMapping();
        float getSensorDistanceThreshold();
    public:
        MinotaurMazeMapping();
        ~MinotaurMazeMapping();
        
        void receivedUltrasonicData(const robot_control_beagle::UltrasonicData &p_sensorData);
        void mapNode(unsigned int p_x, unsigned int p_y, Direction p_direction);
    };
}

#endif
