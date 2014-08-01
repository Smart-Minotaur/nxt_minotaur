#ifndef MINOTAUR_IMAZE_MAPPING_HPP
#define MINOTAUR_IMAZE_MAPPING_HPP

#include "minotaur_maze/MazeMapUser.hpp"
#include "minotaur_maze/MinotaurControlNodeUser.hpp"

namespace minotaur
{
    class MazeMapping : public MazeMapUser, public MinotaurControlNodeUser
    {
    public:
        virtual void receivedUltrasonicData(const minotaur_common::UltrasonicData &p_sensorData) = 0;
        
        virtual void mapNode(const unsigned int p_x, const unsigned int p_y, const Direction p_direction) = 0;
    };
}

#endif
