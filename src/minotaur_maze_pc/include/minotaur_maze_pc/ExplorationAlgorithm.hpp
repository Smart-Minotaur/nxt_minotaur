#ifndef MINOTAUR_IEXPLORATION_ALGORITHM_HPP
#define MINOTAUR_IEXPLORATION_ALGORITHM_HPP

#include "minotaur_maze_pc/MazeMapUser.hpp"

namespace minotaur
{
    
    class ExplorationAlgorithm: public MazeMapUser
    {
    public:
        virtual Direction calculateMovementDirection(unsigned int p_x, unsigned int p_y, Direction p_direction) = 0;
    };
}

#endif
