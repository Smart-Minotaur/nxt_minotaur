#ifndef MINOTAUR_MINOTAUR_EXPLORATION_ALGORITHM_HPP
#define MINOTAUR_MINOTAUR_EXPLORATION_ALGORITHM_HPP

#include "minotaur_maze/ExplorationAlgorithm.hpp"

namespace minotaur
{
    /**
     * \brief The MinotaurExplorationAlgorithm calculates the decisions
     *        of the robot in the maze.
     */
    class MinotaurExplorationAlgorithm: public ExplorationAlgorithm
    {
    public:
        MinotaurExplorationAlgorithm();
        ~MinotaurExplorationAlgorithm();
        
        Direction calculateMovementDirection(unsigned int p_x, unsigned int p_y, Direction p_direction);
    };
}

#endif
