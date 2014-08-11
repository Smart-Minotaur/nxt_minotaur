#ifndef MINOTAUR_MINOTAUR_EXPLORATION_ALGORITHM_HPP
#define MINOTAUR_MINOTAUR_EXPLORATION_ALGORITHM_HPP

#include "minotaur_maze/ExplorationAlgorithm.hpp"

namespace minotaur
{
    /**
     * \brief The MinotaurExplorationAlgorithm class is an implementation of
     *        the ExplorationAlgorithm interface.
     * 
     * It calculates the next direction to take using the current pose
     * of the robot and the blocked properties of the MazeNode the robot
     * currently is in.
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
