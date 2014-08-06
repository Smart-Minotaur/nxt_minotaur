#ifndef MINOTAUR_IEXPLORATION_ALGORITHM_HPP
#define MINOTAUR_IEXPLORATION_ALGORITHM_HPP

#include "minotaur_maze/MazeMapUser.hpp"

namespace minotaur
{
    /**
     * \brief The ExplorationAlgorithm class provides an interface for
     *        the exploration logic of a MazeSolver object.
     */
    class ExplorationAlgorithm: public MazeMapUser
    {
    public:
    
        /**
         * Calculates the next direction the robot should
         * take considering its the current pose.
         * @param p_x current x positon of the robot
         * @param p_y current y position of the robot
         * @param p_direction current direction of the robot
         */
        virtual Direction calculateMovementDirection(unsigned int p_x, unsigned int p_y, Direction p_direction) = 0;
    };
}

#endif
