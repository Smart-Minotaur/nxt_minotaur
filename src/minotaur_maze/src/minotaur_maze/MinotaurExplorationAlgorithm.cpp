#include "minotaur_maze/MinotaurExplorationAlgorithm.hpp"

namespace minotaur
{
    MinotaurExplorationAlgorithm::MinotaurExplorationAlgorithm()
    {
        
    }
    
    MinotaurExplorationAlgorithm::~MinotaurExplorationAlgorithm()
    {
        
    }
    
    Direction MinotaurExplorationAlgorithm::calculateMovementDirection(unsigned int p_x, unsigned int p_y, Direction p_direction)
    {
        Direction rightDirection = turnDirection(p_direction, 1);
        Direction leftDirection = turnDirection(p_direction, -1);
        Direction backDirection = turnDirection(p_direction, 2);
        Direction result;
        
        if(!map->node(p_x, p_y).isBlocked(rightDirection)) {
            // go to right because it is not blocked
            result = rightDirection;
        } else if(!map->node(p_x, p_y).isBlocked(leftDirection)) {
            // go to left
            result = leftDirection;
        } else if(!map->node(p_x, p_y).isBlocked(p_direction)) {
            // got straight forward
            result = p_direction;
        } else {
            // ge back
            result = backDirection;
        }
        
        return result;
            
    }
}
