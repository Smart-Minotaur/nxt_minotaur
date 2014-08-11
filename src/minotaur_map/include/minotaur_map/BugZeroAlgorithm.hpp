#ifndef MINOTAUR_MAP_BUG_ZERO_ALGORITHM_HEADER_HPP_
#define MINOTAUR_MAP_BUG_ZERO_ALGORITHM_HEADER_HPP_

#include "minotaur_map/MovementAlgorithmSensor.hpp"

namespace minotaur
{
    class BugZeroAlgorithm : public MovementAlgorithmSensor
    {
    private:
    public:
        BugZeroAlgorithm()
        : MovementAlgorithmSensor() { };
        virtual ~BugZeroAlgorithm(){}
        
        virtual RobotPosition getNextPosition(const RobotPosition pos);
    };
}


#endif
