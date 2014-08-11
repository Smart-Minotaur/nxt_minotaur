#ifndef MINOTAUR_MAP_BUG_ZERO_ALGORITHM_HEADER_HPP_
#define MINOTAUR_MAP_BUG_ZERO_ALGORITHM_HEADER_HPP_

#include "minotaur_map/MovementAlgorithmSensor.hpp"

namespace minotaur
{
    /**
     * \brief Implements the bug 0 algorithm.
     * 
     * This class  is an implementation of MovementAlgorithmSensor and
     * realizes a bug 0 behaviour.
     */
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
