#ifndef MINOTAUR_PC_BUG_ZERO_ALGORITHM_HEADER_HPP_
#define MINOTAUR_PC_BUG_ZERO_ALGORITHM_HEADER_HPP_



#include "minotaur_pc/MovementAlgorithm.hpp"

namespace minotaur
{
    class BugZeroAlgorithm : public MovementAlgorithm
    {
    private:
    public:
        BugZeroAlgorithm(Map* pMap)
        : MovementAlgorithm(pMap) { };
        virtual ~BugZeroAlgorithm(){}
        
        virtual RobotPosition getNextPosition(const RobotPosition pos);
    };
}


#endif