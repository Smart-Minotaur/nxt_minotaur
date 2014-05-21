#ifndef MINOTAUR_PC_PLEDGE_ALGORITHM_HEADER_HPP_
#define MINOTAUR_PC_PLEDGE_ALGORITHM_HEADER_HPP_

#include "minotaur_pc/MovementAlgorithm.hpp"

namespace minotaur
{
    class PledgeAlgorithm : public MovementAlgorithm
    {
    private:
        int turn;
        bool obstacleFound;
        
    public:
        PledgeAlgorithm(Map* pMap);
        virtual ~PledgeAlgorithm(){}
        
        virtual RobotPosition getNextPosition(const RobotPosition pos);
    };
}


#endif