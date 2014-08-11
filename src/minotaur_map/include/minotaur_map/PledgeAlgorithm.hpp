#ifndef MINOTAUR_PC_PLEDGE_ALGORITHM_HEADER_HPP_
#define MINOTAUR_PC_PLEDGE_ALGORITHM_HEADER_HPP_

#include "minotaur_map/MovementAlgorithm.hpp"

namespace minotaur
{
    /**
     * \brief Implements the pledge algorithm.
     *
     * This class implements MovementAlgorithm and realizes a pledge
     * algorithm behaviour to find way ot of a maze.
     */
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
