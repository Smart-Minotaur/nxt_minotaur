#ifndef MINOTAUR_PC_DEADRECKONING_HPP_
#define MINOTAUR_PC_DEADRECKONING_HPP_

#include "minotaur_pc/RobotPosition.hpp"
#include "minotaur_pc/Movement.hpp"

namespace minotaur
{
    class DeadReckoning
    {
    private:
        RobotPosition pos;
        
        
    public:
        DeadReckoning() { };
        virtual ~DeadReckoning() { };
        
        void setPosition(const RobotPosition p_pos)
        { pos = p_pos; }
        
        RobotPosition getPosition() const
        { return pos; }
        
        void step(const Movement u, const int interval_ms);
        
    };
}

#endif