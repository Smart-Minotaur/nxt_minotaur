#ifndef ROBOT_VELOCITY_HPP_
#define ROBOT_VELOCITY_HPP_

namespace minotaur
{
    class RobotVelocity
    {
    public:
        float linearVelocity;
        float angularVelocity;
        
        RobotVelocity()
        :linearVelocity(0.0f), angularVelocity(0.0f) { }
        
        RobotVelocity(const float p_linearVel, const float p_angularVel)
        :linearVelocity(p_linearVel), angularVelocity(p_angularVel) { }
        
        virtual ~RobotVelocity() { }
        
        void set(const float p_linearVel, const float p_angularVel)
        { linearVelocity = p_linearVel; angularVelocity = p_angularVel;}
    };
}

#endif