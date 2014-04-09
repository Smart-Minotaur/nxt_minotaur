/*
 * Author: Fabian Meyer 
 */

#include "nxt_beagle/MotorVelocity.hpp"

namespace minotaur
{

    void MotorVelocity::set(const float p_leftMPS, const float p_rightMPS)
    {
        leftMPS = p_leftMPS;
        rightMPS = p_rightMPS;
    }
    
    void MotorVelocity::zero()
    {
        set(0,0);
    }
            
    const MotorVelocity operator+(MotorVelocity const& op_a, MotorVelocity const& op_b)
    {
        MotorVelocity result(op_a);
        result += op_b;
        return result;
    }

    const MotorVelocity operator-(MotorVelocity const& op_a, MotorVelocity const& op_b)
    {
        MotorVelocity result(op_a);
        result -= op_b;
        return result;
    }

    const MotorVelocity operator*(const float op_a, MotorVelocity const& op_b)
    {
        return op_b * op_a;
    }

    const MotorVelocity operator*(MotorVelocity const& op_a, const float op_b)
    {
        MotorVelocity result(op_a);
        result *= op_b;
        return result;
    }

    const MotorVelocity operator/(const float op_a, MotorVelocity const& op_b)
    {
        return op_b / op_a;
    }

    const MotorVelocity operator/(MotorVelocity const& op_a, const float op_b)
    {
        MotorVelocity result(op_a);
        result /= op_b;
        return result;
    }

    MotorVelocity& MotorVelocity::operator+=(MotorVelocity const& motorVel)
    {
        leftMPS += motorVel.leftMPS;
        rightMPS += motorVel.rightMPS;
        return *this;
    }

    MotorVelocity& MotorVelocity::operator-=(MotorVelocity const& motorVel)
    {
        leftMPS -= motorVel.leftMPS;
        rightMPS -= motorVel.rightMPS;
        return *this;
    }

    MotorVelocity& MotorVelocity::operator*=(const float factor)
    {
        leftMPS *= factor;
        rightMPS *= factor;
        return *this;
    }

    MotorVelocity& MotorVelocity::operator/=(const float divisor)
    {
        leftMPS /= divisor;
        rightMPS /= divisor;
        return *this;
    }
}