#ifndef MINOTAUR_MOTOR_VELOCITY_HPP
#define MINOTAUR_MOTOR_VELOCITY_HPP

namespace minotaur
{
    /* This Class is used as simple Container for the velocities of the left and right
     * motor of a robot. The velocity ist stored as meter per second.
     * Mathoperators provide a simpler usage of its values. */
    class MotorVelocity
    {
    public:
        float leftMPS;
        float rightMPS;
        
        MotorVelocity()
        :leftMPS(0.0f), rightMPS(0.0f) { }
        
        MotorVelocity(const MotorVelocity& p_velocity)
        :leftMPS(p_velocity.leftMPS), rightMPS(p_velocity.rightMPS) { }
        
        MotorVelocity(const float p_leftMPS, const float p_rightMPS)
        :leftMPS(p_leftMPS), rightMPS(p_rightMPS) { }
        
        virtual ~MotorVelocity() { }
        
        void set(const float p_leftMPS, const float p_rightMPS);
        void zero();
        
        MotorVelocity& operator+=(MotorVelocity const& motorVel);
        MotorVelocity& operator-=(MotorVelocity const& motorVel);
        MotorVelocity& operator*=(const float factor);
        MotorVelocity& operator/=(const float divisor);
    };
    
    const MotorVelocity operator+(MotorVelocity const& op_a, MotorVelocity const& op_b);
    const MotorVelocity operator-(MotorVelocity const& op_a, MotorVelocity const& op_b);
    
    const MotorVelocity operator*(MotorVelocity const& op_a, const float op_b);
    const MotorVelocity operator/(MotorVelocity const& op_a, const float op_b);
    const MotorVelocity operator*(const float op_a, MotorVelocity const& op_b);
    const MotorVelocity operator/(const float op_a, MotorVelocity const& op_b);
}

#endif
