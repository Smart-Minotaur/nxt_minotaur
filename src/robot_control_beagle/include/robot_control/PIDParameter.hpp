/*
 * Author: Fabian Meyer 
 */

#ifndef ROBOT_CONTROL_PID_PARAMETER_HPP_
#define ROBOT_CONTROL_PID_PARAMETER_HPP_

namespace minotaur
{
    /* A simple container class to store the proportinal, integral and differential values
     * of a pid-controller. */
    class PIDParameter
    {
    public:
        float Kp;
        float Ki;
        float Kd;
        
        PIDParameter()
        : Kp(0), Ki(0), Kd(0) { }
        
        PIDParameter(const float p_Kp, const float p_Ki, const float p_Kd)
        : Kp(p_Kp), Ki(p_Ki), Kd(p_Kd) { }
        
        ~PIDParameter() { }
        
        void set(const float p_Kp, const float p_Ki, const float p_Kd)
        { Kp = p_Kp; Ki = p_Ki; Kd = p_Kd; }
    };
}

#endif