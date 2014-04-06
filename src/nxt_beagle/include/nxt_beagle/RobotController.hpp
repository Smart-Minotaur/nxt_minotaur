#ifndef ROBOT_CONTROLLER_HPP_
#define ROBOT_CONTROLLER_HPP_

#include "nxt_beagle/IRobotController.hpp"
#include "nxt_beagle/PIDController.hpp"

namespace minotaur
{
    class RobotController: public IRobotController
    {
    private:
        RobotVelocity velocity;
        float wheelTrack;
        PIDController pidController;
        
        void calculateMotorVelocity();
        
    public:
        RobotController()
        : velocity(), wheelTrack(0.0f), pidController() { }
        
        virtual ~RobotController() { }
        
        const RobotVelocity& getRobotVelocity() const;
        IPIDController& getPIDController();
        float getWheelTrack() const;
        RobotVelocity getMeasuredVelocity() const;
        
        void setRobotVelocity(const RobotVelocity& p_velocity);
        void setWheelTrack(const float p_wheelTrack);
        
        void step(const float p_samplingIntervall);
    };
}

#endif