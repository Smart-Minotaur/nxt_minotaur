#include <sstream>
#include <stdexcept>
#include "robot_control/SensorController.hpp"

namespace minotaur
{
    void SensorController::setBrick(nxtcon::Brick *p_brick)
    {
        brick = p_brick;
    }
        
    uint8_t SensorController::getDistance(const uint8_t p_id)
    {
        if(p_id < 0 || p_id >= sensors.size())
        {
            std::stringstream ss;
            ss << "No sensor available with id: " << p_id;
            throw std::logic_error(ss.str());
        }
        
        return sensors[p_id].getDistance();
    }
    
    uint8_t SensorController::addSensor(const uint8_t p_port)
    {
        sensors.push_back(nxtcon::UltrasonicSensor(brick, p_port));
        return sensors.size() - 1;
    }
    
    void SensorController::clearSensors()
    {
        sensors.clear();
    }
    
    int SensorController::count()
    {
        return sensors.size();
    }
}
