#ifndef MINOTAUR_ROBOT_SETTINGS_HPP
#define MINOTAUR_ROBOT_SETTINGS_HPP

#include <string>
#include "minotaur_common/PIDParameter.h"

namespace minotaur
{
    class RobotSettings
    {
    public:
        std::string modelName;
        minotaur_common::PIDParameter pidParameter;
        float wheelTrack;
        float wheelRadius;
        int samplingInterval;
        
        RobotSettings();
        ~RobotSettings();
        
        void loadFromParamServer(const std::string &p_modelName);
        void loadCurrentFromParamServer();
        
    };
}

#endif
