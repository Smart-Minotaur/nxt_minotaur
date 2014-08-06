#ifndef MINOTAUR_ROBOT_SETTINGS_HPP
#define MINOTAUR_ROBOT_SETTINGS_HPP

#include <string>
#include "minotaur_common/PIDParameter.h"

namespace minotaur
{
    /**
     * \brief The RobotSettings class contains all possible parameters
     *        for a robot of the minotaur project.
     * 
     * It provides functions to load the parameters from the ROS
     * Paramserver. Therefore loading the parameters manually is not
     * needed anymore.
     */
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
