#ifndef MINOTAUR_MINOTAUR_CONTROL_NODE_USER_HPP
#define MINOTAUR_MINOTAUR_CONTROL_NODE_USER_HPP

#include "minotaur_control_pc/MinotaurControlNode.hpp"
#include "minotaur_control_pc/SensorSettings.hpp"

namespace minotaur
{
    class MinotaurControlNodeUser
    {
    protected:
        MinotaurControlNode *controlNode;
        std::vector<SensorSetting> sensorSettings;
    public:
        MinotaurControlNodeUser():controlNode(NULL), sensorSettings() { }
        virtual ~MinotaurControlNodeUser() { }
        
        void setMinotaurControlNode(MinotaurControlNode *p_controlNode)
        { controlNode = p_controlNode; sensorSettings = getSensorSettings(); }
    };
}

#endif
