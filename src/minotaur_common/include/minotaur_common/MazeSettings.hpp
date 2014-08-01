#ifndef MINOTAUR_MAZE_SETTINGS_HPP
#define MINOTAUR_MAZE_SETTINGS_HPP

#include <string>

namespace minotaur
{
    class MazeSettings
    {
    public:
        std::string mazeName;
        int mapWidth, mapHeight;
        float nodeWidth, nodeHeight;
        std::string initialRobotDirection;
        
        MazeSettings();
        ~MazeSettings();
        
        void loadFromParamServer(const std::string &p_mazeName);
        void loadCurrentFromParamServer();
    };
}

#endif
