#ifndef MINOTAUR_MAZE_SETTINGS_HPP
#define MINOTAUR_MAZE_SETTINGS_HPP

#include <string>

namespace minotaur
{
    /**
     * \brief The MazeSettings class contains all possible parameters
     *        for a maze of the minotaur project.
     * 
     * It provides functions to load the parameters from the ROS
     * Paramserver. Therefore loading the parameters manually is not
     * needed anymore.
     */
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
