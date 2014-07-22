#ifndef MINOTAUR_MAZE_PARAMETER_HPP
#define MINOTAUR_MAZE_PARAMETER_HPP

#include "robot_control_beagle/Utils.hpp"

namespace minotaur
{
    inline std::string PARAM_MAZE(const std::string& p_name)
    { return (MINOTAUR_ROS_NS "/mazes/") + p_name; }

    inline std::string PARAM_CURRENT_MAZE()
    { return (MINOTAUR_ROS_NS "/mazes/CurrentMaze"); }

    inline std::string PARAM_MAZE_MAP_WIDTH(const std::string& p_name)
    { return PARAM_MAZE(p_name) + "/map_width"; }

    inline std::string PARAM_MAZE_MAP_HEIGHT(const std::string& p_name)
    { return PARAM_MAZE(p_name) + "/map_height"; }

    inline std::string PARAM_MAZE_NODE_WIDTH(const std::string& p_name)
    { return PARAM_MAZE(p_name) + "/node_width"; }

    inline std::string PARAM_MAZE_NODE_HEIGHT(const std::string& p_name)
    { return PARAM_MAZE(p_name) + "/node_height"; }

    inline std::string PARAM_MAZE_INIT_DIRECTION(const std::string& p_name)
    { return PARAM_MAZE(p_name) + "/node_height"; }
}

#endif
