#include <ros/ros.h>
#include <stdexcept>
#include <sstream>
#include "minotaur_common/MazeSettings.hpp"
#include "minotaur_common/MinotaurParam.hpp"

#define DEF_MAZE_NAME "Unknown"

namespace minotaur
{
    MazeSettings::MazeSettings()
    :mazeName(DEF_MAZE_NAME), mapWidth(1), mapHeight(1), nodeWidth(1.0f), nodeHeight(1.0f), initialRobotDirection("NORTH")
    { }
    MazeSettings::~MazeSettings()
    { }
    
    void MazeSettings::loadFromParamServer(const std::string &p_mazeName)
    {
        if(!ros::master::check())
            throw std::logic_error("ROS master is not running. Cannot load MazeSettings");
        
        if(!ros::param::has(PARAM_MAZE(p_mazeName))) {
            std::stringstream ss;
            ss << "Maze \"" << p_mazeName <<"\" does not exist. Cannot load MazeSettings";
            throw std::logic_error(ss.str());
        }
        
        mazeName = p_mazeName;
        
        if(!ros::param::get(PARAM_MAZE_MAP_WIDTH(p_mazeName), mapWidth)) {
            std::stringstream ss;
            ss << "No map width found (" << p_mazeName <<"). Cannot load MazeSettings";
            throw std::logic_error(ss.str());
        }
        
        if(!ros::param::get(PARAM_MAZE_MAP_HEIGHT(p_mazeName), mapHeight)) {
            std::stringstream ss;
            ss << "No map height found (" << p_mazeName <<"). Cannot load MazeSettings";
            throw std::logic_error(ss.str());
        }
        
        if(!ros::param::get(PARAM_MAZE_NODE_WIDTH(p_mazeName), nodeWidth)) {
            std::stringstream ss;
           ss << "No node width found (" << p_mazeName <<"). Cannot load MazeSettings";
            throw std::logic_error(ss.str());
        }
        
        if(!ros::param::get(PARAM_MAZE_NODE_HEIGHT(p_mazeName), nodeHeight)) {
            std::stringstream ss;
            ss << "No node height found (" << p_mazeName <<"). Cannot load MazeSettings";
            throw std::logic_error(ss.str());
        }
        
        if(!ros::param::get(PARAM_MAZE_INIT_DIRECTION(p_mazeName), initialRobotDirection)) {
            std::stringstream ss;
           ss << "No initial robot direction found (" << p_mazeName <<"). Cannot load MazeSettings";
            throw std::logic_error(ss.str());
        }
    }
    
    void MazeSettings::loadCurrentFromParamServer()
    {
        if(!ros::master::check())
            throw std::logic_error("ROS master is not running. Cannot load MazeSettings");
            
        std::string name;
        if(!ros::param::get(PARAM_CURRENT_MAZE(), name))
            throw std::logic_error("No current maze name available. Cannot load MazeSettings");
        loadFromParamServer(name);
    }
}
