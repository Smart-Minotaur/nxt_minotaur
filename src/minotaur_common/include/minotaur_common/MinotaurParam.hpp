/**
 * \file MinotaurParam.hpp
 * \brief Contains symbols and functions (name generation) to access the
 *        parameters on the ROS ParamServer.
 */

#ifndef MINOTAUR_PARAM_HPP
#define MINOTAUR_PARAM_HPP

#include <string>
#include <sstream>

/* Model names */
#define HERACLES_NAME "Heracles"
#define THESEUS_NAME "Theseus"
#define CURRENT_MODEL_NAME "CurrentModel"

/* Parameter for ROSParamServer */
#define MINOTAUR_PARAM_NS "/minotaur"

namespace minotaur
{
    /* Robot Parameter */
    inline std::string PARAM_MODEL(const std::string& p_name)
    { return (MINOTAUR_PARAM_NS "/models/") + p_name; }

    inline std::string PARAM_WHEEL_TRACK(const std::string& p_name)
    { return PARAM_MODEL(p_name) + "/wheel_track"; }

    inline std::string PARAM_WHEEL_RADIUS(const std::string& p_name)
    { return PARAM_MODEL(p_name) + "/wheel_radius"; }

    inline std::string PARAM_KP(const std::string& p_name)
    { return PARAM_MODEL(p_name) + "/kp"; }

    inline std::string PARAM_KI(const std::string& p_name)
    { return PARAM_MODEL(p_name) + "/ki"; }

    inline std::string PARAM_KD(const std::string& p_name)
    { return PARAM_MODEL(p_name) + "/kd"; }

    inline std::string PARAM_SAMPLING_INTERVAL(const std::string& p_name)
    { return PARAM_MODEL(p_name) + "/sampling_interval"; }

    inline std::string PARAM_CURRENT_MODEL()
    { return PARAM_MODEL(CURRENT_MODEL_NAME);}

	/* Sensor Parameter */
    inline std::string PARAM_SENSOR(const std::string& p_name, const int p_id)
    {   
        std::stringstream ss;
        ss << "/sensor_" << p_id;
        return PARAM_MODEL(p_name) + ss.str();
    }

    inline std::string PARAM_SENSOR_X(const std::string& p_name, const int p_id)
    { return PARAM_SENSOR(p_name, p_id) + "/x"; }

    inline std::string PARAM_SENSOR_Y(const std::string& p_name, const int p_id)
    { return PARAM_SENSOR(p_name, p_id) + "/y"; }

    inline std::string PARAM_SENSOR_DIRECTION(const std::string& p_name, const int p_id)
    { return PARAM_SENSOR(p_name, p_id) + "/direction"; }
	
	/* Mouse Parameter */
	inline std::string PARAM_MOUSE(const std::string& p_name, const int p_id)
    {   
        std::stringstream ss;
        ss << "/mouse_" << p_id;
        return PARAM_MODEL(p_name) + ss.str();
    }
	
	inline std::string PARAM_MOUSE_DEVICE(const std::string& p_name, const int p_id)
    { return PARAM_MOUSE(p_name, p_id) + "/device"; }
	
	inline std::string PARAM_MOUSE_X(const std::string& p_name, const int p_id)
    { return PARAM_MOUSE(p_name, p_id) + "/x"; }
	
	inline std::string PARAM_MOUSE_Y(const std::string& p_name, const int p_id)
    { return PARAM_MOUSE(p_name, p_id) + "/y"; }
	
	inline std::string PARAM_MOUSE_ERROR_ANGLE(const std::string& p_name, const int p_id)
    { return PARAM_MOUSE(p_name, p_id) + "/error_angle"; }
	
	inline std::string PARAM_MOUSE_X_RESOLUTION(const std::string& p_name, const int p_id)
    { return PARAM_MOUSE(p_name, p_id) + "/x_resolution"; }
	
	inline std::string PARAM_MOUSE_Y_RESOLUTION(const std::string& p_name, const int p_id)
    { return PARAM_MOUSE(p_name, p_id) + "/y_resolution"; }
    
    /* Maze Parameter */
    inline std::string PARAM_MAZE(const std::string& p_name)
    { return (MINOTAUR_PARAM_NS "/mazes/") + p_name; }

    inline std::string PARAM_CURRENT_MAZE()
    { return (MINOTAUR_PARAM_NS "/mazes/CurrentMaze"); }

    inline std::string PARAM_MAZE_MAP_WIDTH(const std::string& p_name)
    { return PARAM_MAZE(p_name) + "/map_width"; }

    inline std::string PARAM_MAZE_MAP_HEIGHT(const std::string& p_name)
    { return PARAM_MAZE(p_name) + "/map_height"; }

    inline std::string PARAM_MAZE_NODE_WIDTH(const std::string& p_name)
    { return PARAM_MAZE(p_name) + "/node_width"; }

    inline std::string PARAM_MAZE_NODE_HEIGHT(const std::string& p_name)
    { return PARAM_MAZE(p_name) + "/node_height"; }

    inline std::string PARAM_MAZE_INIT_DIRECTION(const std::string& p_name)
    { return PARAM_MAZE(p_name) + "/init_direction"; }
	
}

#endif
