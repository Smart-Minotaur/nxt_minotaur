#include <stdexcept>
#include <algorithm>
#include <sstream>
#include "minotaur_maze_pc/MazeSolver.hpp"
#include "minotaur_maze_pc/Parameter.hpp"

namespace minotaur
{
    void MazeSolverConfig::loadFromParamServer(const std::string &p_name)
    {
        if(!ros::param::has(PARAM_MAZE(p_name))) {
            std::stringstream ss;
            ss << "Maze configuration \"" << p_name <<"\" does not exist";
            throw std::logic_error(ss.str());
        }
        
        int mapWidthTmp, mapHeightTmp;
        if(!ros::param::get(PARAM_MAZE_MAP_WIDTH(p_name), mapWidthTmp)) {
            std::stringstream ss;
            ss << "Map width for maze configuration \"" << p_name <<"\" does not exist";
            throw std::logic_error(ss.str());
        }
        mapWidth = mapWidthTmp;
        
        if(!ros::param::get(PARAM_MAZE_MAP_HEIGHT(p_name), mapHeightTmp)) {
            std::stringstream ss;
            ss << "Map height for maze configuration \"" << p_name <<"\" does not exist";
            throw std::logic_error(ss.str());
        }
        mapHeight = mapHeightTmp;
        
        double nodeWidthTmp;
        if(!ros::param::get(PARAM_MAZE_NODE_WIDTH(p_name), nodeWidth)) {
            std::stringstream ss;
            ss << "Node width for maze configuration \"" << p_name <<"\" does not exist";
            throw std::logic_error(ss.str());
        }
        
        if(!ros::param::get(PARAM_MAZE_NODE_HEIGHT(p_name), nodeHeight)) {
            std::stringstream ss;
            ss << "Node height for maze configuration \"" << p_name <<"\" does not exist";
            throw std::logic_error(ss.str());
        }
        
        std::string initDirection;
        if(!ros::param::get(PARAM_MAZE_INIT_DIRECTION(p_name), initDirection)) {
            std::stringstream ss;
            ss << "Initial robot direction for maze configuration \"" << p_name <<"\" does not exist";
            throw std::logic_error(ss.str());
        }
        
        std::transform(initDirection.begin(), initDirection.end(), initDirection.begin(), ::tolower);
        if(initDirection == "east")
            initialRobotDirection = EAST;
        else if(initDirection == "west")
            initialRobotDirection = WEST;
        else if(initDirection == "north")
            initialRobotDirection = NORTH;
        else if(initDirection == "south")
            initialRobotDirection = SOUTH;
        else {
            std::stringstream ss;
            ss << "Unknown initial robot direction \"" << initDirection <<"\"";
            throw std::logic_error(ss.str());
        }
    }
    
    void MazeSolverConfig::loadCurrentFromParamServer()
    {
        std::string mazeName;
        if(!ros::param::get(PARAM_CURRENT_MAZE(), mazeName))
            throw std::logic_error("No current maze name available");
        loadFromParamServer(mazeName);
    }
    
    MazeSolver::MazeSolver(const MazeSolverConfig &p_config)
    : map(p_config.mapWidth, p_config.mapHeight), keepRunning(false), paused(false)
    {
        navigator = p_config.navigator;
        
        map.setNodeDimension(p_config.nodeWidth, p_config.nodeHeight);
        minotaurNode.setMinotaurListener(this);
        robot.x = p_config.mapWidth / 2;
        robot.y = p_config.mapHeight / 2;
        robot.direction = p_config.initialRobotDirection;
        
        if(pthread_mutex_init(&pauseMutex, NULL) != 0)
            throw std::logic_error("Failed to init mutex");
        if(pthread_cond_init(&pauseCond, NULL) != 0)
            throw std::logic_error("Failed to init condition variable");
        
        navigator->setMinotaurControlNode(&minotaurNode);
        navigator->setMazeMap(&map);
        
        minotaurNode.connectToROS(*p_config.handle);
    }
    
    MazeSolver::~MazeSolver()
    {
        pthread_cond_destroy(&pauseCond);
        pthread_mutex_destroy(&pauseMutex);
    }
    
    static void* runThread(void* arg)
    {
        MazeSolver* solver = (MazeSolver*) arg;
        solver->run();
        
        pthread_exit(NULL);
    }
    
    void MazeSolver::run()
    {
        while(keepRunning)
            runExceptionSave();
    }
    
    void MazeSolver::runExceptionSave()
    {
        try {
            while(keepRunning)
            {
                pthread_mutex_lock(&pauseMutex);
                while(paused)                
                    pthread_cond_wait(&pauseCond, &pauseMutex);
                pthread_mutex_unlock(&pauseMutex);
                
                if(!keepRunning)
                    break;
                
                navigator->moveToNextNode(robot.direction);
            }
        } catch (std::exception const &e) {
            ROS_ERROR("MazeSolver: exception during spin: %s.", e.what());
        }
    }
    
    void MazeSolver::start()
    {
        keepRunning = true;
        paused = false;
        
        if(pthread_create(&mazeThread, NULL, runThread, this) != 0)
            throw std::logic_error("Failed to create Thread");
            
        minotaurNode.spin();
    }
    
    void MazeSolver::stop()
    {
        void *retVal;
        keepRunning = false;
        resume();
        
        pthread_join(mazeThread, &retVal);
        
        minotaurNode.stop();
    }
    
    void MazeSolver::pause()
    {
        pthread_mutex_lock(&pauseMutex);
        paused = true;
        pthread_mutex_unlock(&pauseMutex);
    }
    
    void MazeSolver::resume()
    {
        pthread_mutex_lock(&pauseMutex);
        paused = false;
        pthread_mutex_unlock(&pauseMutex);
        pthread_cond_signal(&pauseCond);
    }
    
    const MazeMap& MazeSolver::getMap() const
    {
        return map;
    }
    
    void MazeSolver::onReceiveOdometry(const nav_msgs::Odometry &p_odometry)
    {
        navigator->receivedOdometry(p_odometry);
    }
    
    void MazeSolver::onReceivedUltrasonicData(const robot_control_beagle::UltrasonicData &p_sensorData)
    {
        navigator->receivedUltrasonicData(p_sensorData);
    }
}
