#include <stdexcept>
#include <algorithm>
#include <sstream>
#include "minotaur_maze_pc/MazeSolver.hpp"
#include "minotaur_maze_pc/Parameter.hpp"
#include "robot_control_beagle/RAIILock.hpp"

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
                checkPaused();
                
                if(!keepRunning)
                    break;
                
                ROS_INFO("==MOVE TO NEXT NODE==");
                navigator->moveToNextNode(robot.direction);
                ROS_INFO("==REACHED TARGET==");
                stepRobotPosition();
                ROS_INFO("Robot (%d,%d) Direction %s.", robot.x, robot.y, DirectionStrings[robot.direction]);
                
                usleep(1000000);
                
                ROS_INFO("==MOVE TO NEXT NODE==");
                navigator->moveToNextNode(robot.direction);
                ROS_INFO("==REACHED TARGET==");
                stepRobotPosition();
                ROS_INFO("Robot (%d,%d) Direction %s.", robot.x, robot.y, DirectionStrings[robot.direction]);
                
                usleep(1000000);
                
                ROS_INFO("==TURNING ROBOT==");
                ROS_INFO("Target Direction %s.", DirectionStrings[EAST]);
                navigator->turnRobotTo(robot.direction, EAST);
                robot.direction = EAST;
                ROS_INFO("==REACHED DIRECTION==");
                
                usleep(1000000);
                
                ROS_INFO("==MOVE TO NEXT NODE==");
                navigator->moveToNextNode(robot.direction);
                ROS_INFO("==REACHED TARGET==");
                stepRobotPosition();
                ROS_INFO("Robot (%d,%d) Direction %s.", robot.x, robot.y, DirectionStrings[robot.direction]);
                
                usleep(1000000);
                
                ROS_INFO("==MOVE TO NEXT NODE==");
                navigator->moveToNextNode(robot.direction);
                ROS_INFO("==REACHED TARGET==");
                stepRobotPosition();
                ROS_INFO("Robot (%d,%d) Direction %s.", robot.x, robot.y, DirectionStrings[robot.direction]);
                
                usleep(1000000);
                
                ROS_INFO("==TURNING ROBOT==");
                ROS_INFO("Target Direction %s.", DirectionStrings[SOUTH]);
                navigator->turnRobotTo(robot.direction, SOUTH);
                robot.direction = EAST;
                ROS_INFO("==REACHED DIRECTION==");
                
                usleep(1000000);
                
                ROS_INFO("==MOVE TO NEXT NODE==");
                navigator->moveToNextNode(robot.direction);
                ROS_INFO("==REACHED TARGET==");
                stepRobotPosition();
                ROS_INFO("Robot (%d,%d) Direction %s.", robot.x, robot.y, DirectionStrings[robot.direction]);
                
                keepRunning = false;
            }
        } catch (std::exception const &e) {
            ROS_ERROR("MazeSolver: exception during spin: %s.", e.what());
        }
    }
    
    void MazeSolver::checkPaused()
    {
        RAIILock lock(&pauseMutex);
        while(paused)                
            pthread_cond_wait(&pauseCond, &pauseMutex);
    }
    
    void MazeSolver::stepRobotPosition()
    {
        switch(robot.direction) {
            case EAST:
                ++robot.x;
                break;
            case WEST:
                --robot.x;
                break;
            case NORTH:
                ++robot.y;
                break;
            case SOUTH:
                --robot.y;
                break;
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
        ROS_INFO("MazeSolver: stop() called.");
        void *retVal;
        keepRunning = false;
        
        navigator->shutdown();
        
        resume();
        
        ROS_INFO("MazeSolver: joining maze thread.");
        pthread_join(mazeThread, &retVal);
        
        ROS_INFO("MazeSolver: stopping minotaur node.");
        minotaurNode.stop();
    }
    
    void MazeSolver::pause()
    {
        setPaused(true);
    }
    
    void MazeSolver::resume()
    {
        setPaused(false);
        pthread_cond_signal(&pauseCond);
    }
    
    void MazeSolver::setPaused(bool p_value)
    {
        RAIILock lock(&pauseMutex);
        paused = p_value;
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
