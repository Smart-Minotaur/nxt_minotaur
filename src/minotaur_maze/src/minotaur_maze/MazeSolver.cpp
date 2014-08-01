#include <stdexcept>
#include <algorithm>
#include <sstream>
#include "minotaur_maze/MazeSolver.hpp"
#include "minotaur_common/RAIILock.hpp"

namespace minotaur
{
    void MazeSolverConfig::loadFromParamServer(const std::string &p_mazeName)
    {
        settings.loadFromParamServer(p_mazeName);
        upateInitialRobotDirection();
    }
    
    void MazeSolverConfig::loadCurrentFromParamServer()
    {
        settings.loadCurrentFromParamServer();
        upateInitialRobotDirection();
    }
    
    void MazeSolverConfig::upateInitialRobotDirection()
    {
        std::transform(settings.initialRobotDirection.begin(), settings.initialRobotDirection.end(), settings.initialRobotDirection.begin(), ::tolower);
        if(settings.initialRobotDirection == "east")
            initialRobotDirection = EAST;
        else if(settings.initialRobotDirection == "west")
            initialRobotDirection = WEST;
        else if(settings.initialRobotDirection == "north")
            initialRobotDirection = NORTH;
        else if(settings.initialRobotDirection == "south")
            initialRobotDirection = SOUTH;
        else {
            std::stringstream ss;
            ss << "Unknown initial robot direction \"" << settings.initialRobotDirection <<"\". Cannot load MazeSolverConfig";
            throw std::logic_error(ss.str());
        }
    }
    
    MazeSolver::MazeSolver(const MazeSolverConfig &p_config)
    : map(p_config.settings.mapWidth, p_config.settings.mapHeight), keepRunning(false), paused(false)
    {
        navigator = p_config.navigator;
        mapping = p_config.mapping;
        
        map.setNodeDimension(p_config.settings.nodeWidth, p_config.settings.nodeHeight);
        minotaurNode.setMinotaurListener(this);
        robot.x = p_config.settings.mapWidth / 2;
        robot.y = p_config.settings.mapHeight / 2;
        robot.direction = p_config.initialRobotDirection;
        
        if(pthread_mutex_init(&pauseMutex, NULL) != 0)
            throw std::logic_error("Failed to init mutex");
        if(pthread_cond_init(&pauseCond, NULL) != 0)
            throw std::logic_error("Failed to init condition variable");
        
        navigator->setMinotaurControlNode(&minotaurNode);
        navigator->setMazeMap(&map);
        
        mapping->setMinotaurControlNode(&minotaurNode);
        mapping->setMazeMap(&map);
        
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
        
        return NULL;
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
                
                ROS_INFO("==MAPPING==");
                mapping->mapNode(robot.x, robot.y, robot.direction);
                
                ROS_INFO("==MOVE TO NEXT NODE==");
                navigator->moveToNextNode(robot.direction);
                ROS_INFO("==REACHED TARGET==");
                stepRobotPosition();
                ROS_INFO("Robot (%d,%d) Direction %s.", robot.x, robot.y, DirectionStrings[robot.direction]);
                
                if(!keepRunning)
                    break;
                
                ROS_INFO("==MAPPING==");
                mapping->mapNode(robot.x, robot.y, robot.direction);
                
                ROS_INFO("==MOVE TO NEXT NODE==");
                navigator->moveToNextNode(robot.direction);
                ROS_INFO("==REACHED TARGET==");
                stepRobotPosition();
                ROS_INFO("Robot (%d,%d) Direction %s.", robot.x, robot.y, DirectionStrings[robot.direction]);
                
                if(!keepRunning)
                    break;
                
                ROS_INFO("==MAPPING==");
                mapping->mapNode(robot.x, robot.y, robot.direction);
                
                ROS_INFO("==TURNING ROBOT==");
                ROS_INFO("Target Direction %s.", DirectionStrings[EAST]);
                navigator->turnRobotTo(robot.direction, EAST);
                robot.direction = EAST;
                ROS_INFO("==REACHED DIRECTION==");
                
                if(!keepRunning)
                    break;
                
                ROS_INFO("==MOVE TO NEXT NODE==");
                navigator->moveToNextNode(robot.direction);
                ROS_INFO("==REACHED TARGET==");
                stepRobotPosition();
                ROS_INFO("Robot (%d,%d) Direction %s.", robot.x, robot.y, DirectionStrings[robot.direction]);
                
                if(!keepRunning)
                    break;
                
                ROS_INFO("==MAPPING==");
                mapping->mapNode(robot.x, robot.y, robot.direction);
                
                ROS_INFO("==MOVE TO NEXT NODE==");
                navigator->moveToNextNode(robot.direction);
                ROS_INFO("==REACHED TARGET==");
                stepRobotPosition();
                ROS_INFO("Robot (%d,%d) Direction %s.", robot.x, robot.y, DirectionStrings[robot.direction]);
                
                if(!keepRunning)
                    break;
                
                ROS_INFO("==MAPPING==");
                mapping->mapNode(robot.x, robot.y, robot.direction);
                
                ROS_INFO("==TURNING ROBOT==");
                ROS_INFO("Target Direction %s.", DirectionStrings[SOUTH]);
                navigator->turnRobotTo(robot.direction, SOUTH);
                robot.direction = SOUTH;
                ROS_INFO("==REACHED DIRECTION==");
                
                if(!keepRunning)
                    break;
                
                ROS_INFO("==MOVE TO NEXT NODE==");
                navigator->moveToNextNode(robot.direction);
                ROS_INFO("==REACHED TARGET==");
                stepRobotPosition();
                ROS_INFO("Robot (%d,%d) Direction %s.", robot.x, robot.y, DirectionStrings[robot.direction]);
                
                if(!keepRunning)
                    break;
                
                ROS_INFO("==MAPPING==");
                mapping->mapNode(robot.x, robot.y, robot.direction);
                
                keepRunning = false;
            }
        } catch (const std::exception &e) {
            ROS_ERROR("MazeSolver: exception during spin: %s.", e.what());
        } catch (const std::string &s) {
            ROS_ERROR("MazeSolver: catched string during spin: %s.", s.c_str());
        } catch (...) {
            ROS_ERROR("MazeSolver: catched unknown instance.");
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
            throw std::logic_error("Failed to create thread");
            
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
    
    void MazeSolver::onReceivedUltrasonicData(const minotaur_common::UltrasonicData &p_sensorData)
    {
        navigator->receivedUltrasonicData(p_sensorData);
        mapping->receivedUltrasonicData(p_sensorData);
    }
}
