#include <stdexcept>
#include <algorithm>
#include <sstream>
#include "minotaur_maze/MazeSolver.hpp"
#include "minotaur_common/RAIILock.hpp"

namespace minotaur
{
    MazeSolverConfig::MazeSolverConfig(MazeNavigator *p_navigator,
                                       MazeMapping *p_mapping,
                                       ExplorationAlgorithm *p_explorationAlgorithm)
    : navigator(p_navigator), mapping(p_mapping), explorationAlgorithm(p_explorationAlgorithm), handle(NULL)
    {
    }
    
    MazeSolverConfig::~MazeSolverConfig()
    {
    }
    
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
        explorationAlgorithm = p_config.explorationAlgorithm;
        
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
        
        explorationAlgorithm->setMazeMap(&map);
        
        minotaurNode.connectToROS(*p_config.handle);
    }
    
    MazeSolver::~MazeSolver()
    {
        pthread_cond_destroy(&pauseCond);
        pthread_mutex_destroy(&pauseMutex);
    }
    
    void MazeSolver::onStart()
    {
        keepRunning = true;
    }
    
    void MazeSolver::onStop()
    {
        keepRunning = false;
        navigator->shutdown();
        resume();
        minotaurNode.stop();
    }
    
    void MazeSolver::run()
    {
        ROS_INFO("MazeSolver: started.");
        currentStep = 0;
        while(keepRunning)
            runExceptionSave();
        ROS_INFO("MazeSolver: terminated.");
    }
    
    void MazeSolver::runExceptionSave()
    {
        try {
            while(keepRunning)
            {
                checkPaused();
                if(!keepRunning)
                    break;
                
                mapCurrentNode();
                Direction targetDirection = getNextTargetDirection();
                if(targetDirection != robot.direction)
                    turnRobotTo(targetDirection);
                    
                moveToNextNode();
                
                ROS_INFO("Finished step %d.", currentStep);
                ROS_INFO("-- keepRunning=%d; paused=%d.", keepRunning, paused);
                ++currentStep;
            }
        } catch (const std::exception &e) {
            ROS_ERROR("MazeSolver: catched exception during solving: %s.", e.what());
        } catch (const std::string &s) {
            ROS_ERROR("MazeSolver: catched string during solving: %s.", s.c_str());
        } catch (...) {
            ROS_ERROR("MazeSolver: catched unknown instance.");
        }
    }
    
    void MazeSolver::checkPaused()
    {
        RAIILock lock(&pauseMutex);
        while(keepRunning && paused)                
            pthread_cond_wait(&pauseCond, &pauseMutex);
    }
    
    void MazeSolver::mapCurrentNode()
    {
        ROS_INFO("Mapping current node.");
        ROS_INFO("-- Position: (%d,%d,%s).", robot.x, robot.y, DirectionStrings[robot.direction]);
        mapping->mapNode(robot.x, robot.y, robot.direction);
    }
    
    void MazeSolver::moveToNextNode()
    {
        ROS_INFO("Moving to next node.");
        ROS_INFO("-- Current pose: (%d,%d,%s).", robot.x, robot.y, DirectionStrings[robot.direction]);
        stepRobotPosition();
        ROS_INFO("-- Target pose: (%d,%d,%s).", robot.x, robot.y, DirectionStrings[robot.direction]);
        navigator->moveToNextNode(robot.direction);
        ROS_INFO("-- Reached target.");
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
    
    void MazeSolver::turnRobotTo(const Direction p_direction)
    {
        ROS_INFO("Turning Robot.");
        ROS_INFO("-- Current pose: (%d,%d,%s).", robot.x, robot.y, DirectionStrings[robot.direction]);
        ROS_INFO("-- Target pose: (%d,%d,%s).", robot.x, robot.y, DirectionStrings[p_direction]);
        navigator->turnRobotTo(robot.direction, p_direction);
        robot.direction = p_direction;
        ROS_INFO("-- Reached Target.");
    }
    
    Direction MazeSolver::getNextTargetDirection()
    {
        Direction result;
        ROS_INFO("Calculating next Direction.");
        ROS_INFO("-- Current direction: %s.", DirectionStrings[robot.direction]);
        result = explorationAlgorithm->calculateMovementDirection(robot.x, robot.y, robot.direction);
        ROS_INFO("-- Target direction: %s.", DirectionStrings[result]);
        return result;
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
    
    MinotaurControlNode& MazeSolver::getControlNode()
    {
        return minotaurNode;
    }
    
    void MazeSolver::onReceiveOdometry(const nav_msgs::Odometry &p_odometry)
    {
        navigator->receivedOdometry(p_odometry);
    }
    
    void MazeSolver::onReceiveUltrasonicData(const minotaur_common::UltrasonicData &p_sensorData)
    {
        navigator->receivedUltrasonicData(p_sensorData);
        mapping->receivedUltrasonicData(p_sensorData);
    }
}
