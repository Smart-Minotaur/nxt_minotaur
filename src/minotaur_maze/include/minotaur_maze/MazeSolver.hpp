#ifndef MINOTAUR_MAZE_SOLVER_HPP
#define MINOTAUR_MAZE_SOLVER_HPP

#include <pthread.h>
#include <string>
#include "minotaur_maze/MazeNavigator.hpp"
#include "minotaur_maze/MazeMapping.hpp"
#include "minotaur_maze/MazeMap.hpp"
#include "minotaur_common/MinotaurControlNode.hpp"
#include "minotaur_common/IMinotaurListener.hpp"
#include "minotaur_common/MazeSettings.hpp"

namespace minotaur
{
    class MazeSolverConfig
    {
    private:
        void upateInitialRobotDirection();
    public:
        MazeNavigator *navigator;
        MazeMapping *mapping;
        ros::NodeHandle *handle;
        
        MazeSettings settings;
        Direction initialRobotDirection;
    
        void loadFromParamServer(const std::string &p_name);
        void loadCurrentFromParamServer();
    };
    
    class MazeRobot
    {
    public:
        unsigned int x, y;
        Direction direction;
        
        MazeRobot(): x(0), y(0), direction(EAST) { }
    };
    
    class MazeSolver : public IMinotaurListener
    {
    private:
        pthread_t mazeThread;
        volatile bool keepRunning, paused;
        
        pthread_mutex_t pauseMutex;
        pthread_cond_t pauseCond;
        
        MazeNavigator *navigator;
        MazeMapping *mapping;
        
        MinotaurControlNode minotaurNode;
        MazeMap map;
        MazeRobot robot;
        
        void runExceptionSave();
        void checkPaused();
        void stepRobotPosition();
        void setPaused(bool p_value);
    public:
        MazeSolver(const MazeSolverConfig &p_config);
        ~MazeSolver();
        
        void start();
        void stop();
        void pause();
        void resume();
        
        void run();
        
        const MazeMap& getMap() const;
        
        void onReceiveOdometry(const nav_msgs::Odometry &p_odometry);
        void onReceivedUltrasonicData(const minotaur_common::UltrasonicData &p_sensorData);
    };
}

#endif
