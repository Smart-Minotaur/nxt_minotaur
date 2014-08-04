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
#include "minotaur_common/Thread.hpp"

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
    
    class MazeSolver : public IMinotaurListener, public Thread
    {
    private:
        volatile bool keepRunning, paused;
        int currentStep;
        
        pthread_mutex_t pauseMutex;
        pthread_cond_t pauseCond;
        
        MazeNavigator *navigator;
        MazeMapping *mapping;
        
        MinotaurControlNode minotaurNode;
        MazeMap map;
        MazeRobot robot;
        
        void mapCurrentNode();
        void moveToNextNode();
        void turnRobotTo(const Direction p_direction);
        
        void runExceptionSave();
        void checkPaused();
        void stepRobotPosition();
        void setPaused(bool p_value);
    protected:
        void onStart();
        void onStop();
    public:
        MazeSolver(const MazeSolverConfig &p_config);
        ~MazeSolver();
        
        void run();
        void pause();
        void resume();
        
        const MazeMap& getMap() const;
        
        void onReceiveOdometry(const nav_msgs::Odometry &p_odometry);
        void onReceiveUltrasonicData(const minotaur_common::UltrasonicData &p_sensorData);
    };
}

#endif
