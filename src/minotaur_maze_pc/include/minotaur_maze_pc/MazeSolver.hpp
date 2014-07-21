#ifndef MINOTAUR_MAZE_SOLVER_HPP
#define MINOTAUR_MAZE_SOLVER_HPP

#include <pthread.h>
#include "minotaur_maze_pc/MazeNavigator.hpp"
#include "minotaur_maze_pc/MazeMap.hpp"
#include "minotaur_control_pc/MinotaurControlNode.hpp"
#include "minotaur_control_pc/IMinotaurListener.hpp"

namespace minotaur
{
    class MazeSolverConfig
    {
    public:
        MazeNavigator *navigator;
        ros::NodeHandle *handle;
        
        unsigned int mapWidth, mapHeight;
        float nodeWidth, nodeHeight;
        
        Direction initialRobotDirection;
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
        
        MinotaurControlNode minotaurNode;
        MazeMap map;
        MazeRobot robot;
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
        void onReceiveUltrasonicData(const robot_control_beagle::UltrasonicData &p_sensorData);
    };
}

#endif
