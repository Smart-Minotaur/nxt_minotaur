#ifndef MINOTAUR_MAZE_SOLVER_HPP
#define MINOTAUR_MAZE_SOLVER_HPP

#include <pthread.h>
#include <string>
#include "minotaur_maze/MazeNavigator.hpp"
#include "minotaur_maze/MazeMapping.hpp"
#include "minotaur_maze/ExplorationAlgorithm.hpp"
#include "minotaur_maze/MazeMap.hpp"
#include "minotaur_common/MinotaurControlNode.hpp"
#include "minotaur_common/IMinotaurListener.hpp"
#include "minotaur_common/MazeSettings.hpp"
#include "minotaur_common/Thread.hpp"

namespace minotaur
{
    /**
     * \brief The MazeSolverConfig class is a container for settings for
     *        a MazeSolver object.
     * 
     * Its properties can be loaded form the ROS paramserver.
     */
    class MazeSolverConfig
    {
    private:
        void upateInitialRobotDirection();
    public:
        MazeNavigator *navigator;
        MazeMapping *mapping;
        ExplorationAlgorithm *explorationAlgorithm;
        ros::NodeHandle *handle;
        
        MazeSolverConfig(MazeNavigator *p_navigator,
                         MazeMapping *p_mapping,
                         ExplorationAlgorithm *p_explorationAlgorithm);
        ~MazeSolverConfig();
        
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
    
    /**
     * \brief The MazeSolver class runs all logic for solving a maze.
     * 
     * Its can be changed by implementing the MazeNavigator, MazeMapping
     * and ExplorationAlgorithm interfaces. These have to be set using
     * a MazeSolverConfig object.
     * The configuration of a MazeSolver object can only be set at
     * construction. Afterwards the configuration is not changeable
     * anymore.
     */
    class MazeSolver : public IMinotaurListener, public Thread
    {
    private:
        volatile bool keepRunning, paused;
        int currentStep;
        
        pthread_mutex_t pauseMutex;
        pthread_cond_t pauseCond;
        
        MazeNavigator *navigator;
        MazeMapping *mapping;
        ExplorationAlgorithm *explorationAlgorithm;
        
        MinotaurControlNode minotaurNode;
        MazeMap map;
        MazeRobot robot;
        
        void mapCurrentNode();
        void moveToNextNode();
        void turnRobotTo(const Direction p_direction);
        Direction getNextTargetDirection();
        
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
        
        /**
         * Pauses the object from solving the maze.
         */
        void pause();
        /**
         * Resumes the solving of the maze if it was pause.
         */
        void resume();
        
        const MazeMap& getMap() const;
        MinotaurControlNode& getControlNode();
        
        void onReceiveOdometry(const nav_msgs::Odometry &p_odometry);
        void onReceiveUltrasonicData(const minotaur_common::UltrasonicData &p_sensorData);
    };
}

#endif
