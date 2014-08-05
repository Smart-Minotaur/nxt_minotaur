#include <ros/ros.h>
#include <signal.h>
#include "minotaur_maze/MazeSolver.hpp"
#include "minotaur_maze/StayInMidNavigator.hpp"
#include "minotaur_maze/MinotaurMazeMapping.hpp"
#include "minotaur_maze/MinotaurExplorationAlgorithm.hpp"

#define NODE_NAME "MoveInMaze"
#define MAP_WIDTH 50
#define MAP_HEIGHT 50
#define NODE_WIDTH 0.41f
#define NODE_HEIGHT 0.41f
#define INITIAL_DIRECTION minotaur::EAST
#define MAP_SAVE_FILE "/home/ubuntu/map.txt"

static minotaur::StayInMidNavigator navigator;
static minotaur::MinotaurMazeMapping mapping;
static minotaur::MinotaurExplorationAlgorithm explorationAlgorithm;
static minotaur::MazeSolver *solver;
static struct sigaction sa;

static void sighandler(int sig)
{
    ROS_INFO("Stopping MazeSolver.");
    solver->stop();
    ROS_INFO("Joining MazeSolver.");
    solver->join();
    ROS_INFO("Stopping MinotaurControlNode.");
    solver->getControlNode().stop();
}

static void setSignalAction()
{
    memset(&sa, 0, sizeof(struct sigaction));
    sa.sa_handler = sighandler;
    sigaction(SIGINT, &sa, NULL);
    sigaction(SIGQUIT, &sa, NULL);
    sigaction(SIGTERM, &sa, NULL);
}

int main(int argc, char** argv)
{
    
    ros::init(argc, argv, NODE_NAME);
    ros::NodeHandle handle;
    setSignalAction();
    

    try {
        ROS_INFO("Loading current MazeSettings.");
        minotaur::MazeSolverConfig config(&navigator, &mapping, &explorationAlgorithm);
        config.handle = &handle;
        config.loadCurrentFromParamServer();
        
        ROS_INFO("-- Found maze \"%s\".", config.settings.mazeName.c_str());
        ROS_INFO("-- NodeWidth=%.3fm; NodeHeight=%.3fm.", config.settings.nodeWidth, config.settings.nodeHeight);
        ROS_INFO("-- MapWidth=%d; MapHeight=%d.", config.settings.mapWidth, config.settings.mapHeight);
        ROS_INFO("-- InitialRobotDirection=%s.", config.settings.initialRobotDirection.c_str());
        
        ROS_INFO("Creating MazeSolver.");
        minotaur::MazeSolver local_solver(config);
        
        solver = &local_solver;
        
        ROS_INFO("Start Solving.");
        solver->start();
        solver->getControlNode().spin();
        ROS_INFO("Saving Map.");
        ROS_INFO("-- Target=\"%s\"",MAP_SAVE_FILE);
        solver->getMap().saveASCIIFile(MAP_SAVE_FILE);
    } catch (std::exception &e) {
        ROS_ERROR("Exception occured: %s.", e.what());
        return -1;
    }
    
    ros::shutdown();
    
    return 0;
}
