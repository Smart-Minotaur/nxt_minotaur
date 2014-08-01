#include <ros/ros.h>
#include <signal.h>
#include "minotaur_maze/MazeSolver.hpp"
#include "minotaur_maze/StayInMidNavigator.hpp"
#include "minotaur_maze/MinotaurMazeMapping.hpp"

#define NODE_NAME "MoveInMaze"
#define MAP_WIDTH 50
#define MAP_HEIGHT 50
#define NODE_WIDTH 0.41f
#define NODE_HEIGHT 0.41f
#define INITIAL_DIRECTION minotaur::EAST

static minotaur::StayInMidNavigator navigator;
static minotaur::MinotaurMazeMapping mapping;
static minotaur::MazeSolver *solver;
static struct sigaction sa;

static void sighandler(int sig)
{
    solver->stop();
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
    minotaur::MazeSolverConfig config;
    ros::init(argc, argv, NODE_NAME);
    ros::NodeHandle handle;
    setSignalAction();
    

    try {
        ROS_INFO("Getting Config.");
        config.navigator = &navigator;
        config.mapping = &mapping;
        config.handle = &handle;
        config.loadCurrentFromParamServer();
        
        ROS_INFO("Creating MazeSolver.");
        minotaur::MazeSolver local_solver(config);
        
        solver = &local_solver;
        
        ROS_INFO("Start Solving.");
        solver->start();
        solver->getMap().saveASCIIFile("/home/ubuntu/map.txt");
    } catch (std::exception &e) {
        ROS_ERROR("Exception occured: %s.", e.what());
        return -1;
    }
    
    ros::shutdown();
    
    return 0;
}
