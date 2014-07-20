#ifndef MINOTAUR_MAZE_NAVIGATOR_HPP
#define MINOTAUR_MAZE_NAVIGATOR_HPP

#include <nav_msgs/Odometry.h>
#include "robot_control_beagle/UltrasonicData.h"
#include "minotaur_maze_pc/MazeMapUser.hpp"
#include "minotaur_maze_pc/MinotaurControlNodeUser.hpp"

namespace minotaur
{
    class MazeNavigator: public MazeMapUser, public MinotaurControlNodeUser
    {
    public:
        MazeNavigator() { }
        virtual ~MazeNavigator() { }
    
        virtual void receivedOdometry(const nav_msgs::Odometry &p_odometry) = 0;
        virtual void receivedUltrasonicData(const robot_control_beagle::UltrasonicData &p_sensorData) = 0;
        
        virtual void moveToNextNode(Direction p_currentDirection) = 0;
        virtual void turnRobotTo(Direction p_currentDirection, Direction p_newDirection) = 0;
    };
}

#endif
