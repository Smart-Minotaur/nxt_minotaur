#ifndef MINOTAUR_MAZE_NAVIGATOR_HPP
#define MINOTAUR_MAZE_NAVIGATOR_HPP

#include <nav_msgs/Odometry.h>
#include "minotaur_common/UltrasonicData.h"
#include "minotaur_maze/MazeMapUser.hpp"
#include "minotaur_maze/MinotaurControlNodeUser.hpp"

namespace minotaur
{
    /**
     * \brief The MazeNavigator class provides an interface for the
     *        navigation of a robot in a maze by a MapSolver object.
     * 
     * It is able to move the robot to another node or turn it in a
     * certain direction. These actions represent pure translation /
     * pure rotation.
     */
    class MazeNavigator: public MazeMapUser, public MinotaurControlNodeUser
    {
    public:
        MazeNavigator() { }
        virtual ~MazeNavigator() { }
    
        /**
         * This method is called when an Odometry message is
         * recieved. Processing the message is done here.
         * @param p_odometry the Odometry message
         */
        virtual void receivedOdometry(const nav_msgs::Odometry &p_odometry) = 0;
        /**
         * This method is called when an UltrasonicData message is
         * recieved. Processing the message is done here.
         * @param p_sensorData the UltrasonicData message
         */
        virtual void receivedUltrasonicData(const minotaur_common::UltrasonicData &p_sensorData) = 0;
        
        /**
         * Moves the robot straight forward to the next node in the maze.
         * @param p_currentDirection current direction of the robot
         */
        virtual void moveToNextNode(const Direction p_currentDirection) = 0;
        
        /**
         * Turns the robot to the given direction.
         * @param p_currentDirection current direction of the robot
         * @param p_newDirection target direction to which the robot is turned
         */
        virtual void turnRobotTo(const Direction p_currentDirection, const Direction p_newDirection) = 0;
        
        /**
         * Interrupts the MazeNavigator and shuts it down. If the robot
         * is moving the movement is stopped immediatly. 
         */
        virtual void shutdown() = 0;
    };
}

#endif
