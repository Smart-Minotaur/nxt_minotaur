#ifndef MINOTAUR_IMAZE_MAPPING_HPP
#define MINOTAUR_IMAZE_MAPPING_HPP

#include "minotaur_maze/MazeMapUser.hpp"
#include "minotaur_maze/MinotaurControlNodeUser.hpp"

namespace minotaur
{
    /**
     * \brief The MazeMapping class provides an interface for the
     *        mapping algorithm of a MazeSolver object.
     * 
     * It processes UltrasonicData messages and as a result it sets
     * bolcked properties of the corresponding MazeNode object.
     */
    class MazeMapping : public MazeMapUser, public MinotaurControlNodeUser
    {
    public:
    
        /**
         * This method is called when an UltrasonicData message is
         * recieved. Processing the message is done here.
         * @param p_sensorData the UltrasonicData message
         */
        virtual void receivedUltrasonicData(const minotaur_common::UltrasonicData &p_sensorData) = 0;
        
        /**
         * Signals that the node at the given position should be mapped.
         * The method updates the MazeMap at that position.
         * @param p_x current x position of the robot
         * @param p_y current y position of the robot
         * @param p_direction current direction of the robot
         */
        virtual void mapNode(const unsigned int p_x, const unsigned int p_y, const Direction p_direction) = 0;
    };
}

#endif
