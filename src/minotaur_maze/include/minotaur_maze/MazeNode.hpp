#ifndef MINOTAUR_MAZE_NODE_HPP
#define MINOTAUR_MAZE_NODE_HPP

namespace minotaur
{
    enum Direction {EAST, SOUTH, WEST, NORTH};
    extern const char *DirectionStrings[];
    
    /**
     * Calculates the difference between 2 Directions and the shortest 
     * turndirection.
     * @param p_dirA direction A
     * @param p_dirB direction B
     * @return turns needed to get from A to B
     * @retval negative means turn clockwise
     * @retval positive means turn anti-clockwise
     */
    int getDirectionDiff(const Direction p_dirA, const Direction p_dirB);
    
    /**
     * Turns the given direction p_turns times. Negative values mean
     * turn clockwise, positive values mean turn anti-clockwise.
     * @param p_direction direction to turn
     * @param p_turns number of turns to do
     * @return the resulting direction
     */
    Direction turnDirection(const Direction p_direction, const int p_turns);
    
    /**
     * \brief The MazeNode class represents a single node in a maze.
     * 
     * It can be blocked in EAST, SOUTH, WEST, NORTH direction.
     */
    class MazeNode
    {
    private:
        bool east, west, north, south;
    public:
        
        MazeNode(): east(false), west(false), north(false), south(false) { }
        ~MazeNode() { }
        
        bool isBlocked(const Direction p_direction)
        {
            switch(p_direction) {
            case EAST:
                return east;
            case WEST:
                return west;
            case NORTH:
                return north;
            case SOUTH:
                return south;
            default:
                return false;
            }
        }
        
        void setBlocked(const Direction p_direction, const bool p_val)
        {
            switch(p_direction) {
            case EAST:
                east = p_val;
                break;
            case WEST:
                west = p_val;
                break;
            case NORTH:
                north = p_val;
                break;
            case SOUTH:
                south = p_val;
                break;
            default:
                break;
            }
        }
    };
}

#endif
