#ifndef MINOTAUR_MAZE_NODE_HPP
#define MINOTAUR_MAZE_NODE_HPP

namespace minotaur
{
    enum Direction {EAST, SOUTH, WEST, NORTH};
    extern const char *DirectionStrings[];
    
    /* Calculates the difference between 2 Directions and the shortest 
     * turndirection.
     * Negative result means turn right, positive turn left. */
    int getDirectionDiff(const Direction p_dirA, const Direction p_dirB);
    
    Direction turnDirection(Direction p_direction, int p_turns);
    
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
