#ifndef MINOTAUR_MAZE_NODE_HPP
#define MINOTAUR_MAZE_NODE_HPP

namespace minotaur
{
    enum Direction {EAST, SOUTH, WEST, NORTH};
    // look in "MazeMap.cpp" for declaration
    extern const char *DirectionStrings[];
    
    class MazeNode
    {
    private:
        bool east, west, north, south;
    public:
        
        MazeNode(): east(false), west(false), north(false), south(false) { }
        ~MazeNode() { }
        
        bool blocked(Direction p_direction)
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
        
        void setBlocked(Direction p_direction, bool p_val)
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
