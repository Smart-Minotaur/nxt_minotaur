#include "minotaur_maze/MazeNode.hpp"

namespace minotaur
{
    const char *DirectionStrings[] = {"EAST", "SOUTH", "WEST", "NORTH"};
    
    static int getDirectionDiffEAST(const Direction p_dir)
    {
        switch(p_dir) {
            case EAST:
                return 0;
            case WEST:
                return 2;
            case NORTH:
                return 1;
            case SOUTH:
                return -1;
            default:
                return 0;
        }
    }
    
    static int getDirectionDiffWEST(const Direction p_dir)
    {
        switch(p_dir) {
            case EAST:
                return -2;
            case WEST:
                return 0;
            case NORTH:
                return -1;
            case SOUTH:
                return 1;
            default:
                return 0;
        }
    }
    
    static int getDirectionDiffNORTH(const Direction p_dir)
    {
        switch(p_dir) {
            case EAST:
                return -1;
            case WEST:
                return 1;
            case NORTH:
                return 0;
            case SOUTH:
                return 2;
            default:
                return 0;
        }
    }
    
    static int getDirectionDiffSOUTH(const Direction p_dir)
    {
        switch(p_dir) {
            case EAST:
                return 1;
            case WEST:
                return -1;
            case NORTH:
                return -2;
            case SOUTH:
                return 0;
            default:
                return 0;
        }
    }
    
    int getDirectionDiff(const Direction p_dirA, const Direction p_dirB)
    {
        switch(p_dirA) {
            case EAST:
                return getDirectionDiffEAST(p_dirB);
            case WEST:
                return getDirectionDiffWEST(p_dirB);
            case NORTH:
                return getDirectionDiffNORTH(p_dirB);
            case SOUTH:
                return getDirectionDiffSOUTH(p_dirB);
            default:
                return 0;
        }
    }
    
    static Direction turnDirectionLeft(Direction p_direction)
    {
        switch(p_direction) {
            case EAST:
                return NORTH;
            case WEST:
                return SOUTH;
            case NORTH:
                return WEST;
            case SOUTH:
                return EAST;
            default:
                return p_direction;
        }
    }
    
    static Direction turnDirectionRight(Direction p_direction)
    {
        switch(p_direction) {
            case EAST:
                return SOUTH;
            case WEST:
                return NORTH;
            case NORTH:
                return EAST;
            case SOUTH:
                return WEST;
            default:
                return p_direction;
        }
    }
    
    Direction turnDirection(Direction p_direction, int p_turns)
    {
        Direction result = p_direction;
        int turns = p_turns;
        
        while(turns < 0) {
            result = turnDirectionRight(result);
            ++turns;
        }
            
        while(turns > 0) {
            result = turnDirectionLeft(result);
            --turns;
        }
        
        return result;
    }
    
    
}
