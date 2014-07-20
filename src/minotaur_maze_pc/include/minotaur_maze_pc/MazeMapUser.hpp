#ifndef MINOTAUR_MAZE_MAP_USER_HPP
#define MINOTAUR_MAZE_MAP_USER_HPP

#include "minotaur_maze_pc/MazeMap.hpp"

namespace minotaur
{
    class MazeMapUser
    {
    protected:    
        MazeMap *map;
    public:
        MazeMapUser():map(NULL) { }
        virtual ~MazeMapUser() { }
    
        void setMazeMap(MazeMap *p_map)
        { map = p_map; }
    };
}

#endif
