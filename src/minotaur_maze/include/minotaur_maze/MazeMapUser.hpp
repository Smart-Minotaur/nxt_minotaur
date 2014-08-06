#ifndef MINOTAUR_MAZE_MAP_USER_HPP
#define MINOTAUR_MAZE_MAP_USER_HPP

#include "minotaur_maze/MazeMap.hpp"

namespace minotaur
{
    /**
     * \brief The MazeMapUser class provides a MazeMap member and setter
     *        methods.
     * 
     * Its purpose is to prevent code duplication.
     */
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
