#ifndef MINOTAUR_MAZE_MAP_HPP
#define MINOTAUR_MAZE_MAP_HPP

#include "minotaur_maze_pc/MazeNode.hpp"

namespace minotaur
{
    class MazeMap
    {
    private:
        unsigned int width;
        unsigned int height;
        unsigned int nodeWidth;
        unsigned int nodeHeight;
        MazeNode **mazeGrid;
    
    public:
        
        MazeMap(unsigned int p_width, unsigned int p_height);
        ~MazeMap();
        
        MazeNode& node(unsigned int p_x, unsigned int p_y);
        
        unsigned int getWidth();
        unsigned int getHeight();
        
        void setNodeDimension(unsigned int p_width, unsigned int p_height);
        unsigned int getNodeWidth();
        unsigned int getNodeHeight();
    };
}

#endif