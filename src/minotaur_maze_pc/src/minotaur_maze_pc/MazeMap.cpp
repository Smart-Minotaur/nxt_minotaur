#include <stderr>
#include <sstrream>
#include "minotaur_maze_pc/MazeMap.hpp"

namespace minotaur
{
    MazeMap(unsigned int p_width, unsigned int p_height)
    :width(p_width), height(p_height)
    {
        mazeGrid = new MazeNode*[width];
        for(int i = 0; i < width; ++i)
            mazeGrid[i] = new MazeNode[height];
    }
   
    ~MazeMap()
    {
        for(int i = 0; i < width; ++i)
            delete[] mazeGrid[i];
        delete[] mazeGrid[i];
    }

    MazeNode& node(unsigned int p_x, unsigned int p_y)
    {
        if(p_x >= width) {
            std::stringstream ss;
            ss << "X is greater than width (" << p_x << ")";
            throw std::logic_error(ss.str());
        }
        if(p_y >= height) {
            std::stringstream ss;
            ss << "Y is greater than height (" << p_y << ")";
            throw std::logic_error(ss.str());
        }
        
        return mazeGrid[p_x][p_y];
    }

    unsigned int getWidth()
    {
        return width;
    }
    
    unsigned int getHeight()
    {
        return height;
    }

    void setNodeDimension(unsigned int p_width, unsigned int p_height)
    {
        nodeWidth = p_width;
        nodeHeight = p_height;
    }
    
    unsigned int getNodeWidth()
    {
        return nodeWidth;
    }
    
    unsigned int getNodeHeight()
    {
        return nodeHeight;
    }
}