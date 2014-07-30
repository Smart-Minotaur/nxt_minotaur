#include <stdexcept>
#include <sstream>
#include <iostream>
#include <fstream>
#include "minotaur_maze_pc/MazeMap.hpp"

namespace minotaur
{
    MazeMap::MazeMap(unsigned int p_width, unsigned int p_height)
    :width(p_width), height(p_height)
    {
        mazeGrid = new MazeNode*[width];
        for(int i = 0; i < width; ++i)
            mazeGrid[i] = new MazeNode[height];
    }
   
    MazeMap::~MazeMap()
    {
        for(int i = 0; i < width; ++i)
            delete[] mazeGrid[i];
        delete[] mazeGrid;
    }

    MazeNode& MazeMap::node(unsigned int p_x, unsigned int p_y)
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

    unsigned int MazeMap::getWidth()
    {
        return width;
    }
    
    unsigned int MazeMap::getHeight()
    {
        return height;
    }

    void MazeMap::setNodeDimension(float p_width, float p_height)
    {
        nodeWidth = p_width;
        nodeHeight = p_height;
    }
    
    float MazeMap::getNodeWidth()
    {
        return nodeWidth;
    }
    
    float MazeMap::getNodeHeight()
    {
        return nodeHeight;
    }
    
    void MazeMap::saveASCIIFile(const std::string &p_file) const
    {
        std::ofstream file;
        file.open(p_file.c_str());
        
        for(int x = 0; x < width; ++x) {
            file << ' ';
            if(mazeGrid[x][height - 1].isBlocked(NORTH))
                file << '_';
            else
                file << ' ';
        }
        file << '\n';
        
        for(int y = height - 2; y >= 0; --y) {
            for(int x = 0; x < width; ++x) {
                if(mazeGrid[x][y].isBlocked(WEST) || (x - 1 >= 0 && mazeGrid[x - 1][y].isBlocked(EAST)))
                    file << '|';
                else
                    file << ' ';
                    
                if(mazeGrid[x][y].isBlocked(SOUTH) || (y - 1 >= 0 && mazeGrid[x][y - 1].isBlocked(NORTH)))
                    file << '_';
                else
                    file << ' ';
                
                if(x == (width - 1)) {
                    if(mazeGrid[x][y].isBlocked(EAST))
                        file << '|';
                    else
                        file << ' ';
                }
            }
            file << '\n';
        }
        
        file.close();
    }
}
