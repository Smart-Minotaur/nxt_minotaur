#ifndef MINOTAUR_MAZE_MAP_HPP
#define MINOTAUR_MAZE_MAP_HPP

#include <string>
#include "minotaur_maze/MazeNode.hpp"

namespace minotaur
{
    /**
     * \brief The MazeMap class manages MazeNode objects in a 2D array
     *        and contains various properties of the maze.
     */
    class MazeMap
    {
    private:
        unsigned int width;
        unsigned int height;
        float nodeWidth;
        float nodeHeight;
        MazeNode **mazeGrid;
    
    public:
        
        MazeMap(const unsigned int p_width, const unsigned int p_height);
        ~MazeMap();
        
        MazeNode& node(const unsigned int p_x, const unsigned int p_y);
        
        unsigned int getWidth();
        unsigned int getHeight();
        
        /**
         * Set the dimensions of a node in meter.
         * @param p_width node width in meter
         * @param p_height node height in meter
         */
        void setNodeDimension(const float p_width, const float p_height);
        /**
         * @return node width in meter
         */
        float getNodeWidth();
        /**
         * @return node height in meter
         */
        float getNodeHeight();
        
        /**
         * Saves the map and its blocked areas as ASCII art file.
         * @param p_file target file
         */
        void saveASCIIFile(const std::string &p_file) const;
    };
}

#endif
