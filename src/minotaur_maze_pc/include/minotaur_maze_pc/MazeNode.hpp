#ifndef MINOTAUR_MAZE_NODE_HPP
#define MINOTAUR_MAZE_NODE_HPP

namespace minotaur
{
    class MazeNode
    {
    public:
        bool east, west, north, south;
        
        MazeNode(): east(false), west(false), north(false), south(false) { }
        ~MazeNode() { }
    };
}

#endif