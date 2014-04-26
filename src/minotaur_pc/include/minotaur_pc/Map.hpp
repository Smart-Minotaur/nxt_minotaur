#ifndef MINOTAUR_PC_MAP_HPP_
#define MINOTAUR_PC_MAP_HPP_

namespace minotaur
{
    class Map
    {
    private:
        int width;
        int height;
        int **field;
        
    public:
        
        Map()
        :width(10), height(10)
        { field = new int[width][height]; }
        
        virtual ~Map() { delete[] field; }
        
        void setDimension(const int p_width, const int p_height)
        {
            width = p_width;
            height = p_height;
            delete[] field;
            field = new int[width][height];
        }
        
        int getWidth() const
        { return width; }
        
        int getHeight() const
        { return height; }
        
        int const**getField() const
        { return field; }
    }
}

#endif