#ifndef MINOTAUR_PC_MAP_HPP_
#define MINOTAUR_PC_MAP_HPP_

#include <stdlib.h>

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
        {
            setDimension(width, height);
        }
        
        virtual ~Map() { delete[] field; }
        
        void setDimension(const int p_width, const int p_height)
        {
            width = p_width;
            height = p_height;
            if(field != NULL)
            {
                for(int i = 0; i < height; ++i)
                {
                    delete[] field[i];
                }
                delete[] field;
            }
            
            field = new int*[height];
            for(int i = 0; i < height; ++i)
            {
                field[i] = new int[width];
            }
        }
        
        int getWidth() const
        { return width; }
        
        int getHeight() const
        { return height; }
        
        int **getField() const
        { return field; }
    };
}

#endif