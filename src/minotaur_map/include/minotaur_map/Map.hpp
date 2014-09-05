#ifndef MINOTAUR_MAP_MAP_HPP_
#define MINOTAUR_MAP_MAP_HPP_

#include <stdlib.h>

namespace minotaur
{
    /**
     * \brief Represents a histogramm field.
     *
     * This class is a histogramm field and provides getter and setter
     * methods to access the individual points.
     */
    class Map
    {
    private:
        int width;
        int height;
        int **field;
        
    public:
        
        Map()
        :width(10), height(10), field(NULL)
        {
            setDimension(width, height);
        }
        
        virtual ~Map() { delete[] field; }
        
        void setDimension(const int p_width, const int p_height)
        {
            if(field != NULL)
            {
                for(int i = 0; i < width; ++i)
                {
                    delete[] field[i];
                }
                delete[] field;
            }
            
            width = p_width;
            height = p_height;
            field = new int*[width];
            for(int i = 0; i < width; ++i)
            {
                field[i] = new int[height];
                for(int j = 0; j < height; ++j)
                    field[i][j] = 0;
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
