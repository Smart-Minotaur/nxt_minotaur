#include <algorithm>
#include "minotaur_maze_pc/MedianFilter.hpp"

#define DEF_SIZE 5

namespace minotaur
{
    
    MedianFilter::MedianFilter()
    { MedianFilter(DEF_SIZE); }
    
    MedianFilter::MedianFilter(int p_size)
    :current(0), elementCount(0), values(p_size), sorted(false)
    {
        for(int i = 0; i < p_size; ++i)
            values.push_back(0);
    }
    MedianFilter::~MedianFilter()
    {  }
    
    void MedianFilter::add(const float p_value)
    {
        values[current] = p_value;
        if(elementCount < values.size())
            ++elementCount;
        
        current = (current + 1) % values.size();
        
        sortedValues.clear();
        for(int i = 0; i < elementCount; ++i)
            sortedValues.push_back(values[i]);
            
        sorted = false;
    }
    
    float MedianFilter::value()
    {
        if(!sorted) {
            std::sort(sortedValues.begin(), sortedValues.end());
            sorted = true;
        }
        return sortedValues[sortedValues.size() / 2];
    }
    
    void MedianFilter::clear()
    {
        current = 0;
        elementCount = 0;
        sorted = false;
    }
    
    int MedianFilter::size() const
    {
        return values.size();
    }
    
    int MedianFilter::count() const
    {
        return elementCount;
    }
    
    bool MedianFilter::isEmpty() const
    {
        return elementCount == 0;
    }
}
