#include <algorithm>
#include "minotaur_maze_pc/UltrasonicMedianFilter.hpp"

#define DEF_SIZE 5

namespace minotaur
{
    
    UltrasonicMedianFilter::UltrasonicMedianFilter()
    { UltrasonicMedianFilter(DEF_SIZE); }
    
    UltrasonicMedianFilter::UltrasonicMedianFilter(int p_size)
    :current(0), count(0), values(p_size)
    {
        for(int i = 0; i < p_size; ++i)
            values.push_back(0);
    }
    UltrasonicMedianFilter::~UltrasonicMedianFilter()
    {  }
    
    void UltrasonicMedianFilter::add(const float p_value)
    {
        values[current] = p_value;
        if(count < values.size())
            ++count;
        
        current = (current + 1) % values.size();
        
        sortedValues.clear();
        for(int i = 0; i < count; ++i)
            sortedValues.push_back(values[i]);
            
        std::sort(sortedValues.begin(), sortedValues.end());
    }
    
    float UltrasonicMedianFilter::value()
    {
        return sortedValues[sortedValues.size() / 2];
    }
    
    void UltrasonicMedianFilter::clear()
    {
        current = 0;
        count = 0;
    }
}
