#ifndef MINOTAUR_MEDIAN_FILTER_HPP
#define MINOTAUR_MEDIAN_FILTER_HPP

#include <vector>

namespace minotaur
{
    class MedianFilter
    {
    private:
        int current;
        int count;
        bool sorted;
        
        std::vector<float> values;
        std::vector<float> sortedValues;
        
    public:
        MedianFilter();
        MedianFilter(const int p_size);
        ~MedianFilter();
        
        void add(const float p_value);
        float value();
        void clear();
    };
}

#endif
