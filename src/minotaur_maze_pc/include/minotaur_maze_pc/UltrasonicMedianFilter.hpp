#ifndef MINOTAUR_ULTRASONIC_MEDIAN_FILTER_HPP
#define MINOTAUR_ULTRASONIC_MEDIAN_FILTER_HPP

#include <vector>

namespace minotaur
{
    class UltrasonicMedianFilter
    {
    private:
        int current;
        int count;
        
        std::vector<float> values;
        std::vector<float> sortedValues;
        
    public:
        UltrasonicMedianFilter();
        UltrasonicMedianFilter(int p_size);
        ~UltrasonicMedianFilter();
        
        void add(const float p_value);
        float value();
        void clear();
    };
}

#endif
