#ifndef MINOTAUR_MEDIAN_FILTER_HPP
#define MINOTAUR_MEDIAN_FILTER_HPP

#include <vector>

namespace minotaur
{
    /**
     * \brief The MedianFilter class represents a simple MedianFilter
     *        with fixed size.
     * 
     * The size is set in MedianFilter(const int p_size) and cannot be
     * changed after construction. The default constructor generates a
     * MedianFilter with a default size.
     */
    class MedianFilter
    {
    private:
        int current;
        int elementCount;
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
        int size() const;
        int count() const;
        bool isEmpty() const;
    };
}

#endif
