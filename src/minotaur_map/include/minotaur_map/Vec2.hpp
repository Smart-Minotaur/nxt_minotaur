#ifndef MINOTAUR_MAP_VEC2_HPP_
#define MINOTAUR_MAP_VEC2_HPP_

namespace minotaur
{
    /**
     * \brief Represents a 2D vector, which stores x and y coordinates.
     */
    class Vec2
    {
    public:
        float x;
        float y;
        
        Vec2()
        : x(0), y(0) { }
        Vec2(const float p_x, const float p_y)
        : x(p_x), y(p_y) { }
        virtual ~Vec2() { }
        
        void set(const float p_x, const float p_y)
        { x = p_x; y = p_y; }
    };
}

#endif
