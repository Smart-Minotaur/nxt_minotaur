#ifndef MINOTAUR_PC_MOVEMENT_HPP_
#define MINOTAUR_PC_MOVEMENT_HPP_

namespace minotaur
{
    class Movement
    {
    public:
        float v;
        float w;
        float  sigma[2][2];
        
        Movement() { }
        Movement(const float p_v, const float p_w)
        :v(p_v), w(p_w) { }
        virtual ~Movement() { }
        
        void set(const float p_v, const float p_w)
        { v = p_v; w = p_w; }
        
        void setSigma(float **p_sigma)
        {
            for(int i = 0; i < 2; ++i)
                for(int j = 0;  j < 2; ++j)
                    sigma[i][j] = p_sigma[i][j];
        }
        
        
    };
}

#endif