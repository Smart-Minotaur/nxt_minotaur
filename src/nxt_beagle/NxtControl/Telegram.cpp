#include "nxt_control/Telegram.hpp"

namespace nxtcon
{
    Telegram::Telegram()
    :data()
    { }
    
    Telegram::~Telegram()
    { }
    
    void Telegram::add(const unsigned char p_value)
    {
        data.push_back(p_value);
    }
    
    void Telegram::add_uint8(const uint8_t p_value)
    {
        data.push_back((unsigned char) p_value);
    }
    
    void Telegram::add_uint16(const uint16_t p_value)
    {
        addArray((unsigned char*) &p_value, 2);
    }
    
    void Telegram::add_uint32(const uint32_t p_value)
    {
        addArray((unsigned char*) &p_value, 4);
    }
    
    void Telegram::clear()
    {
        data.clear();
    }
    
    size_t Telegram::getLength() const
    {
        return data.size();
    }
    
    void Telegram::getData(unsigned char *p_array) const
    {
        std::list<unsigned char>::const_iterator it;
        int i = 0;
        for(it = data.begin(); it != data.end(); ++it)
        {
            p_array[i] = *it;
            ++i;
        }
    }
    
    void Telegram::addArray(unsigned char *p_array, ssize_t p_count)
    {
        for(int i = 0; i < p_count; ++i)
            data.push_back(p_array[i]);
    }
}