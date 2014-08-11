#ifndef NXT_CONTROL_TELEGRAM_HPP_
#define NXT_CONTROL_TELEGRAM_HPP_

#include <list>
#include <cstdint>
#include <cstdlib>

namespace nxtcon
{
    /**
     * \brief This class creates messages to communicate with the brick
     * 
     * Telegrams are Messages that are used to dcommunicate twith the brick.
     * For detailed information about available Telegrams check NxtTelegram.hpp
     */
    class Telegram
    {
    private:
        std::list<unsigned char> data;
        
        
    public:
        Telegram();
        virtual ~Telegram();
        
        void add(const unsigned char p_value);
        void add_uint8(const uint8_t p_value);
        void add_uint16(const uint16_t p_value);
        void add_uint32(const uint32_t p_value);
        void addArray(unsigned char *p_array, ssize_t p_count);
        void clear();
        
        size_t getLength() const;
        void getData(unsigned char* p_array) const;
    };
}

#endif