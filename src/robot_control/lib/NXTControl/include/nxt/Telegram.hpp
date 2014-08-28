#ifndef NXT_TELEGRAM_HPP
#define NXT_TELEGRAM_HPP

#include <vector>
#include <cstdint>

namespace nxt
{
	bool isLittleEndian();
	
	/**
     * \brief This class represents a message to communicate with the brick
     * 
     * Telegrams are messages that are used to communicate with the brick. They
	 * do not consider endianess and thus they are only a simple byte buffer.
     * For detailed information about available Telegrams check the TelegramFactory
	 * class.
     */
	class Telegram
	{
	private:
		std::vector<unsigned char> data;
	public:
		Telegram();
		Telegram(const int p_size);
		~Telegram();
		
		void add(const unsigned char p_byte);
		void addUINT8(const uint8_t p_byte);
		void addUINT16(const uint16_t p_word);
		void addUINT32(const uint32_t p_dword);
		void addData(const unsigned char *p_data, const int p_count);
		void addTelegram(const Telegram& p_telegram);
		void clear();
		
		int size() const;
		int getData(unsigned char *p_data) const;
	};
}

#endif