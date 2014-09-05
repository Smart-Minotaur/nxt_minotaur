#include "nxt/Telegram.hpp"

namespace nxt
{
	bool isLittleEndian()
    {
        int num = 1;
        return (*((char *) &num) == 1);
    }
	
	Telegram::Telegram()
	:data()
	{
		
	}
	
	Telegram::Telegram(const int p_size)
	:data()
	{
		data.reserve(p_size);
	}
	
	Telegram::~Telegram()
	{
		
	}
	
	void Telegram::add(const unsigned char p_byte)
	{
		data.push_back(p_byte);
	}
	
	void Telegram::addUINT8(const uint8_t p_byte)
	{
		data.push_back((unsigned char) p_byte);
	}
	
	void Telegram::addUINT16(const uint16_t p_word)
	{
		unsigned char *ptr = (unsigned char*) &p_word;
		data.push_back(ptr[0]);
		data.push_back(ptr[1]);
	}
	
	void Telegram::addUINT32(const uint32_t p_dword)
	{
		unsigned char *ptr = (unsigned char*) &p_dword;
		data.push_back(ptr[0]);
		data.push_back(ptr[1]);
		data.push_back(ptr[2]);
		data.push_back(ptr[3]);
	}
	
	void Telegram::addData(const unsigned char *p_data, const int p_count)
	{
		for(int i = 0; i < p_count; ++i)
			add(p_data[i]);
	}
	
	void Telegram::addTelegram(const Telegram& p_telegram)
	{
		data.insert(data.end(), p_telegram.data.begin(), p_telegram.data.end());
	}
	
	void Telegram::clear()
	{
		data.clear();
	}
	
	int Telegram::size() const
	{
		return data.size();
	}
	
	int Telegram::getData(unsigned char *p_data) const
	{
		for(int i = 0; i < data.size(); ++i)
			p_data[i] = data[i];
			
		return data.size();
	}
}
