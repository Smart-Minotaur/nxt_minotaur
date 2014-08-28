#ifndef NXT_TELEGRAM_DECODER_HPP
#define NXT_TELEGRAM_DECODER_HPP

#include "nxt/Telegram.hpp"
#include "nxt/OutputState.hpp"
#include "nxt/InputValues.hpp"
#include "nxt/BatteryLevel.hpp"
#include "nxt/KeepAlive.hpp"
#include "nxt/LSStatus.hpp"
#include "nxt/LSRead.hpp"
#include "nxt/ProgramName.hpp"
#include "nxt/MailboxMessage.hpp"

namespace nxt
{
	/**
	 * \brief Provides functionality to create handy data structures from Telegram
	 * 		  objects.
	 */
	class TelegramDecoder
	{
	private:
		uint32_t getUINT32(const unsigned char *p_data, int p_index) const;
		uint16_t getUINT16(const unsigned char *p_data, int p_index) const;
	public:
		TelegramDecoder();
		~TelegramDecoder();
		
		OutputState decodeGetOutputStateMsg(const Telegram &p_telegram) const;
		InputValues decodeGetInputValuesMsg(const Telegram &p_telegram) const;
		BatteryLevel decodeGetBatteryLevelMsg(const Telegram &p_telegram) const;
		KeepAlive decodeKeepAliveMsg(const Telegram &p_telegram) const;
		LSStatus decodeGetLSStatusMsg(const Telegram &p_telegram) const;
		LSRead decodeLSReadMsg(const Telegram &p_telegram) const;
		ProgramName decodeGetCurrentProgramNameMsg(const Telegram &p_telegram) const;
		MailboxMessage decodeMessageReadMsg(const Telegram &p_telegram) const;
	};
}

#endif