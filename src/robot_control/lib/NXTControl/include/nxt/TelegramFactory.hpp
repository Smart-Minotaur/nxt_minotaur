#ifndef NXT_TELEGRAM_FACTORY_HPP
#define NXT_TELEGRAM_FACTORY_HPP

#include <string>
#include "nxt/Telegram.hpp"

namespace nxt
{
	/**
	 * \brief Provides methods to create Telegram objects.
	 */
	class TelegramFactory
	{
	private:
		void addString(Telegram &p_telegram, const std::string &p_string) const;
		void addUINT16(Telegram &p_telegram, const uint16_t p_value) const;
		void addUINT32(Telegram &p_telegram, const uint32_t p_value) const;
	public:
		TelegramFactory();
		~TelegramFactory();
		
		Telegram startProgramMsg(const std::string &p_file) const;
		Telegram stopProgramMsg() const;
		Telegram playSoundFileMsg(const uint8_t p_loop,
								  const std::string &p_file) const;
		Telegram playToneMsg(const uint16_t p_frequency,
							 const uint16_t p_durationMS) const;
		Telegram setOutputStateMsg(const uint8_t p_port,
								   const uint8_t p_power,
								   const uint8_t p_motorMode,
								   const uint8_t p_regulationMode,
								   const uint8_t p_turnRatio,
								   const uint8_t p_runState,
								   const uint32_t p_tachoLimit) const;
		Telegram setInputModeMsg(const uint8_t p_port,
								 const uint8_t p_type,
								 const uint8_t p_mode) const;
		Telegram getOutputStateMsg(const uint8_t p_port) const;
		Telegram resetInputScaledValueMsg(const uint8_t p_port) const;
		Telegram getInputValuesMsg(const uint8_t p_port) const;
		Telegram messageWriteMsg(const uint8_t p_mailbox,
								 const std::string& p_message) const;
		Telegram resetMotorPositionMsg(const uint8_t p_port,
									   const uint8_t p_relative) const;
		Telegram getBatteryLevelMsg() const;
		Telegram stopSoundPlaybackMsg() const;
		Telegram keepAliveMsg() const;
		Telegram lsGetStatusMsg(const uint8_t p_port) const;
		Telegram lsWriteMsg(const uint8_t p_port,
							const uint8_t p_txDataLength,
							const uint8_t p_rxDataLength,
							const unsigned char *p_txData) const;
		Telegram lsReadMsg(const uint8_t p_port) const;
		Telegram getCurrentProgramNameMsg() const;
		Telegram messageReadMsg(const uint8_t p_remoteMailbox,
								const uint8_t p_localMailbox,
								const uint8_t p_removeMessage) const;
	};
}

#endif