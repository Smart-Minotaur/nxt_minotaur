#include "nxt/TelegramFactory.hpp"
#include "nxt/OPCodes.hpp"
#include "nxt/Exceptions.hpp"

namespace nxt
{
	TelegramFactory::TelegramFactory()
	{
		
	}
	
	TelegramFactory::~TelegramFactory()
	{
		
	}
	
	Telegram TelegramFactory::startProgramMsg(const std::string &p_file) const
	{
		if(p_file.length() + 1 > NXT_MAX_FILE_NAME_LENGTH)
			throw NXTException("Cannot start program on NXT. Filename too long");

		Telegram telegram(2 + p_file.length() + 1);
		
		telegram.add(DIRECT_CMD_NO_REPLY);
		telegram.add(CMD_START_PROGRAM);
		// add file name
		addString(telegram, p_file);
		
		return telegram;
	}
	
	Telegram TelegramFactory::stopProgramMsg() const
	{
		Telegram telegram(2);
		
		telegram.add(DIRECT_CMD_NO_REPLY);
		telegram.add(CMD_STOP_PROGRAM);
		
		return telegram;
	}
	
	Telegram TelegramFactory::playSoundFileMsg(const uint8_t p_loop,
											   const std::string &p_file) const
	{
		if(p_file.length() + 1 > NXT_MAX_FILE_NAME_LENGTH)
			throw NXTException("Cannot play sound file on NXT. Filename too long");
			
		Telegram telegram(3 + p_file.length() + 1);
		
		telegram.add(DIRECT_CMD_NO_REPLY);
		telegram.add(CMD_PLAY_SOUND_FILE);
		telegram.addUINT8(p_loop);
		addString(telegram, p_file);
		
		return telegram;
	}
	
	Telegram TelegramFactory::playToneMsg(const uint16_t p_frequency,
										  const uint16_t p_durationMS) const
	{
		Telegram telegram(6);
		
        telegram.add(DIRECT_CMD_NO_REPLY);
        telegram.add(CMD_PLAY_TONE);
		addUINT16(telegram, p_frequency);
		addUINT16(telegram, p_durationMS);
		
		return telegram;
	}
	
	Telegram TelegramFactory::setOutputStateMsg(const uint8_t p_port,
							   const uint8_t p_power,
							   const uint8_t p_motorMode,
							   const uint8_t p_regulationMode,
							   const uint8_t p_turnRatio,
							   const uint8_t p_runState,
							   const uint32_t p_tachoLimit) const
	{
		Telegram telegram(12);
		
		telegram.add(DIRECT_CMD_NO_REPLY);
		telegram.add(CMD_SET_OUTPUT_STATE);
		//set motor port
		telegram.addUINT8(p_port);
		//set motor power
		telegram.addUINT8(p_power);
		//set motor mode
		telegram.addUINT8(p_motorMode);
		//set regulation mode
		telegram.addUINT8(p_regulationMode);
		//set turn ratio
		telegram.addUINT8(p_turnRatio);
		//set run state
		telegram.addUINT8(p_runState);
		addUINT32(telegram, p_tachoLimit);

		return telegram;
	}
   
	Telegram TelegramFactory::setInputModeMsg(const uint8_t p_port,
											  const uint8_t p_type,
											  const uint8_t p_mode) const
	{
		Telegram telegram(5);
		
		telegram.add(DIRECT_CMD_NO_REPLY);
		telegram.add(CMD_SET_INPUT_MODE);
		//set sensor port
		telegram.addUINT8(p_port);
		//set sensor type
		telegram.addUINT8(p_type);
		//set sensor mode
		telegram.addUINT8(p_mode);
		
		return telegram;
	}
	
	Telegram TelegramFactory::getOutputStateMsg(const uint8_t p_port) const
	{
		Telegram telegram(3);
		
		telegram.add(DIRECT_CMD);
		telegram.add(CMD_GET_OUTPUT_STATE);
        telegram.addUINT8(p_port);
		
		return telegram;
	}
	
	Telegram TelegramFactory::getInputValuesMsg(const uint8_t p_port) const
	{
		Telegram telegram(3);
		
		telegram.add(DIRECT_CMD);
		telegram.add(CMD_GET_INPUT_VALUES);
		telegram.addUINT8(p_port);
		
		return telegram;
	}
	
	Telegram TelegramFactory::resetInputScaledValueMsg(const uint8_t p_port) const
	{
		Telegram telegram(3);
		
        telegram.add(DIRECT_CMD_NO_REPLY);
        telegram.add(CMD_RESET_INPUT_SCALED_VALUE);
        telegram.addUINT8(p_port);
		
		return telegram;
	}
	
	Telegram TelegramFactory::messageWriteMsg(const uint8_t p_mailbox,
											  const std::string& p_message) const
	{
		if(p_message.length() + 1 > NXT_MAX_MESSAGE_LENGTH)
			throw NXTException("Cannot write message. Message is too long");
			
		Telegram telegram(4 + p_message.length() + 1);
		
		telegram.add(DIRECT_CMD_NO_REPLY);
		telegram.add(CMD_MESSAGE_WRITE);
		telegram.addUINT8(p_mailbox);
		telegram.addUINT8(p_message.length() + 1);
		addString(telegram, p_message);
		
		return telegram;
	}
	
	Telegram TelegramFactory::resetMotorPositionMsg(const uint8_t p_port,
													const uint8_t p_relative) const
	{
		Telegram telegram(4);
		
		telegram.add(DIRECT_CMD_NO_REPLY);
		telegram.add(CMD_RESET_MOTOR_POSITION);
		telegram.addUINT8(p_port);
		telegram.addUINT8(p_relative);
		
		return telegram;
	}

	Telegram TelegramFactory::getBatteryLevelMsg() const
	{
		Telegram telegram(2);
		
		telegram.add(DIRECT_CMD);
        telegram.add(CMD_GET_BATTERY_LEVEL);
		
		return telegram;
	}
	
	Telegram TelegramFactory::stopSoundPlaybackMsg() const
	{
		Telegram telegram(2);
		
		telegram.add(DIRECT_CMD_NO_REPLY);
		telegram.add(CMD_STOP_SOUND_PLAYBACK);
		
		return telegram;
	}
	
	Telegram TelegramFactory::keepAliveMsg() const
	{
		Telegram telegram(2);
		
		telegram.add(DIRECT_CMD);
		telegram.add(CMD_KEEP_ALIVE);
		
		return telegram;
	}
	
	Telegram TelegramFactory::lsGetStatusMsg(const uint8_t p_port) const
	{
		Telegram telegram(3);
		
		telegram.add(DIRECT_CMD);
        telegram.add(CMD_LSGETSTATUS);
        telegram.addUINT8(p_port);
		
		return telegram;
	}
	
	Telegram TelegramFactory::lsWriteMsg(const uint8_t p_port,
										 const uint8_t p_txDataLength,
										 const uint8_t p_rxDataLength,
										 const unsigned char *p_txData) const
	{
		Telegram telegram(5 + p_txDataLength);
		
		telegram.add(DIRECT_CMD_NO_REPLY);
		telegram.add(CMD_LSWRITE);
		telegram.addUINT8(p_port);
		telegram.addUINT8(p_txDataLength);
		telegram.addUINT8(p_rxDataLength);
		telegram.addData(p_txData, p_txDataLength);
		
		return telegram;
	}
	
	Telegram TelegramFactory::lsReadMsg(const uint8_t p_port) const
	{
		Telegram telegram(3);
		
		telegram.add(DIRECT_CMD);
		telegram.add(CMD_LSREAD);
		telegram.addUINT8(p_port);
		
		return telegram;
	}
	
	Telegram TelegramFactory::getCurrentProgramNameMsg() const
	{
		Telegram telegram(2);
		
		telegram.add(DIRECT_CMD);
		telegram.add(CMD_GET_CURRENT_PROGRAM_NAME);
		
		return telegram;
	}
	
	Telegram TelegramFactory::messageReadMsg(const uint8_t p_remoteMailbox,
											 const uint8_t p_localMailbox,
											 const uint8_t p_removeMessage) const
	{
		Telegram telegram(5);
		
		telegram.add(DIRECT_CMD);
		telegram.add(CMD_MESSAGE_READ);
		telegram.addUINT8(p_remoteMailbox);
		telegram.addUINT8(p_localMailbox);
		telegram.addUINT8(p_removeMessage);
		
		return telegram;
	}
	
	void TelegramFactory::addString(Telegram &p_telegram, const std::string &p_string) const
	{
		for(int i = 0; i < p_string.length(); ++i)
			p_telegram.add((unsigned char) p_string[i]);
		p_telegram.add(0x00);
	}
	
	void TelegramFactory::addUINT16(Telegram &p_telegram, const uint16_t p_value) const
	{
		if(isLittleEndian())
			p_telegram.addUINT16(p_value);
		else {
			unsigned char *ptr = (unsigned char*) &p_value;
			p_telegram.add(ptr[1]);
			p_telegram.add(ptr[0]);
		}
	}
	
	void TelegramFactory::addUINT32(Telegram &p_telegram, const uint32_t p_value) const
	{
		if(isLittleEndian())
			p_telegram.addUINT32(p_value);
		else {
			unsigned char *ptr = (unsigned char*) &p_value;
			p_telegram.add(ptr[3]);
			p_telegram.add(ptr[2]);
			p_telegram.add(ptr[1]);
			p_telegram.add(ptr[0]);
		}
	}
	
}