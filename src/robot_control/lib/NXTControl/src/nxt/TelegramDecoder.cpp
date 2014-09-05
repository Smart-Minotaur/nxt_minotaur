#include "nxt/TelegramDecoder.hpp"
#include "nxt/OPCodes.hpp"
#include "nxt/Exceptions.hpp"

namespace nxt
{
	TelegramDecoder::TelegramDecoder()
	{
		
	}
	
	TelegramDecoder::~TelegramDecoder()
	{
		
	}
	
	OutputState TelegramDecoder::decodeGetOutputStateMsg(const Telegram& p_telegram) const
	{
		unsigned char data[p_telegram.size()];
		p_telegram.getData(data);
		if(data[0] != REPLY_TELEGRAM)
			throw NXTException("Cannot decode telegram. Is no reply-telegram");
		if(data[1] != CMD_GET_OUTPUT_STATE)
			throw NXTException("Cannot decode telegram. Is no GetOutputState telegram");
			
		OutputState result;
		result.status = data[2];
		result.port = data[3];
		result.power = data[4];
		result.motorMode = data[5];
		result.regulationMode = data[6];
		result.turnRatio = data[7];
		result.runState = data[8];
		result.tachoInfo.tachoLimit = getUINT32(data, 9);
		result.tachoInfo.tachoCount = getUINT32(data, 13);
		result.tachoInfo.blockTachoCount = getUINT32(data, 17);
		result.tachoInfo.rotationTachoCount = getUINT32(data, 21);
		
		return result;
	}
	
	InputValues TelegramDecoder::decodeGetInputValuesMsg(const Telegram &p_telegram) const
	{
		unsigned char data[p_telegram.size()];
		p_telegram.getData(data);
		if(data[0] != REPLY_TELEGRAM)
			throw NXTException("Cannot decode telegram. Is no reply-telegram");
		if(data[1] != CMD_GET_INPUT_VALUES)
			throw NXTException("Cannot decode telegram. Is no GetInputValues telegram");
			
		InputValues result;
		result.status = data[2];
		result.port = data[3];
		result.valid = data[4];
		result.calibrated = data[5];
		result.sensorType = data[6];
		result.sensorMode = data[7];
		result.sensorInfo.rawValue = getUINT16(data, 8);
		result.sensorInfo.normalizedValue = getUINT16(data, 10);
		result.sensorInfo.scaledValue = getUINT16(data, 12);
		result.sensorInfo.calibratedValue = getUINT16(data, 14);
		
		return result;
	}
	
	BatteryLevel TelegramDecoder::decodeGetBatteryLevelMsg(const Telegram &p_telegram) const
	{
		unsigned char data[p_telegram.size()];
		p_telegram.getData(data);
		if(data[0] != REPLY_TELEGRAM)
			throw NXTException("Cannot decode telegram. Is no reply-telegram");
		if(data[1] != CMD_GET_BATTERY_LEVEL)
			throw NXTException("Cannot decode telegram. Is no GetBatteryLevel telegram");
			
		BatteryLevel result;
		result.status = data[2];
		result.voltage = getUINT16(data, 3);
		
		return result;
	}
	
	KeepAlive TelegramDecoder::decodeKeepAliveMsg(const Telegram &p_telegram) const
	{
		unsigned char data[p_telegram.size()];
		p_telegram.getData(data);
		if(data[0] != REPLY_TELEGRAM)
			throw NXTException("Cannot decode telegram. Is no reply-telegram");
		if(data[1] != CMD_KEEP_ALIVE)
			throw NXTException("Cannot decode telegram. Is no KeepAlive telegram");
			
		KeepAlive result;
		result.status = data[2];
		result.sleepTimeLimit = getUINT32(data, 3);
		
		return result;
	}
	
	LSStatus TelegramDecoder::decodeGetLSStatusMsg(const Telegram &p_telegram) const
	{
		unsigned char data[p_telegram.size()];
		p_telegram.getData(data);
		if(data[0] != REPLY_TELEGRAM)
			throw NXTException("Cannot decode telegram. Is no reply-telegram");
		if(data[1] != CMD_LSGETSTATUS)
			throw NXTException("Cannot decode telegram. Is no LSGetStatus telegram");
			
		LSStatus result;
		result.status = data[2];
		result.readyBytesCount = data[3];
		
		return result;
	}
	
	LSRead TelegramDecoder::decodeLSReadMsg(const Telegram &p_telegram) const
	{
		unsigned char data[p_telegram.size()];
		p_telegram.getData(data);
		if(data[0] != REPLY_TELEGRAM)
			throw NXTException("Cannot decode telegram. Is no reply-telegram");
		if(data[1] != CMD_LSREAD)
			throw NXTException("Cannot decode telegram. Is no LSRead telegram");
		
		LSRead result;
		result.status = data[2];
		result.bytesRead = data[3];
		for(int i = 0; i < 16; ++i)
			result.data[i] = data[i + 4];
		
		return result;
	}
	
	ProgramName TelegramDecoder::decodeGetCurrentProgramNameMsg(const Telegram &p_telegram) const
	{
		unsigned char data[p_telegram.size()];
		p_telegram.getData(data);
		if(data[0] != REPLY_TELEGRAM)
			throw NXTException("Cannot decode telegram. Is no reply-telegram");
		if(data[1] != CMD_GET_CURRENT_PROGRAM_NAME)
			throw NXTException("Cannot decode telegram. Is no GetCurrentProgramName telegram");
			
		ProgramName result;
		char tmpName[NXT_MAX_FILE_NAME_LENGTH];
		result.status = data[2];
		for(int i = 0; i < NXT_MAX_FILE_NAME_LENGTH; ++i)
			tmpName[i] = data[i + 3];
		result.name = std::string(tmpName);
		
		return result;
	}
	
	MailboxMessage TelegramDecoder::decodeMessageReadMsg(const Telegram &p_telegram) const
	{
		unsigned char data[p_telegram.size()];
		p_telegram.getData(data);
		if(data[0] != REPLY_TELEGRAM)
			throw NXTException("Cannot decode telegram. Is no reply-telegram");
		if(data[1] != CMD_MESSAGE_READ)
			throw NXTException("Cannot decode telegram. Is no MessageRead telegram");
			
		MailboxMessage result;
		char tmpMessage[NXT_MAX_MESSAGE_LENGTH];
		result.status = data[2];
		result.mailboxNumber = data[3];
		for(int i = 0; i < NXT_MAX_MESSAGE_LENGTH; ++i)
			tmpMessage[i] = data[i + 4];
		result.message = std::string(tmpMessage);
		
		return result;
	}
	
	uint32_t TelegramDecoder::getUINT32(const unsigned char *p_data, const int p_index) const
	{
		uint32_t result;
		uint8_t *ptr = (uint8_t*) &result;
		if(isLittleEndian()) {
			ptr[0] = p_data[p_index];
			ptr[1] = p_data[p_index + 1];
			ptr[2] = p_data[p_index + 2];
			ptr[3] = p_data[p_index + 3];
		} else {
			ptr[0] = p_data[p_index + 3];
			ptr[1] = p_data[p_index + 2];
			ptr[2] = p_data[p_index + 1];
			ptr[3] = p_data[p_index];
		}
		
		return result;
	}
	
	uint16_t TelegramDecoder::getUINT16(const unsigned char *p_data, const int p_index) const
	{
		uint16_t result;
		uint8_t *ptr = (uint8_t*) &result;
		if(isLittleEndian()) {
			ptr[0] = p_data[p_index];
			ptr[1] = p_data[p_index + 1];
		} else {
			ptr[0] = p_data[p_index + 1];
			ptr[1] = p_data[p_index];
		}
		
		return result;
	}
}