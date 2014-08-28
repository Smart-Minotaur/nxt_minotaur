#include "nxt/Brick.hpp"
#include "nxt/OPCodes.hpp"
#include "nxt/Exceptions.hpp"

namespace nxt
{
	
	class Lock
	{
	private:
		pthread_mutex_t *mutex;
	public:
		Lock(pthread_mutex_t *p_mutex)
		:mutex(p_mutex)
		{
			pthread_mutex_lock(mutex);
		}
		
		~Lock()
		{
			pthread_mutex_unlock(mutex);
		}
	};

	Brick::Brick()
	:connected(false)
	{
		pthread_mutex_init(&receiveMutex, NULL);
		pthread_mutex_init(&sendMutex, NULL);
	}

	Brick::~Brick()
	{
		if(connected)
			socket.close();
			
		pthread_mutex_destroy(&sendMutex);
		pthread_mutex_destroy(&receiveMutex);
	}
	
	void Brick::send(const Telegram &p_telegram)
	{
		Lock lock(&sendMutex);
		if(!connected)
			throw NXTException("Cannot send Telegram. Brick is not connected");
		socket.send(p_telegram, NXT_USB_OUT_ENDPOINT);
	}
	
	Telegram Brick::receive()
	{
		Lock lock(&receiveMutex);
		if(!connected)
			throw NXTException("Cannot receive Telegram. Brick is not connected");
		return socket.receive(NXT_USB_IN_ENDPOINT);
	}
	
	void Brick::connect(const int p_interface)
	{
		if(connected)
			throw NXTException("Cannot connect Brick. Is already connected");
		socket.open(LEGO_VENDOR_ID, NXT_PRODUCT_ID, p_interface);
		connected = true;
	}
	
	void Brick::disconnect()
	{
		if(!connected)
			throw NXTException("Cannot disconnect Brick. Is not connected");
		socket.close();
		connected = false;
	}
	
	void Brick::startProgram(const std::string &p_fileName)
	{
		send(telegramFactory.startProgramMsg(p_fileName));
	}
	
	void Brick::stopProgram()
	{
		send(telegramFactory.stopProgramMsg());
	}
	
	void Brick::playSoundFile(const uint8_t p_loop,
							  const std::string &p_fileName)
	{
		send(telegramFactory.playSoundFileMsg(p_loop, p_fileName));
	}
	
	void Brick::playTone(const uint16_t p_frequency,
						 const uint16_t p_durationMS)
	{
		send(telegramFactory.playToneMsg(p_frequency, p_durationMS));
	}
	
	void Brick::setOutputState(const uint8_t p_port,
							   const uint8_t p_power,
							   const uint8_t p_motorMode,
							   const uint8_t p_regulationMode,
							   const uint8_t p_turnRatio,
							   const uint8_t p_runState,
							   const uint32_t p_tachoLimit)
	{
		send(telegramFactory.setOutputStateMsg(p_port,
											   p_power,
											   p_motorMode,
											   p_regulationMode,
											   p_turnRatio,
											   p_runState,
											   p_tachoLimit));
	}
	
	void Brick::setInputMode(const uint8_t p_port,
							 const uint8_t p_type,
							 const uint8_t p_mode)
	{
		send(telegramFactory.setInputModeMsg(p_port, p_type, p_mode));
	}
	
	void Brick::resetInputScaledValue(const uint8_t p_port)
	{
		send(telegramFactory.resetInputScaledValueMsg(p_port));
	}
	
	void Brick::writeMessage(const uint8_t p_mailbox,
							 const std::string& p_message)
	{
		send(telegramFactory.messageWriteMsg(p_mailbox, p_message));
	}
	
	void Brick::resetMotorPosition(const uint8_t p_port,
								   const uint8_t p_relative)
	{
		send(telegramFactory.resetMotorPositionMsg(p_port, p_relative));
	}
	
	void Brick::stopSoundPlayback()
	{
		send(telegramFactory.stopSoundPlaybackMsg());
	}
	
	void Brick::lsWrite(const uint8_t p_port,
					    const uint8_t p_txDataLength,
					    const uint8_t p_rxDataLength,
					    const unsigned char *p_txData)
	{
		send(telegramFactory.lsWriteMsg(p_port,
										p_txDataLength,
										p_rxDataLength,
										p_txData));
	}
	
	OutputState Brick::getOutputState(const uint8_t p_port)
	{
		send(telegramFactory.getOutputStateMsg(p_port));
		return telegramDecoder.decodeGetOutputStateMsg(receive());
	}
	
	InputValues Brick::getInputValues(const uint8_t p_port)
	{
		send(telegramFactory.getInputValuesMsg(p_port));
		return telegramDecoder.decodeGetInputValuesMsg(receive());
	}
	
	BatteryLevel Brick::getBatteryLevel()
	{
		send(telegramFactory.getBatteryLevelMsg());
		return telegramDecoder.decodeGetBatteryLevelMsg(receive());
	}
	
	KeepAlive Brick::keepAlive()
	{
		send(telegramFactory.keepAliveMsg());
		return telegramDecoder.decodeKeepAliveMsg(receive());
	}
	
	LSStatus Brick::lsGetStatus(const uint8_t p_port)
	{
		send(telegramFactory.lsGetStatusMsg(p_port));
		return telegramDecoder.decodeGetLSStatusMsg(receive());
	}
	
	LSRead Brick::lsRead(const uint8_t p_port)
	{
		send(telegramFactory.lsReadMsg(p_port));
		return telegramDecoder.decodeLSReadMsg(receive());
	}
	
	ProgramName Brick::getCurrentProgramName()
	{
		send(telegramFactory.getCurrentProgramNameMsg());
		return telegramDecoder.decodeGetCurrentProgramNameMsg(receive());
	}
	
	MailboxMessage Brick::readMessage(const uint8_t p_remoteMailbox,
								      const uint8_t p_localMailbox,
								      const uint8_t p_removeMessage)
	{
		send(telegramFactory.messageReadMsg(p_remoteMailbox, p_localMailbox, p_removeMessage));
		return telegramDecoder.decodeMessageReadMsg(receive());
	}
}

