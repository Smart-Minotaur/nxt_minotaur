#ifndef NXT_BRICK_HPP
#define NXT_BRICK_HPP

#include <pthread.h>
#include "nxt/USBSocket.hpp"
#include "nxt/TelegramFactory.hpp"
#include "nxt/TelegramDecoder.hpp"

namespace nxt
{
	/**
	* \brief Used for communication with a NXT brick.
	*
	* This class is used to connect and communicate with
	* a LEGO NXT Brick. Telegram messages are sent via
	* USB to the Brick and responses can be received.
	*/
	class Brick
	{
	private:
		USBSocket socket;
		
		pthread_mutex_t receiveMutex;
		pthread_mutex_t sendMutex;
		
		TelegramFactory telegramFactory;
		TelegramDecoder telegramDecoder;
		bool connected;
		
		void send(const Telegram &p_telegram);
		Telegram sendAndReceive(const Telegram &p_telegram);
	public:
		Brick();
		~Brick();
		
		void connect(const int p_interface = 0);
		void disconnect();
		
		/**
		 * Starts a program on the brick with the given name.
		 * @param p_fileName name of the program
		 */
		void startProgram(const std::string &p_fileName);
		
		/**
		 * Stops the currently running program on the brick.
		 */
		void stopProgram();
		
		/**
		 * Plays a sound on the brick with the given name.
		 * @param p_loop determines if sound should restart
		 * @param p_fileName name of the sound file
		 */
		void playSoundFile(const uint8_t p_loop,
						   const std::string &p_fileName);
		/**
		* Plays a tone on the brick.
		* @param p_frequency frequency of the tone
		* @param p_durationMS duration of the tone
		*/
		void playTone(const uint16_t p_frequency,
					  const uint16_t p_durationMS);
					  
		/**
		 * Allows to control the motor with the given port.
		 * @param p_port port of the motor
		 * @param p_power power in percent [-100;100]
		 * @param p_motorMode operation mode of the motor
		 * @param p_regulationMode sets auto regulation mode of the motor (sync)
		 * @param p_turnRation 
		 * @param p_runState determines the running mode of the motor
		 * @param p_tachoLimit sets the tacho limit
		 */
		void setOutputState(const uint8_t p_port,
						    const uint8_t p_power,
						    const uint8_t p_motorMode,
						    const uint8_t p_regulationMode,
						    const uint8_t p_turnRatio,
						    const uint8_t p_runState,
						    const uint32_t p_tachoLimit);
							
		/**
		 * Sets type and mode of a sensor with the given port.
		 * @param p_port port of the sensor
		 * @param p_type type of the sensor
		 * @param p_mode operation mode of the sensor
		 */
		void setInputMode(const uint8_t p_port,
						  const uint8_t p_type,
						  const uint8_t p_mode);
		
		/**
		 * Resets the scaled output value of a sensor with
		 * the given port.
		 * @param p_port port of the sensor
		 */
		void resetInputScaledValue(const uint8_t p_port);
		
		/**
		 * Sends a string message to the given mailbox of the
		 * brick. The maximum message length is 58 characters.
		 * @param p_mailbox number of the target mailbox
		 * @param p_message message to be sent to the mailbox
		 */
		void writeMessage(const uint8_t p_mailbox,
						  const std::string& p_message);
						  
		/**
		 * Resets the motor data e.g. tachoCount.
		 * @param p_port port of the motor
		 * @param p_relative true: relative to last position. false: absolute position
		 */
		void resetMotorPosition(const uint8_t p_port,
								const uint8_t p_relative);
								
		/**
		 * Stops the currently running sound.
		 */
		void stopSoundPlayback();
		
		void lsWrite(const uint8_t p_port,
					 const uint8_t p_txDataLength,
					 const uint8_t p_rxDataLength,
					 const unsigned char *p_txData);
		
		/**
		 * Reads the current state of the given motor.
		 * @param p_port port of the motor
		 * @return state of the motor
		 */
		OutputState getOutputState(const uint8_t p_port);
		
		/**
		 * Reads the current measured values of the given sensor.
		 * @param p_port port of the sensor
		 * @return measured values of the sensor
		 */
		InputValues getInputValues(const uint8_t p_port);
		
		/**
		 * Reads the current battery level of the brick. The
		 * voltage is measured in millivolts.
		 * @return voltage level of the brick
		 */
		BatteryLevel getBatteryLevel();
		KeepAlive keepAlive();
		LSStatus lsGetStatus(const uint8_t p_port);
		LSRead lsRead(const uint8_t p_port);
		
		/**
		 * Reads the program name of the currently running
		 * program.
		 * @return program name
		 */
		ProgramName getCurrentProgramName();
		
		/**
		 * Reads the first message in the message queue of the given
		 * mailbox.
		 * @param p_remoteMailbox mailbox on another connected brick
		 * @param p_localMailbox mailbox on the current brick
		 * @param p_removeMessage true if message should be deleted after reading
		 * @return message received from the mailbox
		 */
		MailboxMessage readMessage(const uint8_t p_remoteMailbox,
								   const uint8_t p_localMailbox,
								   const uint8_t p_removeMessage);

	};
}

#endif
