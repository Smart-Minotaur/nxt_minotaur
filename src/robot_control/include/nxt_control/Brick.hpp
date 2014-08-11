#ifndef NXT_CONTROL_BRICK_HPP_
#define NXT_CONTROL_BRICK_HPP_

#include <pthread.h>
#include "nxt_control/USBSocket.hpp"
#include "nxt_control/Telegram.hpp"

namespace nxtcon
{
    /**
     * \brief Used for communication with the brick
     * 
     * This class is used to connect and communicate with the Brick.
     * Telegram messages are send via USB to the Brick.
     * There a two types of messages that can be send:
     *  - send(): send a one-way-message
     *  - sendWithResponse(): waiting for a response
     */
    class Brick
    {
    private:
        USBSocket usbSocket;
        bool connected;
        pthread_mutex_t usbOutMutex;
        pthread_mutex_t usbInMutex;
    public:
        Brick()
        :usbSocket(), connected(false)
        {
            pthread_mutex_init(&usbOutMutex, NULL);
            pthread_mutex_init(&usbInMutex, NULL);
        }
        virtual  ~Brick();
        
	/**
	 * searches for the USB-Port of the brick and builds up a connection to it
	 * @throws logic_error if a brick is already connected
	 */
        void find(const int p_interface = 0);
        bool isConnected() const;
        
	/**
	 * Sends a one-way-Telegram to the brick
	 */
        void send(const Telegram &p_telegram);
	/**
	 * Sends a Telegram and waiting for a response
	 * @param p_telegram Message to be send
	 * @retval sendWithResponse Response-Telegram from the brick
	 */
        Telegram sendWithResponse(const Telegram &p_telegram);
        
	/**
	 * Plays a sound on the brick.
	 * This was used to test if a brick with damaged display is ready
	 * @param p_frequency frequency of the sounds
	 * @param p_durationMS duration of one tone
	 */
        void playTone(const uint16_t p_frequency, const uint16_t p_durationMS);
        uint16_t getBatteryLevel();
    };
    
}

#endif
