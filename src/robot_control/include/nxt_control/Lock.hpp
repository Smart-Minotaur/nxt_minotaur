#ifndef NXT_CON_LOCK_HPP_
#define NXT_CON_LOCK_HPP_

#include <pthread.h>

namespace nxtcon
{
    /**
     * \brief mutex for Brick-communication
     * 
     * Used in Brick
     */
    class Lock
    {
    private:
        pthread_mutex_t *mutex;
    public:
        Lock(pthread_mutex_t *p_mutex)
        :mutex(p_mutex) { pthread_mutex_lock(mutex); }
        
        virtual ~Lock()
        { pthread_mutex_unlock(mutex); }
    };
}

#endif