#ifndef MINOTAUR_RAII_LOCK
#define MINOTAUR_RAII_LOCK

#include <pthread.h>

namespace minotaur
{
    class RAIILock
    {
    private:
        pthread_mutex_t *mutex;
    public:
        RAIILock(pthread_mutex_t *p_mutex): mutex(p_mutex)
        { pthread_mutex_lock(mutex); }
        ~RAIILock()
        { pthread_mutex_unlock(mutex); }
    };
}

#endif
