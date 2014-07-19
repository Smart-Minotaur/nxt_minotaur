#ifndef MINOTAUR_PLOCK_HPP
#define MINOTAUR_PLOCK_HPP

#include <pthread.h>

namespace minotaur
{
    class PLock
    {
	private:
		pthread_mutex_t *mutex;
	public:
		PLock(pthread_mutex_t *p_mutex)
		:mutex(p_mutex) { pthread_mutex_lock(mutex); }
		~PLock() { pthread_mutex_unlock(mutex); }
        
    };
}

#endif
