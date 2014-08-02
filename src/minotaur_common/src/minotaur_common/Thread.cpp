#include <stdexcept>
#include "minotaur_common/Thread.hpp"

namespace minotaur
{
    static void* runThread(void *arg)
    {
        Thread *thread = (Thread*) arg;
        thread->run();
        return NULL;
    }
    
    Thread::Thread()
    :started(false) { }
    Thread::~Thread() { }
    
    void Thread::start()
    {
        if(started)
            throw std::logic_error("Thread is already started. Cannot start again before joining");
        
        onStart();
        if(pthread_create(&thread, NULL, runThread, this) != 0)
            throw std::logic_error("Failed to create thread. Cannot start Thread");
        started = true;
    }
    
    void Thread::stop()
    {
        onStop();
    }
    
    void Thread::join()
    {
        if(!started)
            throw std::logic_error("Thread has not been started. Cannot join Thread");
            
        void *ret;
        pthread_join(thread, &ret);
        started = false;
    }
}
