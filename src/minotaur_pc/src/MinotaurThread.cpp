#include "minotaur_pc/MinotaurThread.hpp"
#include <signal.h>

namespace minotaur
{
    void *runThread(void *arg);
    
    void MinotaurThread::start()
    {
        pthread_create(&thread, NULL, runThread, this);
    }
    
    void *runThread(void *arg)
    {
        int result;
        MinotaurThread *thread = (MinotaurThread*) arg;
        thread->exitCode = thread->run();
        return &(thread->exitCode);
    }
    
    void MinotaurThread::stop()
    {
        pthread_kill(thread, 9);
    }
    
    int MinotaurThread::join()
    {
        void *result;
        pthread_join(thread, &result);
        return *((int*) result);
    }
}