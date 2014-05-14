#ifndef MINOTAUR_MINOTAUR_THREAD_HPP_
#define MINOTAUR_MINOTAUR_THREAD_HPP_

#include <pthread.h>

namespace minotaur
{
    class MinotaurThread
    {
    private:
        pthread_t thread;
    public:
        int exitCode;
        MinotaurThread() { }
        virtual ~MinotaurThread() { }
        
        void start();
        void stop();
        virtual int run() = 0;
        int join();
        
    };
}

#endif