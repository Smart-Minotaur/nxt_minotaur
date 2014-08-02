#ifndef MINOTAUR_THREAD_HPP
#define MINOTAUR_THREAD_HPP

#include <pthread.h>

namespace minotaur
{
    /* This Thread class is an object oriented representation of a
     * posix thread. It only offers minimal functionality. */
    class Thread
    {
    private:
        pthread_t thread;
        bool started;
    protected:
        virtual void onStart() = 0;
        virtual void onStop() = 0;
    public:
        Thread();
        virtual ~Thread();
        
        void start();
        void stop();
        void join();
        
        virtual void run() = 0;
    };
}

#endif
