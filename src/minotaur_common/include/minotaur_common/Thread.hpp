#ifndef MINOTAUR_THREAD_HPP
#define MINOTAUR_THREAD_HPP

#include <pthread.h>

namespace minotaur
{
    /**
     * \brief The Thread class is an object oriented representation of a
     *        posix thread.
     * 
     * It only offers minimal functionality like starting, stopping and
     * joining the thread. To create a thread another class must inherit
     * from Thread and implement the pure virtual functions onStart(),
     * onStop() and run().
     */
    class Thread
    {
    private:
        pthread_t thread;
        bool started;
    protected:
        /**
         * This method is called before the thread is started (call of
         * start()). Any adjustments of members should be done here.
         */
        virtual void onStart() = 0;
        
        /**
         * This method is called when stop() is called. The correct
         * stopping of the thread should be done here (e.g. set a bool
         * to false).
         */
        virtual void onStop() = 0;
    public:
        Thread();
        virtual ~Thread();
        
        void start();
        void stop();
        void join();
        
        /**
         * The content of this method is executed in the thread. All work
         * done by the thread should be implemented here.
         */
        virtual void run() = 0;
    };
}

#endif
