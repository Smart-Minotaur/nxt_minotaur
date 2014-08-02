#ifndef NXT_QT_LOCK_HPP_
#define NXT_QT_LOCK_HPP_

#include <QMutex>

namespace minotaur
{
    class Lock
    {
    private:
        QMutex &mutex;
        
    public:
        Lock(QMutex &p_mutex)
        : mutex(p_mutex) { mutex.lock(); }
        
        ~Lock() { mutex.unlock(); }
    };
}
#endif