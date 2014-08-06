#ifndef MINOTAUR_QMINOTAUR_CONTROL_THREAD_HPP
#define MINOTAUR_QMINOTAUR_CONTROL_THREAD_HPP

#include <QThread>
#include "minotaur_common_qt/QMinotaurControlNode.hpp"

namespace minotaur
{
    /**
     * \brief The QMinotaurControlThread class is used to run a
     *        QMinotaurControlNode in a seperate thread.
     * 
     * The QMinotaurControlNode object is set in 
     * QMinotaurControlThread(QMinotaurControlNode &p_controlNode) and
     * cannot be changed after construction.
     * After calling start(), the thread will spin the control node and
     * process incoming and outgoing messages.
     */
    class QMinotaurControlThread: public QThread
    {
        Q_OBJECT
    private:
        QMinotaurControlNode &controlNode;
        
        void run();
    public:
        QMinotaurControlThread(QMinotaurControlNode &p_controlNode);
        ~QMinotaurControlThread();
    public Q_SLOTS:
        void stop();
    };
};

#endif
