#ifndef MINOTAUR_QMINOTAUR_CONTROL_THREAD_HPP
#define MINOTAUR_QMINOTAUR_CONTROL_THREAD_HPP

#include <QThread>
#include "minotaur_common_qt/QMinotaurControlNode.hpp"

namespace minotaur
{
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
