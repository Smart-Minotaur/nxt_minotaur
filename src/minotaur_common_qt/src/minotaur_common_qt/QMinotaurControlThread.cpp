#include "minotaur_common_qt/QMinotaurControlThread.hpp"

namespace minotaur
{
    QMinotaurControlThread::QMinotaurControlThread(QMinotaurControlNode &p_controlNode)
    :controlNode(p_controlNode)
    {
        
    }
    
    QMinotaurControlThread::~QMinotaurControlThread()
    {
        
    }
    
    void QMinotaurControlThread::run()
    {
        controlNode.spin();
    }
    
    void QMinotaurControlThread::stop()
    {
        controlNode.stop();
    }
}
