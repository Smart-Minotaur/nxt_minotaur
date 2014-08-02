#include "minotaur_common_qt/MetaTypes.hpp"

namespace minotaur
{
    void registerMetatypes()
    {
        qRegisterMetaType<minotaur::QOdometry>("QOdometry");
        qRegisterMetaType<minotaur::QUltrasonicData>("QUltrasonicData");
    }
}
