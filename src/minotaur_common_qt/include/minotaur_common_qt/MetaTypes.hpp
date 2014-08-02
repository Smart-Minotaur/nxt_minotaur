#ifndef MINOTAUR_META_TYPES_HPP
#define MINOTAUR_META_TYPES_HPP

#include <QMetaType>
#include <nav_msgs/Odometry.h>
#include "minotaur_common/UltrasonicData.h"

namespace minotaur
{
    void registerMetatypes();
    
    class QOdometry
    {
    public:
        nav_msgs::Odometry odometry;
        
        QOdometry() { }
        ~QOdometry() { }
    };
    
    class QUltrasonicData
    {
    public:
        minotaur_common::UltrasonicData data;
        
        QUltrasonicData() { }
        ~QUltrasonicData() { }
    };
}

Q_DECLARE_METATYPE(minotaur::QOdometry);
Q_DECLARE_METATYPE(minotaur::QUltrasonicData);

#endif
