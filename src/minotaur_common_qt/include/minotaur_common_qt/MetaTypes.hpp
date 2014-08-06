/**
 * \file MetaTypes.hpp
 * \brief This file contains MetaTypes for QT, which can be sent by
 *        QSignals and recieved with Slots.
 */

#ifndef MINOTAUR_META_TYPES_HPP
#define MINOTAUR_META_TYPES_HPP

#include <QMetaType>
#include <nav_msgs/Odometry.h>
#include "minotaur_common/UltrasonicData.h"

namespace minotaur
{
    /**
     * This function registers the meta types defined in MetaTypes.hpp.
     * Therefore it must be called before the meta types can be used.
     */
    void registerMetatypes();
    
    /**
     * \brief The QOdometry class is a QT wrapper for the Odometry message of
     *        ROS. Therefore it can be sent with QSignals.
     */
    class QOdometry
    {
    public:
        nav_msgs::Odometry odometry;
        
        QOdometry() { }
        ~QOdometry() { }
    };
    
    /**
     * \brief The QUltrasonicData class is a QT wrapper for the UltrasonicData
     *        message of the minotaur project. Therefore it can be sent
     *        with QSignals.
     */
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
