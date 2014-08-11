#ifndef NXT_QT_MAP_WINDOW_HPP_
#define NXT_QT_MAP_WINDOW_HPP_

#include <QtGui/QMainWindow>
#include "ui_map_window.h"
#include "map_monitor/QMapWidget.hpp"
#include "minotaur_common/SensorSettings.hpp"
#include "minotaur_common_qt/QMinotaurControlNode.hpp"

namespace minotaur
{
    /**
     * \brief Displays a live map of the histogram
     * 
     * This class displays a visualization of a Map histogram. The robot
     * can be controlled via keyboard.
     */
    class MapWindow : public QMainWindow, public Ui::MapWindow
    {
        Q_OBJECT
    private:
        std::vector<SensorSetting> sensorSettings;
        QMinotaurControlNode controlNode;
        QMapWidget *mapWidget;

        RobotPosition robotPosition;

        float linVel;
        float angVel;
        
    private Q_SLOTS:
        void processOdometry(const QOdometry p_odom);
        void processMeasuredSensor(const QUltrasonicData p_sensor);
        
    public:
        MapWindow(QWidget *parent = 0);
        ~MapWindow();
        
        QMinotaurControlNode& getControlNode();
    };
    
}


#endif
