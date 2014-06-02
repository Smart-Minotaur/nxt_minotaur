#ifndef NXT_QT_MAP_WINDOW_HPP_
#define NXT_QT_MAP_WINDOW_HPP_

#include <QtGui/QMainWindow>
#include "ui_map_window.h"
#include "pid_monitor_pc/QMinotaurNavigateNode.hpp"
#include "map_monitor_pc/QMapWidget.hpp"

namespace minotaur {
    
    class MapWindow : public QMainWindow, public Ui::MapWindow
    {
        Q_OBJECT
    private:
        QMinotaurNavigateNode navNode;
        QMapWidget *mapWidget;

        RobotPosition robotPosition;

        float linVel;
        float angVel;
        
    private Q_SLOTS:
        void processOdometry(const QOdometry p_odom);
        void processMeasuredSensor(const QUltraSensor p_sensor);
        
    public:
        MapWindow(QWidget *parent = 0);
        ~MapWindow();
        
        QMinotaurNavigateNode& getNavigationNode();
    };
    
}


#endif
