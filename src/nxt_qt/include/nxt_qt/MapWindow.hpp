#ifndef NXT_QT_MAP_WINDOW_HPP_
#define NXT_QT_MAP_WINDOW_HPP_

#include <QtGui/QMainWindow>
#include <QKeyEvent>
#include "ui_map_window.h"
#include "nxt_qt/QMinotaurNavigateNode.hpp"
#include "nxt_qt/QMapWidget.hpp"

namespace minotaur {
    
    class MapWindow : public QMainWindow, public Ui::MapWindow
    {
        Q_OBJECT
    private:
        QMinotaurNavigateNode navNode;
        QMapWidget *mapWidget;

        
        float linVel;
        float angVel;
        
    private Q_SLOTS:
        void processOdometry(const QOdometry p_odom);
        void processMeasuredSensor(const QUltraSensor p_sensor);
        
    public:
        MapWindow(QWidget *parent = 0);
        ~MapWindow();
        
        QMinotaurNavigateNode& getNavigationNode();
        
    protected:
        void keyPressEvent(QKeyEvent *p_event);
        void keyReleaseEvent(QKeyEvent *p_event);
    };
    
}


#endif