#ifndef MOUSE_MONITOR_PLOT_H
#define MOUSE_MONITOR_PLOT_H

#include <qwt_plot_curve.h>
#include <qwt_plot_layout.h>

namespace minotaur
{

    class MouseMonitorPlot : public QwtPlot
    {
            Q_OBJECT

        private:
            QwtPlotCurve curve;

            //double values[];

        public:
            MouseMonitorPlot(QwtPlot *parent = 0) : QwtPlot(parent) {}
            virtual ~MouseMonitorPlot() {}

            void init(std::string title, QColor color);
            void updatePlot();
    };

}

#endif
