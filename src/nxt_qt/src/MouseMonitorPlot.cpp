#include "nxt_qt/MouseMonitorPlot.hpp"

namespace minotaur
{

    void MouseMonitorPlot::init(std::string title, QColor color)
    {
        curve.setPen(color);
        //curve.setSamples
        curve.attach(this);

        //setTitle(title);
        /*setAxisScale(QwtPlot::xBottom, 0, SAMPLE_RANGE, xStep);
        setAxisScale(QwtPlot::yLeft, -maxYRange, maxYRange, yStep);
        setAxisTitle(QwtPlot::xBottom, xAxisTitle);
        setAxisTitle(QwtPlot::yLeft, yAxisTitle);*/
    }

    void MouseMonitorPlot::updatePlot()
    {

    }

}
