#include "nxt_qt/MouseMonitorPlot.hpp"

// TODO
#define DEFAULT_X_STEP 10
#define DEFAULT_Y_STEP 10
#define DEFAULT_MAX_SIZE 300

namespace minotaur
{

    void MouseMonitorPlot::init(QColor color,
                                std::string title,
                                std::string xAxisTitle,
                                std::string yAxisTitle)
    {
        xStep = DEFAULT_X_STEP;
        yStep = DEFAULT_Y_STEP;
        maxSize = DEFAULT_MAX_SIZE;

        curve.setPen(color);
        curve.setSamples(xData, yData);
        curve.attach(this);

        setTitle(QString(title.c_str()));
        /*setAxisScale(QwtPlot::xBottom, 0, SAMPLE_RANGE, xStep);
        setAxisScale(QwtPlot::yLeft, -maxYRange, maxYRange, yStep);*/
        setAxisTitle(QwtPlot::xBottom, QString(xAxisTitle.c_str()));
        setAxisTitle(QwtPlot::yLeft, QString(yAxisTitle.c_str()));
    }

    void MouseMonitorPlot::updatePlot(double data)
    {
        xData.append(xData.size());
        yData.append(data);

        curve.setSamples(xData, yData);

        if (xData.size() == maxSize) {
            xData.clear();
            yData.clear();
        }

        replot();
    }

}
