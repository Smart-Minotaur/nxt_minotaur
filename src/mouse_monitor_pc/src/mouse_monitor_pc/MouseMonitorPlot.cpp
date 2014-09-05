#include "mouse_monitor_pc/MouseMonitorPlot.hpp"

#define DEFAULT_X_STEP 30
#define DEFAULT_Y_STEP 10
#define DEFAULT_MAX_SIZE 400

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
		setAxisScale(QwtPlot::xBottom, 0, maxSize, xStep);
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

	void MouseMonitorPlot::clear()
	{
		yData.clear();
		xData.clear();
		
		curve.setSamples(xData, yData);

		replot();
	}

}
