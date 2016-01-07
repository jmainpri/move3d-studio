/*
 * Copyright (c) 2010-2014 LAAS/CNRS, WPI
 * All rights reserved.
 *
 * Redistribution  and  use  in  source  and binary  forms,  with  or  without
 * modification, are permitted provided that the following conditions are met:
 *
 *   1. Redistributions of  source  code must retain the  above copyright
 *      notice and this list of conditions.
 *   2. Redistributions in binary form must reproduce the above copyright
 *      notice and  this list of  conditions in the  documentation and/or
 *      other materials provided with the distribution.
 *
 * THE SOFTWARE  IS PROVIDED "AS IS"  AND THE AUTHOR  DISCLAIMS ALL WARRANTIES
 * WITH  REGARD   TO  THIS  SOFTWARE  INCLUDING  ALL   IMPLIED  WARRANTIES  OF
 * MERCHANTABILITY AND  FITNESS.  IN NO EVENT  SHALL THE AUTHOR  BE LIABLE FOR
 * ANY  SPECIAL, DIRECT,  INDIRECT, OR  CONSEQUENTIAL DAMAGES  OR  ANY DAMAGES
 * WHATSOEVER  RESULTING FROM  LOSS OF  USE, DATA  OR PROFITS,  WHETHER  IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR  OTHER TORTIOUS ACTION, ARISING OUT OF OR
 * IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.                                  
 *
 * Siméon, T., Laumond, J. P., & Lamiraux, F. (2001). 
 * Move3d: A generic platform for path planning. In in 4th Int. Symp.
 * on Assembly and Task Planning.
 *
 *                                               Jim Mainprice Tue 27 May 2014 
 */
#include <stdlib.h>
#include <qapplication.h>
#include <qpen.h>
#include <qwt_plot.h>
#include <qwt_plot_grid.h>
#include <qwt_plot_marker.h>
//#include <qwt_interval_data.h>
#include "histogramItem.hpp"

#include "histoWin.hpp"
#include <algorithm>
#include <iostream>

#include "utils/SaveContext.hpp"

using namespace std;

HistoWindow::HistoWindow()
{

	plot = new QwtPlot;
    plot->setCanvasBackground(QColor(Qt::white));
    plot->setTitle("Histogram");

    QwtPlotGrid *grid = new QwtPlotGrid;
    grid->enableXMin(true);
    grid->enableYMin(true);
    grid->setMajPen(QPen(Qt::black, 0, Qt::DotLine));
    grid->setMinPen(QPen(Qt::gray, 0 , Qt::DotLine));
    grid->attach(plot);

    HistogramItem *histogram = new HistogramItem();
    histogram->setColor(Qt::darkCyan);

    int numValues(0);

    if( storedContext.getNumberStored() >0)
    {
    	numValues = storedContext.getTime(0).size();

    	for(int i=0;i<numValues;i++)
    	{
    		cout << "Time of run i= " << storedContext.getTime(0)[i] << endl;
    	}
    }

    QwtArray<QwtDoubleInterval> intervals(numValues);
    QwtArray<double> values(numValues);

    double pos = 0.0;
    for ( int i = 0; i < (int)intervals.size(); i++ )
    {
        const int width = 1;

        intervals[i] = QwtDoubleInterval(pos, pos + double(width));
        values[i] = storedContext.getTime(0)[i];

        pos += width;
    }

    histogram->setData(QwtIntervalData(intervals, values));
    histogram->attach(plot);

    double max =0;

    if( storedContext.getNumberStored() >0)
    {
		max = *std::max_element(
				storedContext.getTime(0).begin(),
				storedContext.getTime(0).end());
    }

    plot->setAxisScale(QwtPlot::yLeft, 0.0, max);
    plot->setAxisScale(QwtPlot::xBottom, 0.0, pos);
    plot->replot();

#if QT_VERSION < 0x040000
    a.setMainWidget(&plot);
#endif

}

void HistoWindow::startWindow()
{
    plot->resize(600,400);
    plot->show();
}
