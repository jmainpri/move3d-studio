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
#include <qwt_painter.h>
#include <qwt_plot_canvas.h>
#include <qwt_plot_marker.h>
#include <qwt_plot_curve.h>
#include <qwt_scale_widget.h>
#include <qwt_legend.h>
#include <qwt_scale_draw.h>
#include <qwt_math.h>

#include "dataPlot.hpp"
#include "../p3d/env.hpp"
#include "planner/planEnvironment.hpp"
#include <algorithm>

//
//  Initialize main window
//
DataPlot::DataPlot(QWidget *parent):
QwtPlot(parent),
d_interval(0),
d_timerId(-1)

{
	// Disable polygon clipping
	QwtPainter::setDeviceClipping(false);
	
	// We don't need the cache here
	canvas()->setPaintAttribute(QwtPlotCanvas::PaintCached, false);
	canvas()->setPaintAttribute(QwtPlotCanvas::PaintPacked, false);
	
#if QT_VERSION >= 0x040000
#ifdef Q_WS_X11
	/*
	 Qt::WA_PaintOnScreen is only supported for X11, but leads
	 to substantial bugs with Qt 4.2.x/Windows
	 */
	canvas()->setAttribute(Qt::WA_PaintOnScreen, true);
#endif
#endif
	
	alignScales();
	
	//  Initialize data
	for (int i = 0; i< DATA_PLOT_SIZE; i++)
	{
		d_x[i] = 0.5 * i;     // time axis
		d_y[i] = 0;
		d_z[i] = 0;
	}
	
	// Assign a title
	setTitle("Temperature");
	insertLegend(new QwtLegend(), QwtPlot::BottomLegend);
	
	// Insert new curves
	QwtPlotCurve *cRight = new QwtPlotCurve("Data Moving Right");
	cRight->attach(this);
	
	QwtPlotCurve *cLeft = new QwtPlotCurve("Data Moving Left");
	cLeft->attach(this);
	
	// Set curve styles
	cRight->setPen(QPen(Qt::red));
	cLeft->setPen(QPen(Qt::blue));
	
	// Attach (don't copy) data. Both curves use the same x array.
	cRight->setRawData(d_x, d_y, DATA_PLOT_SIZE);
	cLeft->setRawData(d_x, d_z, DATA_PLOT_SIZE);
	
#if 0
	//  Insert zero line at y = 0
	QwtPlotMarker *mY = new QwtPlotMarker();
	mY->setLabelAlignment(Qt::AlignRight|Qt::AlignTop);
	mY->setLineStyle(QwtPlotMarker::HLine);
	mY->setYValue(0.0);
	mY->attach(this);
#endif
	
	// Axis 
	setAxisTitle(QwtPlot::xBottom, "Time/seconds");
	setAxisScale(QwtPlot::xBottom, 0, 100);
	
	setAxisTitle(QwtPlot::yLeft, "Values");
	setAxisScale(QwtPlot::yLeft, -0.01, 2);
	
	setTimerInterval(0.0);
	
	init = false;
	Max_y = 0.0;
	Max_z = 0.0;
}

//
//  Set a plain canvas frame and align the scales to it
//
void DataPlot::alignScales()
{
	// The code below shows how to align the scales to
	// the canvas frame, but is also a good example demonstrating
	// why the spreaded API needs polishing.
	
	canvas()->setFrameStyle(QFrame::Box | QFrame::Plain );
	canvas()->setLineWidth(1);
	
	for ( int i = 0; i < QwtPlot::axisCnt; i++ )
	{
		QwtScaleWidget *scaleWidget = (QwtScaleWidget *)axisWidget(i);
		if ( scaleWidget )
			scaleWidget->setMargin(0);
		
		QwtScaleDraw *scaleDraw = (QwtScaleDraw *)axisScaleDraw(i);
		if ( scaleDraw )
			scaleDraw->enableComponent(QwtAbstractScaleDraw::Backbone, false);
	}
}

void DataPlot::setTimerInterval(double ms)
{
	d_interval = qRound(ms);
	
	if ( d_timerId >= 0 )
	{
		killTimer(d_timerId);
		d_timerId = -1;
	}
	if (d_interval >= 0 )
		d_timerId = startTimer(d_interval);
}

void DataPlot::setMax(double max)
{
	setAxisScale(QwtPlot::yLeft, 0.0,max);
	setAxisScale(QwtPlot::yRight, 0.0,max);
}

//  Generate new values 
void DataPlot::timerEvent(QTimerEvent *)
{
	static double phase = 0.0;
	
	if (phase > (M_PI - 0.0001)) 
		phase = 0.0;
	
	// y moves from left to right:
	// Shift y array right and assign new value to y[0].
	
	for ( int i = DATA_PLOT_SIZE - 1; i > 0; i-- )
		d_y[i] = d_y[i-1];
	
	for ( int i = DATA_PLOT_SIZE - 1; i > 0; i-- )
		d_z[i] = d_z[i-1];
	
	d_y[0] = ENV.getDouble(Env::temperatureStart);
	//d_y[0] = PlanEnv->getDouble(PlanParam::costTraj);
	Max_y = *std::max_element(d_y,d_y+DATA_PLOT_SIZE);
	
	d_z[0] = ENV.getDouble(Env::temperatureGoal);
	Max_z = *std::max_element(d_z,d_z+DATA_PLOT_SIZE);
	
	if( Max_y > Max_z )
	{
		setMax(Max_y);
	}
	else
	{
		setMax(Max_z);
	}
	
	
	if(init==false)
	{
		init = true;
		Max_y =  d_y[0];
		Max_z =  d_y[0];
	}
	// update the display
	replot();
	
	phase += M_PI * 0.02;
}
